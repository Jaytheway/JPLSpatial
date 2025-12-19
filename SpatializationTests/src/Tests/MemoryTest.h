//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
//
//   Permission to use, copy, modify, and/or distribute this software for any purpose with or
//   without fee is hereby granted, provided that the above copyright notice and this permission
//   notice appear in all copies. THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
//   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
//   AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
//   CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
//   CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#pragma once

#include "JPLSpatial/Core.h"
#include "JPLSpatial/ErrorReporting.h"
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Memory/Memory.h"
#include "JPLSpatial/SpatialManager.h"
#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/Math/Position.h"
#include "JPLSpatial/Services/PanningService.h"

#include "../Utility/TestMemoryLeakDetector.h"

#include <gtest/gtest.h>

#include <format>
#include <memory_resource>
#include <utility>
#include <vector>
#include <memory>

namespace JPL
{
	class MemoryTest : public testing::Test
	{
	protected:
		using Vec3 = MinimalVec3;

		MemoryTest() = default;

		// Using stack allocator to bypass heap that we're testing
		std::byte mBuffer[1024];
		std::pmr::monotonic_buffer_resource mBufResource{ &mBuffer, sizeof(mBuffer), std::pmr::null_memory_resource()};
		std::pmr::polymorphic_allocator<> mStackAllocator{ &mBufResource };

		PmrMallocResource mPmrStdMalloc;
		PmrMallocResource mPmrJPLMalloc;
		CountingResource mPmrStdCounter{ &mPmrStdMalloc };
		CountingResource mPmrJPLCounter{ &mPmrStdMalloc };

		ScopedStdPmrResourceOverride* mPmrStdOverride = nullptr;
		ScopedGlobalMemoryResource* mPmrJPLOverride = nullptr;
#if JPL_TEST_GLOBAL_NEW_LEAKS
		ScopedNewDeleteCounter* mNewDeleteOverride = nullptr;
		std::size_t mDefaultNewDeleteInUse = 0;
		std::size_t mDefaultNewDeleteMaxUsage = 0;
#endif

		void SetUp() override
		{
			// Reset stack allocator
			mBufResource.release();

			// Reset counters
			mPmrStdCounter.InUse = 0;
			mPmrStdCounter.MaxUsage = 0;
			mPmrJPLCounter.InUse = 0;
			mPmrJPLCounter.MaxUsage = 0;

			// Override default global std pmr
			mPmrStdOverride = mStackAllocator.new_object<ScopedStdPmrResourceOverride>(&mPmrStdCounter);
			// Override default JPLSpatial pmr
			mPmrJPLOverride = mStackAllocator.new_object<ScopedGlobalMemoryResource>(&mPmrJPLCounter);

#if JPL_TEST_GLOBAL_NEW_LEAKS
			// Insert our counter into global new/delete overide.
			// This includes:
			// - all pmr stuff using default resource
			// - global new/delete
			// - std::allocator (internally calls new/delete)
			mNewDeleteOverride = mStackAllocator.new_object<ScopedNewDeleteCounter>();
#endif

			// Sanity check
			ASSERT_TRUE(std::pmr::get_default_resource() == &mPmrStdCounter);
			ASSERT_TRUE(GetDefaultMemoryResource() == &mPmrJPLCounter);
			ASSERT_NE(std::pmr::get_default_resource(), GetDefaultMemoryResource());
		}

		struct MemoryExpectations
		{
			std::size_t NewDeleteInUse = 0;
			std::size_t NewDeleteMaxUsage = 0;

			std::size_t PmrStdInUse = 0;
			std::size_t PmrStdMaxUsage = 0;

			std::size_t PmrJPLInUse = 0;
			std::size_t PmrJPLMaxUsage = 0;

			static constexpr std::size_t NotZero = ~std::size_t(0);
		};

		void Check(const MemoryExpectations& expected)
		{
#if JPL_TEST_GLOBAL_NEW_LEAKS
			// Get the counters before resetting the override object
			mNewDeleteOverride->GetCounters(mDefaultNewDeleteInUse, mDefaultNewDeleteMaxUsage);
#endif

			// - InUse = 0: indicates leaks, either deallocated from different memory resource
			// or not deallocated at all.
			// 
			// - MaxUsage: shows if any allocations happened in the given context.

#if JPL_TEST_GLOBAL_NEW_LEAKS
			// Default new/delete spill
			if (expected.NewDeleteInUse == MemoryExpectations::NotZero)
				EXPECT_GT(mDefaultNewDeleteInUse, 0) << "Allocations bypassed PMR and leaked.";
			else
				EXPECT_EQ(mDefaultNewDeleteInUse, expected.NewDeleteInUse) << "Allocations bypassed PMR and leaked.";

			if (expected.NewDeleteMaxUsage == MemoryExpectations::NotZero)
				EXPECT_GT(mDefaultNewDeleteMaxUsage, 0) << "Allocations bypassed PMR.";
			else
				EXPECT_EQ(mDefaultNewDeleteMaxUsage, expected.NewDeleteMaxUsage) << "Allocations bypassed PMR.";
#endif
	
			// Pmr Std spill
			if (expected.PmrStdInUse == MemoryExpectations::NotZero)
				EXPECT_GT(mPmrStdCounter.InUse, 0) << "PMR allocations bypassed JPL PMR and leaked.";
			else
				EXPECT_EQ(mPmrStdCounter.InUse, expected.PmrStdInUse) << "PMR allocations bypassed JPL PMR and leaked.";

			if (expected.PmrStdMaxUsage == MemoryExpectations::NotZero)
				EXPECT_GT(mPmrStdCounter.MaxUsage, 0) << "PMR allocations bypassed JPL PMR.";
			else
				EXPECT_EQ(mPmrStdCounter.MaxUsage, expected.PmrStdMaxUsage) << "PMR allocations bypassed JPL PMR.";

			// JPL Pmr Std
			if (expected.PmrJPLInUse == MemoryExpectations::NotZero)
				EXPECT_GT(mPmrJPLCounter.InUse, 0) << "JPL PMR allocations leaked.";
			else
				EXPECT_EQ(mPmrJPLCounter.InUse, expected.PmrJPLInUse) << "JPL PMR allocations leaked.";
			
			if (expected.PmrJPLMaxUsage == MemoryExpectations::NotZero)
				EXPECT_GT(mPmrJPLCounter.MaxUsage, 0) << "JPL PMR allocations detected.";
			else
				EXPECT_EQ(mPmrJPLCounter.MaxUsage, expected.PmrJPLMaxUsage) << "JPL PMR allocations detected.";
		}

		void TearDown() override
		{
			// Revert overrides

#if JPL_TEST_GLOBAL_NEW_LEAKS
			mStackAllocator.delete_object(mNewDeleteOverride);
			mNewDeleteOverride = nullptr;
#endif
			mStackAllocator.delete_object(mPmrJPLOverride);
			mPmrJPLOverride = nullptr;

			mStackAllocator.delete_object(mPmrStdOverride);
			mPmrStdOverride = nullptr;
		}
	};

#if JPL_TEST_GLOBAL_NEW_LEAKS
	TEST_F(MemoryTest, NewDelete_BypassesJPLPmr)
	{
		auto* v = new Vec3();
		Check(
			MemoryExpectations{
				.NewDeleteInUse = sizeof(Vec3),
				.NewDeleteMaxUsage = sizeof(Vec3)
			});

		delete v;
		Check(
			MemoryExpectations{
				.NewDeleteInUse = 0,
				.NewDeleteMaxUsage = sizeof(Vec3)
			});
	}
#endif

	TEST_F(MemoryTest, DefaultNewDefaultDelete_UsesJPLPmr)
	{
		auto* v = DefaultNew<Vec3>();
		Check(
			MemoryExpectations{
				.PmrJPLInUse = sizeof(Vec3),
				.PmrJPLMaxUsage = sizeof(Vec3)
			});

		DefaultDelete(v);
		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = sizeof(Vec3)
			});
	}

	TEST_F(MemoryTest, make_pmr_shared_DefaultConstructed_UsesPmr)
	{
		{
			std::shared_ptr<Vec3> sp = make_pmr_shared<Vec3>();
			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero, // size can vary
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			});
	}

	TEST_F(MemoryTest, make_pmr_shared_DefaultNewPtrClaimed_UsesPmr)
	{
		{
			auto* v = DefaultNew<Vec3>();
			std::shared_ptr<Vec3> sp = make_pmr_shared(v);
			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero, // size can vary
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			});
	}

	TEST_F(MemoryTest, make_pmr_shared_reset_pmr_shared_UsesPmr)
	{
		{
			auto* v = DefaultNew<Vec3>();
			std::shared_ptr<Vec3> sp;
			reset_pmr_shared(sp, v);
			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero, // size can vary
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			});
	}

	TEST_F(MemoryTest, make_pmr_shared_reset_pmr_shared_ToNull_UsesPmr)
	{
		{
			std::shared_ptr<Vec3> sp = make_pmr_shared<Vec3>();
			reset_pmr_shared(sp);
			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero, // control block may be still alive
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			});
	}

	TEST_F(MemoryTest, make_pmr_shared_copy_PropagatesPmr)
	{
		{
			std::shared_ptr<Vec3> sp = make_pmr_shared<Vec3>();
			std::shared_ptr<Vec3> sp2 = sp;
			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero, // size can vary
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			});
	}

	TEST_F(MemoryTest, make_pmr_shared_move_PropagatesPmr)
	{
		{
			std::shared_ptr<Vec3> sp = make_pmr_shared<Vec3>();
			std::shared_ptr<Vec3> sp2 = std::move(sp);
			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero, // size can vary
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			});
	}

	TEST_F(MemoryTest, make_pmr_unique_DefaultConstructed_UsesPmr)
	{
		{
			pmr_unique_ptr<Vec3> up = make_pmr_unique<Vec3>();
			Check(
				MemoryExpectations{
					.PmrJPLInUse = sizeof(Vec3),
					.PmrJPLMaxUsage = sizeof(Vec3)
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = sizeof(Vec3)
			});
	}

	TEST_F(MemoryTest, make_pmr_unique_DefaultNewPtrClaimed_UsesPmr)
	{
		{
			auto* v = DefaultNew<Vec3>();
			pmr_unique_ptr<Vec3> sp(v);
			Check(
				MemoryExpectations{
					.PmrJPLInUse = sizeof(Vec3),
					.PmrJPLMaxUsage = sizeof(Vec3)
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = sizeof(Vec3)
			});
	}

	TEST_F(MemoryTest, make_pmr_unique_Reset_UsesPmr)
	{
		{
			auto* v = DefaultNew<Vec3>();
			pmr_unique_ptr<Vec3> sp;
			sp.reset(v);
			Check(
				MemoryExpectations{
					.PmrJPLInUse = sizeof(Vec3),
					.PmrJPLMaxUsage = sizeof(Vec3)
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = sizeof(Vec3)
			});
	}

	TEST_F(MemoryTest, make_pmr_unique_ReleaseDefaultDelete_NoLeaks)
	{
		{
			pmr_unique_ptr<Vec3> sp = make_pmr_unique<Vec3>();
			Check(
				MemoryExpectations{
					.PmrJPLInUse = sizeof(Vec3), // size can vary
					.PmrJPLMaxUsage = sizeof(Vec3)
				});

			Vec3* v = sp.release();
			DefaultDelete(v);
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = sizeof(Vec3)
			});
	}

	TEST_F(MemoryTest, StdVector_NoPrm)
	{
		{
			std::vector<Vec3> vec;
			vec.resize(24, Vec3(0, 0, 0));
			Check(
				MemoryExpectations{
					.NewDeleteInUse = MemoryExpectations::NotZero,
					.NewDeleteMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.NewDeleteInUse = 0,
				.NewDeleteMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, StdPmrVector_NoJPLPrm)
	{
		{
			std::pmr::vector<Vec3> vec; // uses std::pmr::get_default_resource(), which is new_delete_resoure
			vec.resize(24, Vec3(0, 0, 0));
			Check(
				MemoryExpectations{
					.PmrStdInUse = MemoryExpectations::NotZero,
					.PmrStdMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.PmrStdInUse = 0,
				.PmrStdMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, JPLPmrVector_UsesJPLPrm)
	{
		{
			std::pmr::vector<Vec3> vec(GetDefaultMemoryResource());
			vec.resize(24, Vec3(0, 0, 0));
			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, JPLPmrVector_DoesNOTPropagateOnCopyAssignment) // TODO: because of this we need to add our mem checks to bigger tests
	{
		{
			std::pmr::vector<Vec3> vec(GetDefaultMemoryResource());
			vec.resize(24, Vec3(0, 0, 0));

			// vec2 will default constructed pmr allocator
			std::pmr::vector<Vec3> vec2 = vec;

			ASSERT_NE(vec2.get_allocator().resource(), vec.get_allocator().resource());

			Check(
				MemoryExpectations{
					.PmrStdInUse = MemoryExpectations::NotZero,
					.PmrStdMaxUsage = MemoryExpectations::NotZero,
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.PmrStdInUse = 0,
				.PmrStdMaxUsage = MemoryExpectations::NotZero,
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, JPLPmrVector_PropagatesOnCopyConstructorWithAllocator)
	{
		{
			std::pmr::vector<Vec3> vec(GetDefaultMemoryResource());
			vec.resize(24, Vec3(0, 0, 0));

			std::pmr::vector<Vec3> vec2(vec, vec.get_allocator());

			ASSERT_EQ(vec2.get_allocator().resource(), vec.get_allocator().resource());

			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, JPLPmrVector_PropagatesOnMoveAssignment)
	{
		{
			std::pmr::vector<Vec3> vec(GetDefaultMemoryResource());
			vec.resize(24, Vec3(0, 0, 0));

			std::pmr::vector<Vec3> vec2 = std::move(vec);
			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, JPLPmrVector_PropagatesOnMoveConstructor)
	{
		{
			std::pmr::vector<std::shared_ptr<Vec3>> vec(GetDefaultMemoryResource());
			vec.resize(24);

			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, JplPmrVector_PropagatesToSharedPtrElements)
	{
		{
			std::pmr::vector<Vec3> vec(GetDefaultMemoryResource());
			vec.resize(24, Vec3(0, 0, 0));

			std::pmr::vector<Vec3> vec2(std::move(vec));

			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, FlatMap_SubscriptOperator_PropagatesToElements)
	{
		{
			FlatMapWithAllocator<int, std::pmr::vector<Vec3>, std::pmr::polymorphic_allocator> map(GetDefaultMemoryResource());
			map[0].push_back(Vec3(0, 0, 0));

			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, FlatMap_SharedPtr_SubscriptOperator_PropagatesToElements)
	{
		{
			FlatMapWithAllocator<int, std::shared_ptr<Vec3>, std::pmr::polymorphic_allocator> map(GetDefaultMemoryResource());
			auto& ptr = map[0];
			ptr = make_pmr_shared<Vec3>();

			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, FlatMap__Emplace_DoesNOTPropagateToElementsConsructor)
	{
		{
			FlatMapWithAllocator<int, std::pmr::vector<Vec3>, std::pmr::polymorphic_allocator> map(GetDefaultMemoryResource());

			// What happens here:
			// 1. Vector is created with std pmr
			// 2. Then jpl pmr is used within the map
			map.emplace(0, std::pmr::vector<Vec3>{ Vec3(0, 0, 0) });
			// ..memory resource has to be passed to vector constructor to avoid spillage into std pmr

			// ..still jpl pmr here
			map[0].push_back(Vec3(0, 0, 0));

			Check(
				MemoryExpectations{
					.PmrStdInUse = 0,
					.PmrStdMaxUsage = MemoryExpectations::NotZero,
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				}
			);
		}
		Check(
			MemoryExpectations{
				.PmrStdInUse = 0,
				.PmrStdMaxUsage = MemoryExpectations::NotZero,
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			}
		);
	}

	TEST_F(MemoryTest, PanningService_UsesJPLPmrEverywhere)
	{
		{
			const auto sourceChannelMap = ChannelMap::FromNumChannels(2);
			const auto targetChannelMap = ChannelMap::FromNumChannels(4);

			PanningService<VBAPBaseTraits<Vec3>> panningService;

			const auto newHandle = PanEffectHandle::New();

			using SourceLayout = typename PanningService<VBAPBaseTraits<Vec3>>::SourceLayout;
			std::shared_ptr<const SourceLayout> panningData =
				panningService.CreatePanningDataFor(
					SourceLayoutKey{
						.SourceMap = sourceChannelMap,
						.TargetMap = targetChannelMap
					}, newHandle);

			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			});
	}

	//! Do not enable this, this is just an illustration of a leaky case with pmr and inheritance
#if 0 
	TEST_F(MemoryTest, InheritanceLeak)
	{
		{
			struct Base
			{
				virtual ~Base() = default;
			};

			struct Derived : public Base
			{
				float v;
			};

			auto* derivedRaw = DefaultNew<Derived>();

			auto makeSharedConceal = [](Base* base)
			{
				return make_pmr_shared(base); // Makes Deleter/Deallocator for the base type
			};

			std::shared_ptr<Base> basePtr = makeSharedConceal(derivedRaw);

			// THIS IS FINE (makes Deleter/Deallcoator for Derived
			//std::shared_ptr<Base> basePtr = make_pmr_shared(derivedRaw);

			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0, // THIS FAILS, PmrJPLInUse is 8 bytes (8 bytes with VTable, 3 bytes without)
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			});
	}
#endif

	TEST_F(MemoryTest, JPLSpatialManager_CreateSource_NoLeaks)
	{
		{
			SpatialManager<Vec3> spatializer;
			const ChannelMap quadChannels = ChannelMap::FromChannelMask(ChannelMask::Quad);

			for (int i = 0; i < 10; ++i)
			{
				SourceId source = spatializer.CreateSource(
					SourceInitParameters{
						.NumChannels = 1,
						.NumTargetChannels = quadChannels.GetNumChannels(),
						.PanParameters = {.Focus = 0.0f, .Spread = 0.0f }
					});
			}
			
			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			});
	}

	TEST_F(MemoryTest, JPLSpatialUsesJPLPmrEverywhere)
	{
		{
			SpatialManager<Vec3> spatializer;
			const ChannelMap quadChannels = ChannelMap::FromChannelMask(ChannelMask::Quad);

			SourceId source = spatializer.CreateSource(SourceInitParameters{
														.NumChannels = 1,
														.NumTargetChannels = quadChannels.GetNumChannels()
													   });
			ASSERT_TRUE(source.IsValid());

			Position<Vec3> sourcePos{
				.Location = Vec3(0, 10, 10),
				.Orientation = OrientationData<Vec3>::Identity()
			};
			ASSERT_TRUE(spatializer.SetSourcePosition(source, sourcePos));

			const std::pmr::vector<typename AttenuationCurve::Point> attenuationCurvePoints
			{ {
					{.Distance = 0.0f, .Value = 1.0f, .FunctionType = Curve::EType::Linear},
					{.Distance = 10.0f, .Value = 0.5f, .FunctionType = Curve::EType::Linear}
			}, GetDefaultMemoryResource() };

			auto curve = make_pmr_shared<AttenuationCurve>();
			curve->Points = attenuationCurvePoints;
			curve->SortPoints();

			AttenuationCurveRef attenuationCurve =
				spatializer
				.GetDirectPathService()
				.AssignAttenuationCurve(spatializer.GetDirectEffectHandle(source), curve);

			ASSERT_TRUE(attenuationCurve != nullptr);

			const auto* panner = spatializer.GetPanningService().CreatePannerFor(quadChannels);
			ASSERT_TRUE(panner != nullptr);

			// Process all the data
			spatializer.AdvanceSimulation();

			// Test attenuation
			[[naybe_unused]] const float attenuation = spatializer.GetDistanceAttenuation(source, attenuationCurve);

			const auto channelGains = spatializer.GetChannelGains(source, quadChannels);
			ASSERT_FALSE(channelGains.empty());

			Check(
				MemoryExpectations{
					.PmrJPLInUse = MemoryExpectations::NotZero,
					.PmrJPLMaxUsage = MemoryExpectations::NotZero
				});
		}

		Check(
			MemoryExpectations{
				.PmrJPLInUse = 0,
				.PmrJPLMaxUsage = MemoryExpectations::NotZero
			});
	}
} // namespace JPL