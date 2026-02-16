//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
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

#include <bit>
#include <concepts>
#include <stdint.h>

namespace JPL
{
	namespace RingImpl
	{
		enum class EOperationMode
		{
			Modulo,			// wrap Ring using modulo % operator, ring can be any size
			Pow2BitManip,	// wrap Ring using bit manipulation, requires Ring size to be power of 2
		};

		enum class EWriteDirection
		{
			Forward,	// increment index on write
			Backward,	// decrement index on write
		};

		template<class T> concept COperationMode = std::same_as<T, EOperationMode>;
		template<class T> concept CWriteDirection = std::same_as<T, EWriteDirection>;

		/// Base class that implements the wrap around logic for the OperationMode
		template<COperationMode auto OperationMode>
		class AbstractRingWrap;

		/// Base class for Ring-like indexing operations
		template<CWriteDirection auto WriteDirection, COperationMode auto OperationMode>
		class AbstarctRingBase;
	}

	//==============================================================================
	/// Forward declaration
	template<uint32_t MaxWindow,
		RingImpl::CWriteDirection auto WriteDirection,
		RingImpl::COperationMode auto OperationMode>
	class AbstractRingIndex;

	//==============================================================================
	/// Ring index writing forward/backward, using bit manipulations for wrap around.
	/// Much more efficient than non-Fast versions at a cost of one extra int stored
	/// and requirement of having a ring size power of 2.
	/// Index size: 12-16 bytes (MaxWindow <= 1 ? 12 : 16)
	template<uint32_t MaxWindow> using RingIndexForwardFast = AbstractRingIndex<MaxWindow, RingImpl::EWriteDirection::Forward, RingImpl::EOperationMode::Pow2BitManip>;
	template<uint32_t MaxWindow> using RingIndexBackwardFast = AbstractRingIndex<MaxWindow, RingImpl::EWriteDirection::Backward, RingImpl::EOperationMode::Pow2BitManip>;
	
	/// Ring index writing forward/backward, using modulo operator for wrap around.
	/// Less efficient than the Fast versions, but doestn't require ring size power of 2.
	/// Index size: 8-12 bytes (MaxWindow <= 1 ? 8 : 12)
	template<uint32_t MaxWindow> using RingIndexForward = AbstractRingIndex<MaxWindow, RingImpl::EWriteDirection::Forward, RingImpl::EOperationMode::Modulo>;
	template<uint32_t MaxWindow> using RingIndexBackward = AbstractRingIndex<MaxWindow, RingImpl::EWriteDirection::Backward, RingImpl::EOperationMode::Modulo>;

	//==============================================================================
	/// Class representing an index of a ring buffer
	/// with an optional mirror index to accommodate
	/// minimum length of congiguous data.
	template<uint32_t MaxWindow,
		RingImpl::CWriteDirection auto WriteDirection,
		RingImpl::COperationMode auto OperationMode>
	class AbstractRingIndex : private RingImpl::AbstarctRingBase<WriteDirection, OperationMode>
	{
	private:
		using Base = RingImpl::AbstarctRingBase<WriteDirection, OperationMode>;

	public:
		struct Index
		{
			uint32_t WriteIndex;
			uint32_t MirrorIndex;
		};

	public:
		explicit AbstractRingIndex(uint32_t minSize)
			: Base(minSize)
			, mIndex(0, Base::GetMirrorIndex(0, MaxWindow))
		{
		}

		/// Get total size of the ring buffer including MaxWindow mirror length
		[[nodiscard]] inline uint32_t GetTotalSize() const { return Base::mRing + MaxWindow; }
		[[nodiscard]] inline uint32_t GetRingSize() const { return Base::mRing; }
		[[nodiscard]] inline Index GetCurrent() const { return mIndex; }
		inline void Reset() { UpdateWriteIndex(0); }

		inline Index operator ++() requires(WriteDirection == RingImpl::EWriteDirection::Forward)
		{
			const uint32_t newWriteIndex = Base::IncrementIndex(mIndex.WriteIndex);
			UpdateWriteIndex(newWriteIndex);
			return mIndex;
		}

		inline Index operator ++(int) requires(WriteDirection == RingImpl::EWriteDirection::Forward)
		{
			const Index currentIndex = mIndex;
			const uint32_t newWriteIndex = Base::IncrementIndex(mIndex.WriteIndex);
			UpdateWriteIndex(newWriteIndex);
			return currentIndex;
		}

		inline Index operator --() requires(WriteDirection == RingImpl::EWriteDirection::Backward)
		{
			const uint32_t newWriteIndex = Base::DecrementIndex(mIndex.WriteIndex);
			UpdateWriteIndex(newWriteIndex);
			return mIndex;
		}

		inline Index operator --(int) requires(WriteDirection == RingImpl::EWriteDirection::Backward)
		{
			const Index currentIndex = mIndex;
			const uint32_t newWriteIndex = Base::DecrementIndex(mIndex.WriteIndex);
			UpdateWriteIndex(newWriteIndex);
			return currentIndex;
		}

		/// Get the index of the sample at an offset from the WriteIndex.
		[[nodiscard]] inline uint32_t GetOffset(uint32_t offset) const
		{
			// start of contiguous `MaxWindow` window at an offset from the WriteIndex
			return Base::GetOffset(mIndex.WriteIndex, offset);
		}

	private:
		inline void UpdateWriteIndex(uint32_t newIndex)
		{
			mIndex.WriteIndex = newIndex;
			mIndex.MirrorIndex = Base::GetMirrorIndex(newIndex, MaxWindow);
		}

	private:
		Index mIndex;
	};

	//==============================================================================
	/// Special case for 0-1 MaxWindow doesn't require mirror index
	template<uint32_t MaxWindow,
		RingImpl::CWriteDirection auto WriteDirection,
		RingImpl::COperationMode auto OperationMode> requires(MaxWindow <= 1)
		class AbstractRingIndex<MaxWindow, WriteDirection, OperationMode> : private RingImpl::AbstarctRingBase<WriteDirection, OperationMode>
	{
	private:
		using Base = RingImpl::AbstarctRingBase<WriteDirection, OperationMode>;

	public:
		using Index = uint32_t;

	public:
		explicit AbstractRingIndex(uint32_t minSize)
			: Base(minSize)
			, mIndex(0)
		{
		}

		/// Get total size of the ring buffer 
		[[nodiscard]] inline uint32_t GetTotalSize() const { return Base::mRing; }
		[[nodiscard]] inline uint32_t GetRingSize() const { return Base::mRing; }
		[[nodiscard]] inline Index GetCurrent() const { return mIndex; }
		inline void Reset() { mIndex = { 0 }; }

		inline Index operator ++()
		{
			mIndex = Base::IncrementIndex(mIndex);
			return mIndex;
		}

		inline Index operator ++(int)
		{
			const Index currentIndex = mIndex;
			mIndex = Base::IncrementIndex(mIndex);
			return currentIndex;
		}

		inline Index operator --()
		{
			mIndex = Base::DecrementIndex(mIndex);
			return mIndex;
		}

		inline Index operator --(int)
		{
			const Index currentIndex = mIndex;
			mIndex = Base::DecrementIndex(mIndex);
			return currentIndex;
		}

		[[nodiscard]] inline uint32_t GetOffset(int32_t offset) const
		{
			return Base::GetOffset(mIndex, offset);
		}

	private:
		Index mIndex;
	};
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================
namespace JPL
{
	namespace RingImpl
	{
		template<> class AbstractRingWrap<EOperationMode::Pow2BitManip>
		{
		protected:
			explicit AbstractRingWrap(uint32_t minSize) : mRing(std::bit_ceil(minSize)), mSizeMask(mRing - 1) {}
			inline uint32_t Wrap(uint32_t index) const { return (index & mSizeMask); }

		protected:
			const uint32_t mRing;
			const uint32_t mSizeMask;
		};

		template<> class AbstractRingWrap<EOperationMode::Modulo>
		{
		protected:
			explicit AbstractRingWrap(uint32_t minSize) : mRing(minSize) {}
			inline uint32_t Wrap(uint32_t index) const { return (index % mRing); }

		protected:
			const uint32_t mRing;
		};

		template<CWriteDirection auto WriteDirection, COperationMode auto OperationMode>
		class AbstarctRingBase : protected AbstractRingWrap<OperationMode>
		{
		private:
			using WrapBase = AbstractRingWrap<OperationMode>;

		protected:
			explicit AbstarctRingBase(uint32_t minSize) : WrapBase(minSize) {}

			inline uint32_t IncrementIndex(uint32_t index) const requires(WriteDirection == EWriteDirection::Forward)
			{
				return WrapBase::Wrap(index + 1u);
			}

			inline uint32_t DecrementIndex(uint32_t index) const requires(WriteDirection == EWriteDirection::Backward)
			{
				return WrapBase::Wrap((int32_t(index) - 1));
			}

			/// Get the index of the sample at an offset from the WriteIndex.
			inline uint32_t GetOffset(uint32_t index, uint32_t offset) const
			{
				if constexpr (WriteDirection == EWriteDirection::Forward)
				{
					return WrapBase::Wrap(index + WrapBase::mRing - offset);
				}
				else
				{
					return WrapBase::Wrap(index + WrapBase::mRing + offset);
				}
			}

			/// @param index: index of the sample to mirror, must be < mRing;
			inline uint32_t GetMirrorIndex(uint32_t index, uint32_t maxWindow) const
			{
				// (branch version)
				//if (index < maxWindow) // mirror only if inside first 'maxWindow'
				//    mirrorIndex = return mRing + index;

				// (branch-less simple version)
				// mirrorIndex = (index < maxWindow) * mRing + index;

				// (branch-less clever version)
				// This logic works for both, power of 2 mRing and non-power of 2 mRing.
				const uint32_t mirrorOff = (int32_t(index - maxWindow) >> 31) & WrapBase::mRing; // 0 or mRing
				return (index + mirrorOff);
			}
		};
	} // namespace RingImpl
} // namespace JPL
