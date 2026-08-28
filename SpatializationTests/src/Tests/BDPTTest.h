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
#include "JPLSpatial/PathTracing/BDPT.h"
#include "JPLSpatial/PathTracing/EnergyResponse.h"
#include "JPLSpatial/PathTracing/Math.h"

#include "../Utility/TestUtils.h"

#include <gtest/gtest.h>
#include <functional>
#include <random>
#include <format>

// TODO: link properly
#include "../Utility/EnergyHistogramIO.h"
#include "../Utility/BoxTest.h"
#include "../Utility/SceneInterfaceUtils.h"

namespace JPL
{
	class BDPTTest : public testing::Test
	{
	protected:
		BDPTTest() = default;

		using Vec3 = typename BDPTDefaultTraits::Vec3;

	protected:
		
		struct SourceInterface
		{
			std::function<Vec3()> Position = [] { return Vec3::Zero(); };
			std::function<Vec3()> RayDirection = [] { return TestUtil::Dir::Forward; };

			TestUtil::SceneInterfaceMock::Ray SampleRay() const
			{
				return TestUtil::SceneInterfaceMock::Ray{
					.Origin = Position(),
					.Direction = RayDirection()
				};
			}

			float GetIntencity() const
			{
				return 1.0f;
			}
		};

		struct ListenerInterface
		{
			std::function<Vec3()> Position = [] { return Vec3::Zero(); };
			std::function<Vec3()> RayDirection = [] { return TestUtil::Dir::Forward; };

			TestUtil::SceneInterfaceMock::Ray SampleRay() const
			{
				return TestUtil::SceneInterfaceMock::Ray{
					.Origin = Position(),// RandVec() * 10.0f,
					.Direction = RayDirection()
				};
			};

			Vec3 TransformDirection(const Vec3& inDirection) const
			{
				static const Vec3 right(1.0f, 0.0f, 0.0f);
				static const Vec3 up(0.0f, 1.0f, 0.0f);
				static const Vec3 forward(0.0f, 0.0f, -1.0f);

				return Vec3(
					DotProduct(inDirection, right),
					DotProduct(inDirection, up),
					DotProduct(inDirection, forward)
				);
			}
		};
	};

#if 1
	TEST_F(BDPTTest, EmptyTracer)
	{
		/*const BDPTParameters params{
			.SubPathBudget = 10,
			.MaxBounces = 4,
			.TotalSamples = 200,
			.SampleBinDt = 0.5f,
			.EnergyResponseTime = 2.0f
		};*/

		// More realistic scenario
		const BDPTParameters params{
			.SubPathBudget = 100,
			.MaxBounces = 20,
			.TotalSamples = 50'000,
			.SampleBinDt = 0.003f,
			.EnergyResponseTime = 2.0f
		};

		// Minimal test
		/*const BDPTParameters params{
			.SubPathBudget = 10,
			.MaxBounces = 8,
			.TotalSamples = 10 * 10 * 8,
			.SampleBinDt = 0.005f,
			.EnergyResponseTime = 2.0f
		};*/

		BDPT<> bdpt(params);

		TestUtil::SceneInterfaceMock dummyScene{
			//.Intersect = IntersectStub{
			//	.GetNormal = GetNormalFuncs::ReflectBackRnd,
			//	//.GetPosition = GetPositionFuncs::TimeDistance(params.EnergyResponseTime * 0.25f),
			//	//.GetPosition = GetPositionFuncs::FixedDistance(10.0f),
			//	.GetPosition = GetPositionFuncs::RandomDistance(1.0f, 100.0f),
			//},
			.RayIntersect = TestUtil::IntersectStub{
				//! To make the multi-trace test fair for the algorithm, we need to simulate the same geometry, e.g. a box room
				.CastRay = TestUtil::CastRayFuncs::Box({ Vec3::Zero(), Vec3{ 50.0f, 50.0f, 50.0f } })
			},
			.SampleTraceDirection = TestUtil::NextDirectionFunc::VectorBasedScattering
		};


		SourceInterface dummySource{
			//.Position = Dir::Forward * (params.EnergyResponseTime * 0.25f * JPL_SPEED_OF_SOUND),
			.Position = [] { return  TestUtil::Dir::Forward * 10.0f; }, // source is 10 m forward
			.RayDirection = TestUtil::RandUnitVec
		};

		ListenerInterface dummyListener{
			.Position = [] { return Vec3::Zero(); },
			//.RayDirection = [] { return Normalize(Dir::Left + Dir::Forward); }
			.RayDirection = TestUtil::RandUnitVec
		};

		
		// 2D histrogram for debugging
		EnergyHistogram energyResponse(params.SampleBinDt, params.EnergyResponseTime);

		static constexpr int numFrames = 4;

		Counters counters;
		std::vector<float> averageSNRs(numFrames, 0.0f);


		struct ERStats
		{
			using TracerType = std::remove_cvref_t<decltype(bdpt)>;

			// For all frames
			std::vector<int> PathCount;
			std::vector<MinMaxAvgSum> EnergiesMMA;
			std::vector<MinMaxAvgSum> VarianceMMA;
			std::vector<MinMaxAvgSum> SNRMMA;

			void resize(size_t newSize)
			{
				EnergiesMMA.resize(newSize);
				PathCount.resize(newSize);
				VarianceMMA.resize(newSize);
				SNRMMA.resize(newSize);
			}

			void Extract(const TracerType& tracer, uint32_t frame)
			{
				auto extractEnergy = [](auto erMapIterator)
				{
					const auto& [key, slotAndVariance] = erMapIterator;
					const auto& [slot, variance] = slotAndVariance;
					return slot.Energy;
				};

				const auto& erData = tracer.testERData;
				EnergiesMMA[frame] = MinMaxAvgSum::Of(erData, extractEnergy);

				ExtractERData(erData);

				PathCount[frame] = static_cast<int>(erData.size());
				VarianceMMA[frame] = MinMaxAvgSum::Of(Variance);
				SNRMMA[frame] = MinMaxAvgSum::Of(SNR);
			}

			// Per frame, volatile
			std::vector<float> Energy;
			std::vector<float> Variance;
			std::vector<float> SNR;

		private:
			void ExtractERData(const TracerType::ERMapType& erMap)
			{
				using ERSlot = typename decltype(bdpt)::ERSlot;
				using PairType = std::pair<ERSlot, Variance::Online>;

				std::vector<PairType> vars;
				for (const auto& [key, slotAndVariance] : erMap)
					vars.push_back(slotAndVariance);

				// Sort by time
				std::sort(vars.begin(), vars.end(), [](const PairType& lhs, const PairType& rhs)
				{
					return lhs.first.ArrivalTime < rhs.first.ArrivalTime;
				});

				Variance.clear();
				SNR.clear();
				Energy.clear();
				for (const auto& [slot, variance] : vars)
				{
					Variance.push_back(variance.GetVariance());
					SNR.push_back(variance.GetSNR());
					Energy.push_back(slot.Energy);
				}
			}
			
		} erStats;

		erStats.resize(numFrames);

		for (int i = 0; i < numFrames; ++i)
		{
			//? TEMP
			bdpt.testERData.clear();

			bdpt.Trace(dummyScene, dummySource, dummyListener, energyResponse);
			//SaveEnergyHistogramAsPNG(energyResponse, std::format("energy_histogram_frame_{}.png", i + 1));
			SaveHistogramWithSNRAsPNG(energyResponse, std::format("energy_histogram_frame_{}.png", i + 1));

			// Create histogram to debug how many samples contributed to each bin
			ExtractCountersFromVariances(energyResponse.GetVariance(), counters);
			SaveEnergyHistogramAsPNG(counters.CountNormalized, std::format("counters_histogram_frame_{}.png", i + 1));

			// Visualize sample allocation per bounce order
			SaveEnergyHistogramAsPNG(bdpt.GetAllocationRatios(), std::format("allocations_frame_{}.png", i + 1));

			// Late Response data
			averageSNRs[i] = CalculateAverageSNR(energyResponse.GetVariance());
			
			// Early Response data
			erStats.Extract(bdpt, i);
			SaveEnergyHistogramAsPNG(erStats.Variance , std::format("er_variance_frame_{}.png", i + 1));
			SaveEnergyHistogramAsPNG(erStats.SNR , std::format("er_snr_frame_{}.png", i + 1));
			SaveEnergyHistogramAsPNG(erStats.Energy , std::format("er_enrg_frame_{}.png", i + 1));

		}

		using PrinterType = CoutPrinter;
		PrinterType printer;

		auto printMinMaxAvg = [](const MinMaxAvgSum& mma) { return mma.ToString(); };
		auto printMinMaxAvgLP = [](const MinMaxAvgSum& mma) { return mma.ToStringLowPrecision(); };
		
		//printer.PrintLine("With simple ERs:");
		printer.PrintData(averageSNRs, "LR average SNR: {:.3}");
		printer.PrintData(erStats.PathCount, "num ER paths: {}");
		printer.PrintData(erStats.EnergiesMMA, "ER energy: {}", printMinMaxAvg);
		printer.PrintData(erStats.VarianceMMA, "ER variance: {}", printMinMaxAvg);
		printer.PrintData(erStats.SNRMMA, "ER SNR: {}", printMinMaxAvgLP);


		/*
			--- SNR of the Late Response ---
			
			Without ERs (baseline):
				Frame 1, LR average SNR: 0.477
				Frame 2, LR average SNR: 0.399
				Frame 3, LR average SNR: 0.497
				Frame 4, LR average SNR: 0.477


			With simple ERs:
				Frame 1, LR average SNR: 0.375
				Frame 2, LR average SNR: 0.348
				Frame 3, LR average SNR: 0.484
				Frame 4, LR average SNR: 0.38

			With simple ERs & specular rejection (i.e. only diffuse ERs):
				Frame 1, LR average SNR: 0.373
				Frame 2, LR average SNR: 0.347
				Frame 3, LR average SNR: 0.476
				Frame 4, LR average SNR: 0.38

			With simple ERs & only specular:
				Frame 1, LR average SNR: 0.432
				Frame 2, LR average SNR: 0.404
				Frame 3, LR average SNR: 0.496
				Frame 4, LR average SNR: 0.472
			
			With image source ERs:
				Frame 1, LR average SNR: 0.432
				Frame 2, LR average SNR: 0.404
				Frame 3, LR average SNR: 0.498
				Frame 4, LR average SNR: 0.472

			With image source ERs & specular rejection:
				Frame 1, LR average SNR: 0.43
				Frame 2, LR average SNR: 0.401
				Frame 3, LR average SNR: 0.493
				Frame 4, LR average SNR: 0.458
		*/

		/*
			--- Number of ER paths extracted ---

			With simple ERs:
				Frame 1, num ER paths: 50
				Frame 2, num ER paths: 46
				Frame 3, num ER paths: 2177
				Frame 4, num ER paths: 101

			With simple ERs & specular rejection (i.e. only diffuse ERs):
				Frame 1, num ER paths: 34
				Frame 2, num ER paths: 31
				Frame 3, num ER paths: 1955
				Frame 4, num ER paths: 71
				
			With simple ERs & only specular:
				Frame 1, num ER paths: 16
				Frame 2, num ER paths: 14
				Frame 3, num ER paths: 268
				Frame 4, num ER paths: 24
			
			With image source ERs:
				Frame 1, num ER paths: 41
				Frame 2, num ER paths: 42
				Frame 3, num ER paths: 39
				Frame 4, num ER paths: 46

		*/

		//! For image source ERs the SNR is 1 for the first order and 0 after
		//! is due to the fact that the secodary reflections diverge in directinality
		//! and timing so much that there are no similar enough reflection paths
		//! to merge.
		//! This should not be taken as a reduction of quality, because the reflectoins
		//! are deterministic and don't require many samples to get accurate result.

		/*
			--- ER Energy ---

			Image Source ERs:
				Frame 1, ER energy: min: 5.4971e-05 | max:        0.1 | avg:   0.019849 | sum:    0.81381
				Frame 2, ER energy: min: 5.4187e-05 | max:       0.09 | avg:   0.018424 | sum:     0.7738
				Frame 3, ER energy: min: 6.5961e-05 | max:       0.15 | avg:   0.019079 | sum:    0.74408
				Frame 4, ER energy: min: 5.4009e-05 | max:       0.09 | avg:   0.018585 | sum:     0.8549

				Frame 1, ER variance: min:          0 | max:          0 | avg:          0 | sum:          0
				Frame 2, ER variance: min:          0 | max:          0 | avg:          0 | sum:          0
				Frame 3, ER variance: min:          0 | max:          0 | avg:          0 | sum:          0
				Frame 4, ER variance: min:          0 | max:          0 | avg:          0 | sum:          0

				Frame 1, ER SNR: min:     0 | max:     1 | avg: 0.268 | sum:    11
				Frame 2, ER SNR: min:     0 | max:     1 | avg: 0.286 | sum:    12
				Frame 3, ER SNR: min:     0 | max:     1 | avg: 0.308 | sum:    12
				Frame 4, ER SNR: min:     0 | max:     1 | avg: 0.261 | sum:    12

			Simple ERs:
				Frame 1, ER energy: min: 1.5786e-05 | max:  0.0031299 | avg: 0.00065656 | sum:   0.032828
				Frame 2, ER energy: min: 5.0666e-06 | max:  0.0052671 | avg: 0.00095098 | sum:   0.043745
				Frame 3, ER energy: min:          0 | max:  0.0051043 | avg: 1.9885e-05 | sum:    0.04329
				Frame 4, ER energy: min: 2.7583e-06 | max:   0.003684 | avg: 0.00067718 | sum:   0.068396

				Frame 1, ER variance: min: 1.0654e-14 | max: 3.2339e-10 | avg: 3.2791e-11 | sum: 1.6395e-09
				Frame 2, ER variance: min: 1.0813e-15 | max:   6.71e-10 | avg: 7.6578e-11 | sum: 3.5226e-09
				Frame 3, ER variance: min:          0 | max: 6.0641e-10 | avg: 1.3849e-12 | sum:  3.015e-09
				Frame 4, ER variance: min: 2.1377e-16 | max: 3.8945e-10 | avg: 3.6952e-11 | sum: 3.7321e-09

				Frame 1, ER SNR: min:  1.45 | max:     2 | avg:  1.67 | sum:  83.4
				Frame 2, ER SNR: min:  1.48 | max:  2.03 | avg:  1.74 | sum:  79.9
				Frame 3, ER SNR: min:     0 | max:   224 | avg: 0.457 | sum:   994
				Frame 4, ER SNR: min:  1.48 | max:  1.94 | avg:  1.72 | sum:   174

			Simple ERs, only specular:
				Frame 1, ER energy: min: 0.00017386 | max:  0.0031299 | avg:  0.0012279 | sum:   0.019647
				Frame 2, ER energy: min: 0.00015046 | max:  0.0052671 | avg:  0.0019963 | sum:   0.027949
				Frame 3, ER energy: min:          0 | max:  0.0030181 | avg: 6.9558e-05 | sum:   0.018642
				Frame 4, ER energy: min: 0.00010611 | max:   0.003684 | avg:  0.0011013 | sum:    0.02643

				Frame 1, ER variance: min: 1.2034e-12 | max: 3.2339e-10 | avg: 7.6299e-11 | sum: 1.2208e-09
				Frame 2, ER variance: min: 7.1064e-13 | max:   6.71e-10 | avg:  2.078e-10 | sum: 2.9092e-09
				Frame 3, ER variance: min:          0 | max: 3.4197e-10 | avg: 5.3963e-12 | sum: 1.4462e-09
				Frame 4, ER variance: min: 4.7316e-13 | max: 3.8945e-10 | avg: 6.8618e-11 | sum: 1.6468e-09

				Frame 1, ER SNR: min:  1.45 | max:  1.97 | avg:  1.65 | sum:  26.4
				Frame 2, ER SNR: min:  1.58 | max:  2.03 | avg:  1.81 | sum:  25.4
				Frame 3, ER SNR: min:     0 | max:  19.4 | avg: 0.211 | sum:  56.4
				Frame 4, ER SNR: min:   1.5 | max:  1.92 | avg:  1.71 | sum:    41

			Simple ERs, only diffuse:
				Frame 1, ER energy: min: 1.5786e-05 | max:  0.0018141 | avg: 0.00038769 | sum:   0.013181
				Frame 2, ER energy: min: 5.0666e-06 | max:   0.002207 | avg:  0.0005104 | sum:   0.015823
				Frame 3, ER energy: min:          0 | max:  0.0051043 | avg:  9.877e-06 | sum:   0.019309
				Frame 4, ER energy: min: 2.7583e-06 | max:  0.0032377 | avg: 0.00038742 | sum:   0.027507

				Frame 1, ER variance: min: 1.0654e-14 | max: 1.3486e-10 | avg: 1.2316e-11 | sum: 4.1875e-10
				Frame 2, ER variance: min: 1.0813e-15 | max: 1.7383e-10 | avg: 1.9849e-11 | sum: 6.1532e-10
				Frame 3, ER variance: min:          0 | max: 6.0641e-10 | avg: 7.2997e-13 | sum: 1.4271e-09
				Frame 4, ER variance: min: 2.1377e-16 | max:  3.244e-10 | avg: 1.6069e-11 | sum: 1.1409e-09

				Frame 1, ER SNR: min:  1.45 | max:     2 | avg:  1.68 | sum:    57
				Frame 2, ER SNR: min:  1.48 | max:  1.99 | avg:   1.7 | sum:  52.8
				Frame 3, ER SNR: min:     0 | max:   224 | avg: 0.447 | sum:   874
				Frame 4, ER SNR: min:   1.4 | max:  1.94 | avg:  1.71 | sum:   121
		*/

		/* TODO:
			+ SNR of ERs
			+ SNR combined

			- Fixed histogram time to visualize/compare the distibution of ER energy for different methods
		*/
	}
#endif

} // namespace JPL