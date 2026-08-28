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

#include "JPLSpatial/Utilities/Variance.h"

#include <string>
#include <vector>
#include <span>
#include <cmath>

namespace JPL
{
	class EnergyHistogram
	{
	public:
		EnergyHistogram(float deltaTime, float maxTime)
			: mTimeStep(deltaTime)
			, mTimeStepInv((deltaTime > 0.0f) ? (1.0f / deltaTime) : 0.0f)
			, mNumBins(int(std::ceil(maxTime * mTimeStepInv)) + 1)
			, mBins(mNumBins, 0.0f)
			, mVariance(mNumBins)
		{
		}

		~EnergyHistogram() = default;

		// Add energy contribution at a specific time
		template<class Vec3Type>
		void AddSample(float time, float energy, const Vec3Type&)
		{
			const auto bin = static_cast<int>(time * mTimeStepInv);
			if (bin >= 0 && bin < mNumBins)
			{
				mBins[bin] += energy;

				mVariance[bin].Add(energy);
			}
		}

		// Retrieve the raw bin array for post‐processing
		const std::vector<float>& GetBins() const { return mBins; }

		// Get the time (center) of bin i
		float TimeAtBin(int i) const { return (i + 0.5f) * mTimeStep; }

		void Clear()
		{
			std::fill(mBins.begin(), mBins.end(), 0.0f);
			std::fill(mVariance.begin(), mVariance.end(), OnlineVariance{});
		}

		const std::vector<OnlineVariance>& GetVariance() const { return mVariance; }

	private:
		const float mTimeStep;
		const float mTimeStepInv;
		const int mNumBins;
		std::vector<float> mBins;
		std::vector<OnlineVariance> mVariance;
	};

	void SaveEnergyHistogramAsPNG(std::span<const float> histogram, const std::string& filename, int width = 1024, int height = 256);

	void SaveHistogramWithSNRAsPNG(
		const EnergyHistogram& hist,
		const std::string& filename,
		int width = 1024,
		int height = 512);
} // namespace JPL