//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPL Spatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright Jaroslav Pevno 2026, JPL Spatial is offered under the terms of the ISC license:
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

#include "JPLSpatial/Math/DecibelsAndGain.h"
#include "JPLSpatial/Auralization/FilterComponents.h"
#include "JPLSpatial/FrequencyBands.h"

#include <array>
#include <cmath>
#include <complex>
#include <vector>
#include <span>

//==============================================================================
/// Multi-Shelf graphic equalizer
/// based on Schlecht et al. (2025) and Audfray et al. (2018)

namespace JPL
{
	//==========================================================================
	template<std::size_t N> requires(N > 1)
	struct ShelfEQParams
	{
		std::span<const float, N - 1> Splits;	// Band split frequencies
		std::span<const float, N> G;			// Target gains at band centres
		float SampleRate;
	};

	//==========================================================================
	// Forward declaring
	template<std::size_t NumEQBands> requires(NumEQBands > 1)
	class ShelfEQ;

	/// 4-band 2nd-order graphic shelf equalizer
	using ShelfEQ4 = ShelfEQ<4>;

	//==========================================================================
	/// Multi-Shelf 2nd-order graphic equalizer
	template<std::size_t NumEQBands> requires(NumEQBands > 1)
	class ShelfEQ : public Topology::Cascade<NumEQBands - 1>
	{
	public:
		static constexpr std::size_t NumBands = NumEQBands;
		static constexpr std::size_t NumFilters = NumBands - 1;

		using Base = Topology::Cascade<NumEQBands - 1>;
		using Params = ShelfEQParams<NumBands>;
		using StateType = State::Cascade<NumFilters>;

		using GainVec = std::array<float, NumBands>;
		using GainMat = std::array<float, NumBands * NumBands>;

	public:
		/// Prepare DSF EQ with given parameters.
		/// This must be called at least once before anything else.
		JPL_INLINE bool Prepare(const Params& params);

		/// Set the desired frequencies of the control points.
		bool SetFrequencies(std::span<const float, NumFilters> frequencies);

		/// Set the desired gains at the control points.
		void SetGainsDb(std::span<const float, NumBands> gainsDb);

	private:
		[[nodiscard]] static bool Inverse(const GainMat& A, GainMat& outInvA);

#ifdef JPL_ENABLE_ASSERTS
		void ValidateFrequencies(std::span<const float, NumFilters> frequencies) const;
		void ValidateBandCentres(std::span<const float, NumBands> centres) const;
#endif

	private:
		// Caching terms for easier updates of frequencies or gain
		float mInvSampleRate = -1.0f;	// 1 / samplerate
		GainVec mG;						// target gains at the band centres
		GainMat mGinv;					// inverse gain conversion matrix
		std::array<Cache::BiquadShelf, NumFilters> mCache;
	};
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
#ifdef JPL_ENABLE_ASSERTS
	template<std::size_t NumEQBands> requires(NumEQBands > 1)
	inline void ShelfEQ<NumEQBands>::ValidateFrequencies(std::span<const float, NumFilters> frequencies) const
	{
		JPL_ASSERT(
			std::isfinite(mInvSampleRate) and
			mInvSampleRate > 0.0f,
			"Prepare must supply a valid sample rate.");

		static constexpr double minBandHz = 20.0;
		static constexpr double maxBandHz = 22050.0;
		//! Note: currenlty JPL Spatial uses this as the upper bound across all propagation systesm.
		//! But this might change, and we might use filter's nyquist.

		double previousHz = minBandHz;

		for (float frequency : frequencies)
		{
			JPL_ASSERT(
				std::isfinite(frequency) and
				static_cast<double>(frequency) > previousHz and
				static_cast<double>(frequency) < maxBandHz,
				"Splits must be finite, strictly increasing, "
				"and inside the propagation frequency range.");

			const double normalized =
				static_cast<double>(frequency) * static_cast<double>(mInvSampleRate);

			JPL_ASSERT(
				normalized > 0.0 and normalized < 0.5,
				"Shelf frequencies must be below Nyquist.");

			previousHz = frequency;
		}
	}

	template<std::size_t NumEQBands> requires(NumEQBands > 1)
	inline void ShelfEQ<NumEQBands>::ValidateBandCentres(std::span<const float, NumBands> centres) const
	{
		float previousCentre = 0.0f;
		for (float centre : centres)
		{
			JPL_ASSERT(
				std::isfinite(centre) and
				centre > previousCentre and
				centre < 0.5f,
				"Target centres must be finite, strictly increasing, "
				"and below Nyquist.");

			previousCentre = centre;
		}
	}
#endif
	
	//==========================================================================
	template<std::size_t NumEQBands> requires(NumEQBands > 1)
	JPL_INLINE bool ShelfEQ<NumEQBands>::Prepare(const Params& params)
	{
		JPL_ASSERT(
			std::isfinite(params.SampleRate) and
			params.SampleRate > 0.0f,
			"Sample rate must be finite and positive.");

		std::ranges::copy(params.G, mG.begin());
		mInvSampleRate = 1.0f / params.SampleRate;
		return SetFrequencies(params.Splits);
	}

	template<std::size_t NumEQBands> requires(NumEQBands > 1)
	inline bool ShelfEQ<NumEQBands>::SetFrequencies(std::span<const float, NumFilters> frequencies)
	{
#ifdef JPL_ENABLE_ASSERTS
		ValidateFrequencies(frequencies);
#endif
		
		std::array<Cache::BiquadShelf, NumFilters> newCache;
		std::array<Topology::Biquad, NumFilters> sh;

		// Make prototype shelfs
		for (uint32 i = 0; i < frequencies.size(); ++i)
		{
			newCache[i] = Cache::BiquadShelf::From(frequencies[i] * mInvSampleRate);
			sh[i] = Topology::Biquad::MakeHighShelf(newCache[i], /* gain_dB */1.0f);
		}

		std::array<float, NumBands> centres = ComputeBandCenters(frequencies); // using default nyquist 22050 Hz for band centres
		for (float& c : centres)
			c *= mInvSampleRate;

#ifdef JPL_ENABLE_ASSERTS
		ValidateBandCentres(centres);
#endif
		// Compute gain conversion matrix
		GainMat A;
		for (uint32 row = 0; row < NumBands; ++row)
		{ 
			A[row * NumBands] = 1.0f;
			for (uint32 col = 1; col < NumBands; ++col)
				A[row * NumBands + col] = static_cast<float>(-GainTodB(std::abs(sh[col - 1].CalculateResponse(centres[row]))));
		}

		// Store inverse gain conversion matrix
		const bool bInverted = Inverse(A, mGinv);

		if (not JPL_ENSURE(bInverted, "Gain conversion matrix doesn't have inverse."))
			return false;

		// Cache for later, when gain only changes
		mCache = newCache;
	
		// We need to update gains when frequencies change
		SetGainsDb(mG);

		return true;
	}

	template<std::size_t NumEQBands> requires(NumEQBands > 1)
	inline bool ShelfEQ<NumEQBands>::Inverse(const GainMat& A, GainMat& outInvA)
	{
		static constexpr uint32 N = NumBands;
		static constexpr uint32 Width = N * 2;

		std::array<double, N * Width> augmented{};

		auto At = [&augmented](uint32 row, uint32 col) -> double&
		{
			return augmented[row * Width + col];
		};

		double scale = 0.0;

		for (uint32 row = 0; row < N; ++row)
		{
			for (uint32 col = 0; col < N; ++col)
			{
				const double value = A[row * N + col];
				if (not std::isfinite(value))
					return false;

				At(row, col) = value;
				scale = std::max(scale, std::abs(value));
			}

			At(row, N + row) = 1.0;
		}

		if (scale == 0.0)
			return false;

		const double pivotTolerance =
			64.0 * static_cast<double>(N) * std::numeric_limits<double>::epsilon() * scale;

		for (uint32 col = 0; col < N; ++col)
		{
			uint32 pivotRow = col;

			double pivotMagnitude = std::abs(At(col, col));

			for (uint32 row = col + 1; row < N; ++row)
			{
				const double magnitude = std::abs(At(row, col));

				if (magnitude > pivotMagnitude)
				{
					pivotMagnitude = magnitude;
					pivotRow = row;
				}
			}

			if (not std::isfinite(pivotMagnitude) or pivotMagnitude <= pivotTolerance)
				return false;


			if (pivotRow != col)
			{
				for (uint32 j = 0; j < Width; ++j)
					std::swap(At(col, j), At(pivotRow, j));
			}

			const double inversePivot = 1.0 / At(col, col);

			for (uint32 j = 0; j < Width; ++j)
				At(col, j) *= inversePivot;

			At(col, col) = 1.0;

			for (uint32 row = 0; row < N; ++row)
			{
				if (row == col)
					continue;

				const double factor = At(row, col);

				for (uint32 j = 0; j < Width; ++j)
					At(row, j) -= factor * At(col, j);

				At(row, col) = 0.0;
			}
		}

		GainMat candidate{};

		for (uint32 row = 0; row < N; ++row)
		{
			for (uint32 col = 0; col < N; ++col)
			{
				const double value = At(row, N + col);

				if (not std::isfinite(value) or std::abs(value) > std::numeric_limits<float>::max())
					return false;

				candidate[row * N + col] = static_cast<float>(value);
			}
		}

		outInvA = candidate;
		return true;
	}

	template<std::size_t NumEQBands> requires(NumEQBands > 1)
	inline void ShelfEQ<NumEQBands>::SetGainsDb(std::span<const float, NumBands> gainsDb)
	{
		JPL_ASSERT(mInvSampleRate > 0.0f);
		
		std::ranges::copy(gainsDb, mG.begin());

		GainVec Ks{};

		for (uint32 col = 0, invIdx = 0; col < NumBands; ++col)
		{
			for (uint32 row = 0; row < NumBands; ++row)
				Ks[col] += mGinv[invIdx++] * mG[row];
		}

		const float broadbandGain = dBToGain(Ks[0]);

		for (uint32 i = 0; i < Base::Stages.size(); ++i)
			Base::Stages[i] = Topology::Biquad::MakeHighShelf(mCache[i], Ks[i + 1]);
		
		// Apply broadband gain
		auto& firstStage = Base::Stages[0];
		firstStage.b0 *= broadbandGain;
		firstStage.b1 *= broadbandGain;
		firstStage.b2 *= broadbandGain;

	}
} // namespace JPL
