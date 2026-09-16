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

#include <array>
#include <cmath>
#include <concepts>
#include <vector>
#include <span>
#include <type_traits>

//==================================================================================
/// Dual-Shelf Filter EQ, as per:
/// "Practical Realization of Dual-Shelving Filter Using Proportional Parametric Equalizers",
/// Rémi Audfray , Jean-Marc Jot, and Sam Dicker (2018)

namespace JPL
{
	namespace Impl
	{
		//==========================================================================
		/// Mainly for diagnostics and GUI,
		/// Biquad should be used in production.
		struct DualTopo
		{
			Topology::OnePole A;
			Topology::OnePole B;
			float BroadbandGain = 0.0f;

			[[nodiscard]] JPL_INLINE float CalculateResponse(float normalizedFrequency) const
			{
				return BroadbandGain * A.CalculateResponse(normalizedFrequency) * B.CalculateResponse(normalizedFrequency);
			}

			[[nodiscard]] JPL_INLINE void SetCoefficients(const DualTopo& other)
			{
				*this = other;
			}
		};

		/// Mainly for diagnostics and GUI,
		/// DirectFormI should be used in production.
		struct DualState
		{
			State::DirectFormIOnePole A;
			State::DirectFormIOnePole B;

			[[nodiscard]] JPL_INLINE float Process(float x, const DualTopo& topo)
			{
				return topo.BroadbandGain * A.Process(B.Process(x, topo.B), topo.A);
			}
		};

		//==========================================================================
		/// DSF (EQ) can be either Biquad, or DualTopo (Dual One-Pole)
		template<class T>
		concept CDSFTopo = std::same_as<T, DualTopo> or std::same_as<T, Topology::Biquad>;
		
		//==========================================================================
		template<CDSFTopo TopoType>
		class DSFCommonBase
		{
		protected:
			static constexpr bool bDualTopo = std::same_as<TopoType, DualTopo>;
			using StateType = std::conditional_t<bDualTopo, DualState, State::DirectFormI>;

		protected:
			[[nodiscard]] JPL_INLINE TopoType BuildTopo(float broadbandGain) const;

		protected:
			Cache::Shelf mLowShelfCache;
			Cache::Shelf mHighShelfCache;
		};
	} // namespace Impl

	//==========================================================================
	struct DualShelfEQParams
	{
		float Fl, Fm, Fh;	// Low, Mid, High control point frequencies
		float Gl, Gm, Gh;	// Low, Mid, High control point gains, in dB
		float SampleRate;
		float PrototypeControlFrequency = 640.0f; // Decent default for 44.1/48 kHz sample rate (choose roughly midpoint)
	};

	template<Impl::CDSFTopo TopoType>
	class DualShelfEQBase;

	/// 3-band Dual-Shelf Filter Equalizer.
	using DualShelfEQ = DualShelfEQBase<Topology::Biquad>;

	/// 3-band Dual-Shelf Filter Equalizer.
	// This version uses dual One-Pole topologies/state instead of Biquad,
	// which allows inspecting indiviDual-Shelfs, which can be
	// useful for diagnostics and GUI.
	using DualShelfEQ_DualTopo = DualShelfEQBase<Impl::DualTopo>;

	//==========================================================================
	template<Impl::CDSFTopo TopoType>
	class DualShelfEQBase : public TopoType, private Impl::DSFCommonBase<TopoType>
	{
	public:
		using Base = Impl::DSFCommonBase<TopoType>;
		using StateType = typename Base::StateType;

	public:
		[[nodiscard]] JPL_INLINE std::span<const float> GetPrototypeLUT() const { return mProtoLUT; }

		/// Prepare DSF EQ with given parameters.
		/// This must be called at least once before anything else.
		JPL_INLINE void Prepare(const DualShelfEQParams& params);

		/// Set the desired frequencies of the control points.
		inline void SetFrequencies(float Fl, float Fm, float Fh);

		/// Set the desired gains at the control points.
		inline void SetGainsDb(float Gl, float Gm, float Gh);

	private:
		/// Build shelf prototype LUT around given control frequency.
		/// This is used internally by DSF EQ to compute gain matrix for the requested gains.
		static inline void BuildProtoLUT(float controlFrequency, float sampleRate, std::vector<float>& outLUT);

		/// Returns frequency index offset in the prototype LUT.
		[[nodiscard]] static JPL_INLINE int GetLUTOffset(float frequency, float controlFrequency);

		/// Read magnitude at given prototype LUT index (can be negative or > LUT.size)
		[[nodiscard]] inline float ReadPrototype(int index) const;

	private:
		std::vector<float> mProtoLUT;
		int mControlFrequencyIdx = 0;

		// Caching terms for easier updates of frequencies or gain
		float mInvSampleRate = -1.0f; // 1 / samplerate
		float mGl = 0.0f, mGm = 0.0f, mGh = 0.0f; // gains at the three frequency control points
		std::array<float, 9> mGinv; // inverse gain conversion matrix
	};

	//==========================================================================
	struct DualShelfFilterParams
	{
		float Fl, Fh;		// Low, High control point frequencies
		float Gain;			// Desired gain at nyquist
		float SampleRate;

		// Fraction of the total dB transition assigned to the lower-frequency shelf, in [0, 1].
		// 0: transition at Fh; 1: transition at Fl; 0.5: equal split.
		float Weight;
	};

	template<Impl::CDSFTopo TopoType>
	class DualShelfFilterBase;

	/// Dual-Shelf Filter with single gain and emphasis control.
	// To interpolate parameter changes, copy initial object of this class
	// to persistent Biquad storage and use object of this class
	// as the "target". 
	using DualShelfFilter = DualShelfFilterBase<Topology::Biquad>;
	
	/// Dual-Shelf Filter with single gain and emphasis control
	// This version uses dual One-Pole topologies/state instead of Biquad,
	// which allows inspecting individual shelfs, which can be
	// useful for diagnostics and GUI.
	using DualShelfFilter_DualTopo = DualShelfFilterBase<Impl::DualTopo>;

	//==========================================================================
	template<Impl::CDSFTopo TopoType>
	class DualShelfFilterBase : public TopoType, private Impl::DSFCommonBase<TopoType>
	{
	public:
		using Base = Impl::DSFCommonBase<TopoType>;
		using StateType = typename Base::StateType;

	public:
		/// Parameter gain is expected in dB.
		/// This must be called at least once before anything else.
		JPL_INLINE void Prepare(DualShelfFilterParams params);

		/// Prepare DSF with given parameters.
		/// Parameter gain is expected to be linear.
		/// This must be called at least once before anything else.
		inline void PrepareLinear(const DualShelfFilterParams& params);

		/// Set the desired frequencies of the control points.
		JPL_INLINE void SetFrequencies(float Fl, float Fh);

		/// Set the desired filter gain at nyquist and weight between shelf filters.
		/// Weight parameters controlls how much of the gain change goes to lower vs higher shelfs.
		JPL_INLINE void SetGainLinear(float gain, float weight);

		/// Set the desired filter gain at nyquist.
		inline void SetGainLinear(float gain);

	private:
		// Caching terms for easier updates of frequencies or gain
		float mInvSampleRate = 0.0f; // 1 / samplerate
		float mWeight = 0.0f;
		float mEarlyGain = 0.0f;
	};

} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
	//==========================================================================
	template<Impl::CDSFTopo TopoType>
	JPL_INLINE TopoType Impl::DSFCommonBase<TopoType>::BuildTopo(float broadbandGain) const
	{
		if constexpr (bDualTopo)
		{
			return DualTopo{
				.A = Topology::OnePole::MakeLowShelf(mLowShelfCache),
				.B = Topology::OnePole::MakeHighShelf(mHighShelfCache),
				.BroadbandGain = broadbandGain
			};
		}
		else
		{
			return Topology::Biquad::Combine(
				Topology::OnePole::MakeLowShelf(mLowShelfCache),
				Topology::OnePole::MakeHighShelf(mHighShelfCache),
				broadbandGain
			);
		}
	}
	
	//==========================================================================
	template<Impl::CDSFTopo TopoType>
	JPL_INLINE void DualShelfEQBase<TopoType>::Prepare(const DualShelfEQParams& params)
	{
		JPL_ASSERT(params.SampleRate > 0.0f);
		JPL_ASSERT(params.PrototypeControlFrequency > 20.0f and params.PrototypeControlFrequency < (params.SampleRate * 0.5f));
		JPL_ASSERT(params.Fl < params.Fm and params.Fm < params.Fh and params.Fh < (params.SampleRate * 0.5f));

		BuildProtoLUT(params.PrototypeControlFrequency, params.SampleRate, mProtoLUT);
		mControlFrequencyIdx = GetLUTOffset(params.PrototypeControlFrequency, /* minFrequency */  20.0f);

		mGl = params.Gl;
		mGm = params.Gm;
		mGh = params.Gh;
		mInvSampleRate = 1.0f / params.SampleRate;
		SetFrequencies(params.Fl, params.Fm, params.Fh);
	}

	template<Impl::CDSFTopo TopoType>
	inline void DualShelfEQBase<TopoType>::SetFrequencies(float Fl, float Fm, float Fh)
	{
		JPL_ASSERT(mInvSampleRate > 0.0f);
		JPL_ASSERT(0.0f < Fl and Fl < Fm and Fm < Fh and Fh < (0.5f / mInvSampleRate));

		// Cache for later, when gain only changes
		Base::mLowShelfCache.t = ::tanf(JPL_PI * Fl * mInvSampleRate);
		Base::mHighShelfCache.t = ::tanf(JPL_PI * Fh * mInvSampleRate);

		const int center = mControlFrequencyIdx;
		const float Glm = ReadPrototype(center + GetLUTOffset(Fm, Fl));
		const float Glh = ReadPrototype(center + GetLUTOffset(Fh, Fl));
		const float Ghl = ReadPrototype(center - GetLUTOffset(Fl, Fh));
		const float Ghm = ReadPrototype(center - GetLUTOffset(Fm, Fh));

		// Gain covnersion matrix
#if 0
		std::array<float, 9> G
		{
			1.0f, 1.0f, Ghl,
			1.0f, Glm, Ghm,
			1.0f, Glh, 1.0f
		};
#endif

		std::array<float, 9> G1{
			Glm - Ghm * Glh,	Ghl * Glh - 1.0f,	Ghm - Ghl * Glm,
			Ghm - 1.0f,			1.0f - Ghl,			Ghl - Ghm,
			Glh - Glm,			1.0f - Glh,			Glm - 1.0f
		};
		const float detG = (Glm + Ghm - Ghm * Glh + Ghl * Glh - Ghl * Glm - 1.0f);

		JPL_ASSERT(std::isfinite(detG) and detG != 0.0f,
				   "Dual-shelf gain matrix is singular or non-finite. "
				   "Control frequencies must produce independent LUT constraints; "
				   "increase their spacing.");

		const float invDetG = 1.0f / detG;
		for (uint32 i = 0; i < G1.size(); ++i)
			mGinv[i] = G1[i] * invDetG;

		// We need to update gains when frequencies change
		SetGainsDb(mGl, mGm, mGh);
	}

	template<Impl::CDSFTopo TopoType>
	inline void DualShelfEQBase<TopoType>::SetGainsDb(float Gl, float Gm, float Gh)
	{
		JPL_ASSERT(mInvSampleRate > 0.0f);

		mGl = Gl;
		mGm = Gm;
		mGh = Gh;

		std::array<float, 3> Ks = {
			mGinv[0] * mGl + mGinv[1] * mGm + mGinv[2] * mGh,
			mGinv[3] * mGl + mGinv[4] * mGm + mGinv[5] * mGh,
			mGinv[6] * mGl + mGinv[7] * mGm + mGinv[8] * mGh
		};

		// Kl and Kh are the dB gains of low and high shelving filters
		// at their control frequencies;
		// K is an additional broadband gain.
		auto& [K, Kl, Kh] = Ks;

		// Convert to gain
		K = dBToGain(K);
		Kl = dBToGain(2.0f * Kl);
		Kh = dBToGain(2.0f * Kh);

		Base::mLowShelfCache.sqrtK = ::sqrtf(Kl);
		Base::mLowShelfCache.gain = Kl;

		Base::mHighShelfCache.sqrtK = ::sqrtf(Kh);
		Base::mHighShelfCache.gain = Kh;

		static_cast<TopoType&>(*this) = Base::BuildTopo(K);
	}

	template<Impl::CDSFTopo TopoType>
	inline void DualShelfEQBase<TopoType>::BuildProtoLUT(float controlFrequency, float sampleRate, std::vector<float>& outLUT)
	{
		static constexpr float cMinFrequency = 20.0f;
		JPL_ASSERT(controlFrequency > cMinFrequency);

		const float k = dBToGain(2.0f);
		const auto lsf = Topology::OnePole::MakeLowShelf(controlFrequency, sampleRate, k);
		FilterUtils::ComputeFilterResponse_dB(lsf, sampleRate, outLUT);
	}

	template<Impl::CDSFTopo TopoType>
	JPL_INLINE int DualShelfEQBase<TopoType>::GetLUTOffset(float frequency, float controlFrequency)
	{
		return static_cast<int>(std::lround(12.0 * std::log2(frequency / controlFrequency)));
	}

	template<Impl::CDSFTopo TopoType>
	inline float DualShelfEQBase<TopoType>::ReadPrototype(int index) const
	{
		JPL_ASSERT(not mProtoLUT.empty());

		if (index < 0)
			return 2.0f; // Prototype's low-frequency limit, in dB

		if (static_cast<std::size_t>(index) >= mProtoLUT.size())
			return 0.0f; // Prototype's high-frequency limit, in dB

		return mProtoLUT[static_cast<std::size_t>(index)];
	}

	//==========================================================================
	template<Impl::CDSFTopo TopoType>
	JPL_INLINE void DualShelfFilterBase<TopoType>::Prepare(DualShelfFilterParams params)
	{
		params.Gain = dBToGain(params.Gain);
		PrepareLinear(params);
	}

	template<Impl::CDSFTopo TopoType>
	inline void DualShelfFilterBase<TopoType>::PrepareLinear(const DualShelfFilterParams& params)
	{
		JPL_ASSERT(params.SampleRate > 0.0f);
		JPL_ASSERT(params.Gain > 0.0f);
		JPL_ASSERT(0.0f < params.Fl and params.Fl < params.Fh and params.Fh < (params.SampleRate * 0.5f));
		JPL_ASSERT(params.Weight >= 0.0f and params.Weight <= 1.0f);

		mInvSampleRate = 1.0f / params.SampleRate;
		mWeight = params.Weight;

		Base::mLowShelfCache.t = ::tanf(JPL_PI * params.Fl * mInvSampleRate);
		Base::mHighShelfCache.t = ::tanf(JPL_PI * params.Fh * mInvSampleRate);

		SetGainLinear(params.Gain);
	}

	template<Impl::CDSFTopo TopoType>
	JPL_INLINE void DualShelfFilterBase<TopoType>::SetFrequencies(float Fl, float Fh)
	{
		JPL_ASSERT(mInvSampleRate > 0.0f);
		JPL_ASSERT(0.0f < Fl and Fl < Fh and Fh < (0.5f / mInvSampleRate));

		Base::mLowShelfCache.t = ::tanf(JPL_PI * Fl * mInvSampleRate);
		Base::mHighShelfCache.t = ::tanf(JPL_PI * Fh * mInvSampleRate);

		static_cast<TopoType&>(*this) = Base::BuildTopo(mEarlyGain);
	}

	template<Impl::CDSFTopo TopoType>
	JPL_INLINE void DualShelfFilterBase<TopoType>::SetGainLinear(float gain, float weight)
	{
		JPL_ASSERT(mInvSampleRate > 0.0f);
		JPL_ASSERT(gain > 0.0f);
		JPL_ASSERT(weight >= 0.0f and weight <= 1.0f);

		mWeight = weight;
		SetGainLinear(gain);
	}

	template<Impl::CDSFTopo TopoType>
	inline void DualShelfFilterBase<TopoType>::SetGainLinear(float gain)
	{
		JPL_ASSERT(mInvSampleRate > 0.0f);
		JPL_ASSERT(gain > 0.0f);

		mEarlyGain = ::powf(gain, mWeight);

		const float lowShelfGain = 1.0f / mEarlyGain;
		const float highShelfGain = gain * lowShelfGain;

		Base::mLowShelfCache.sqrtK = ::sqrtf(lowShelfGain);
		Base::mLowShelfCache.gain = lowShelfGain;

		Base::mHighShelfCache.sqrtK = ::sqrtf(highShelfGain);
		Base::mHighShelfCache.gain = highShelfGain;

		static_cast<TopoType&>(*this) = Base::BuildTopo(mEarlyGain);
	}

} // namespace JPL
