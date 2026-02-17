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

#include "JPLSpatial/AcousticMaterial.h"

#include "JPLSpatial/ErrorReporting.h"
#include "JPLSpatial/Math/SIMDMath.h"

namespace JPL
{
static constexpr uint32 GenerateFNVHash(std::string_view str)
{
	constexpr uint32 FNV_PRIME = 16777619u;
	constexpr uint32 OFFSET_BASIS = 2166136261u;

	const size_t length = str.length();
	const char* data = str.data();

	// TODO: vectorize this if it ever becomes a performance drag
	uint32 hash = OFFSET_BASIS;
	for (size_t i = 0; i < length; ++i)
	{
		hash ^= *data++;
		hash *= FNV_PRIME;
	}
	hash ^= '\0';
	hash *= FNV_PRIME;

	return hash;
}

const AcousticMaterial* AcousticMaterial::Get(uint32 materialID)
{
	auto material = sList.find(materialID);
	return material != sList.end() ? &material->second : nullptr;
}

const AcousticMaterial* AcousticMaterial::Get(std::string_view materialName)
{
	const uint32 ID = GenerateFNVHash(materialName);
	auto material = sList.find(ID);
	return material != sList.end() ? &material->second : nullptr;
}

const AcousticMaterial& AcousticMaterial::GetDefault()
{
	static constexpr uint32 defaultID = GenerateFNVHash("Default");
	return sList.at(defaultID);
}

#if 0
// A handful of typical absorption coeeficients for frequencies: 125, 250, 500, 1000, 2000, 4000
typename AcousticMaterial::List AcousticMaterial::sList
{
	AC_MATERIAL(Default,				0.00f,	0.00f,	0.00f,	0.00f,	0.00f,	0.00f),

	// Basic
	AC_MATERIAL(Glass6mm,				0.18f,	0.06f,	0.04f,	0.03f,	0.02f,	0.02f),
	AC_MATERIAL(GlassSmallPane,			0.04f,	0.04f,	0.03f,	0.03f,	0.02f,	0.02f),
	AC_MATERIAL(Carpet,					0.01f,	0.02f,	0.06f,	0.15f,	0.25f,	0.45f),
	AC_MATERIAL(CarpetThick,			0.02f,	0.06f,	0.14f,	0.37f,	0.60f,	0.65f),
	AC_MATERIAL(PlywoodPanel,			0.28f,	0.22f,	0.17f,	0.09f,	0.10f,	0.11f),
	AC_MATERIAL(CurtainsHeavyPleated,	0.14f,	0.35f,	0.53f,	0.75f,	0.70f,	0.60f),
	AC_MATERIAL(CurtainsLightFlat,		0.04f,	0.05f,	0.11f,	0.18f,	0.30f,	0.35f),

	// Misc
	AC_MATERIAL(CeramicTiles,			0.01f,	0.01f,	0.01f,	0.02f,	0.02f,	0.02f),
	AC_MATERIAL(Granite,				0.01f,	0.01f,	0.01f,	0.01f,	0.02f,	0.02f),
	AC_MATERIAL(BallastRocks,			0.41f,	0.53f,	0.64f,	0.84f,	0.91f,	0.63f),
	AC_MATERIAL(SuspendedCeiling,		0.45f,	0.80f,	0.65f,	0.72f,	0.78f,	0.74f),
	AC_MATERIAL(WaterOrIce,				0.008f,	0.008f, 0.013f, 0.015f,	0.02f,	0.025f),

	AC_MATERIAL(Metal_SolidSheet,			0.01f, 0.01f, 0.02f, 0.02f, 0.02f, 0.02f), // reflective proxy
	AC_MATERIAL(MetalDeck_Perf_25mmBatts,	0.19f, 0.69f, 0.99f, 0.88f, 0.52f, 0.27f),
	AC_MATERIAL(MetalDeck_Perf_75mmBatts,	0.73f, 0.99f, 0.99f, 0.89f, 0.52f, 0.31f),

	// Floors
	AC_MATERIAL(ConcreteRough,			0.01f,	0.02f,	0.04f,	0.06f,	0.08f,	0.10f),
	AC_MATERIAL(ConcretePainted,		0.01f,	0.01f,	0.02f,	0.02f,	0.02f,	0.02f),
	AC_MATERIAL(Marble,					0.01f,	0.01f,	0.01f,	0.01f,	0.02f,	0.02f),
	AC_MATERIAL(Linoleum,				0.02f,	0.03f,	0.03f,	0.03f,	0.03f,	0.02f),
	AC_MATERIAL(WoodParquet,			0.04f,	0.04f,	0.07f,	0.06f,	0.06f,	0.07f),
	AC_MATERIAL(WoodFloorHollow,		0.15f,	0.11f,	0.1f,	0.07f,	0.06f,	0.07f),

	// Walls
	AC_MATERIAL(BrickNatural,			0.03f,	0.03f,	0.03f,	0.04f,	0.05f,	0.07f),
	AC_MATERIAL(BrickPainted,			0.01f,	0.01f,	0.02f,	0.02f,	0.02f,	0.03f),
	AC_MATERIAL(ConcreteBlockRough,		0.36f,	0.44f,	0.31f,	0.29f,	0.39f,	0.25f),
	AC_MATERIAL(ConcreteBlockPainted,	0.1f,	0.05f,	0.06f,	0.07f,	0.09f,	0.08f),
	AC_MATERIAL(ConcretePouredRough,	0.01f,	0.02f,	0.04f,	0.06f,	0.08f,	0.10f),
	AC_MATERIAL(WoodenDoors,			0.1f,	0.07f,	0.05f,	0.04f,	0.04f,	0.04f),
	AC_MATERIAL(SteelDoor,				0.05f,	0.05f,	0.05f,	0.06f,	0.04f,	0.02f),
	AC_MATERIAL(PlasterMasonry,			0.01f,	0.02f,	0.02f,	0.03f,	0.04f,	0.05f)	// another good default "hard-reflective" surface
};

#undef AC_MATERIAL
#else

static std::pair<uint32, AcousticMaterial> MakeMaterial(std::string_view name, typename AcousticMaterial::AbsorptionCoeffs&& coeffs)
{
	const float sum = coeffs.reduce();
	const float avg = sum * 0.25f; JPL_ASSERT(coeffs.size() == 4);

	const simd oneMinusAlpha = simd(1.0f) - coeffs;
	
	// ReflectionLoss_dB for simd LevelDrop data member
	const simd levelDrop = -10.0f * log10(max(1e-6f, oneMinusAlpha));
	
	const uint32 hash = GenerateFNVHash(name);

	return { hash, AcousticMaterial{
		.Name = name,
		.ID = hash,
		.AbsorptionAverage_dB = ReflectionLoss_dB(avg),
		.AbsorptionAverage = avg,
		.AbsorptionAverageOneMinus = 1.0f - avg,
		.Coeffs = coeffs,
		.AmplitudeFactors = Math::Sqrt(oneMinusAlpha),
		.LevelDrop = levelDrop
	}};
}

#define AC_MATERIAL(Name_, Coef_59, Coef_369, Coef_1625, Coef_8669)\
MakeMaterial(#Name_, { Coef_59, Coef_369, Coef_1625, Coef_8669 })

typename AcousticMaterial::List AcousticMaterial::sList
{
	AC_MATERIAL(Default,					0.0f,			0.0f,			0.0f,			0.0f),

	// Basic
	AC_MATERIAL(Glass6mm,					0.179466f,		0.0556698f,		0.0243776f,		0.02f),
	AC_MATERIAL(GlassSmallPane,				0.04f,			0.0347056f,		0.0240612f,		0.02f),
	AC_MATERIAL(Carpet,						0.0100445f,		0.0489898f,		0.234212f,		0.448018f),
	AC_MATERIAL(CarpetThick,				0.0201779f,		0.121454f,		0.506233f,		0.649504f),
	AC_MATERIAL(PlywoodPanel,				0.279733f,		0.189648f,		0.099854f,		0.109901f),
	AC_MATERIAL(CurtainsHeavyPleated,		0.140934f,		0.453273f,		0.699508f,		0.600991f),
	AC_MATERIAL(CurtainsLightFlat,			0.0400445f,		0.0877093f,		0.255969f,		0.349504f),

	// Misc
	AC_MATERIAL(CeramicTiles,				0.01f,			0.0109346f,		0.0196836f,		0.02f),
	AC_MATERIAL(Granite,					0.01f,			0.01f,			0.0159388f,		0.02f),
	AC_MATERIAL(BallastRocks,				0.410534f,		0.599738f,		0.836503f,		0.632775f),
	AC_MATERIAL(SuspendedCeiling,			0.451557f,		0.706146f,		0.747884f,		0.740396f),
	AC_MATERIAL(WaterOrIce,					0.008f,			0.0108341f,		0.0185979f,		0.0249504f),
	
	AC_MATERIAL(Metal_SolidSheet	,		0.01f,			0.0152944f,		0.02f,			0.02f),		// reflective proxy
	AC_MATERIAL(MetalDeck_Perf_25mmBatts,	0.192224f,		0.808579f,		0.635093f,		0.272478f),
	AC_MATERIAL(MetalDeck_Perf_75mmBatts,	0.731157f,		0.965068f,		0.644372f,		0.312081f),

	// Floors
	AC_MATERIAL(ConcreteRough,				0.0100445f,		0.0318586f,		0.0740119f,		0.0998018f),
	AC_MATERIAL(ConcretePainted,			0.01f,			0.0152944f,		0.02f,			0.02f),
	AC_MATERIAL(Marble,						0.01f,			0.01f,			0.0159388f,		0.02f),
	AC_MATERIAL(Linoleum,					0.0200445f,		0.0294006f,		0.0286164f,		0.0200991f),
	AC_MATERIAL(WoodParquet,				0.04f,			0.0549485f,		0.0617f,		0.0699009f),
	AC_MATERIAL(WoodFloorHollow,			0.149822f,		0.104299f,		0.0663941f,		0.0699009f),

	// Walls
	AC_MATERIAL(BrickNatural,				0.03f,			0.0309346f,		0.0483895f,		0.0698018f),
	AC_MATERIAL(BrickPainted,				0.01f,			0.0152944f,		0.0213836f,		0.0299009f),
	AC_MATERIAL(ConcreteBlockRough,			0.360356f,		0.364508f,		0.330651f,		0.251388f),
	AC_MATERIAL(ConcreteBlockPainted,		0.0997776f,		0.0592262f,		0.0801776f,		0.0800991f),
	AC_MATERIAL(ConcretePouredRough,		0.0100445f,		0.0318586f,		0.0740119f,		0.0998018f),
	AC_MATERIAL(WoodenDoors,				0.0998666f,		0.0602749f,		0.0403164f,		0.04f),
	AC_MATERIAL(SteelDoor,					0.05f,			0.0509346f,		0.0450387f,		0.0201982f),
	AC_MATERIAL(PlasterMasonry,				0.0100445f,		0.0203352f,		0.037006f,		0.0499009f)	// another good default "hard-reflective" surface
};

#undef AC_MATERIAL

#endif
} // namespace JPL
