#include <JPLSpatial/ChannelMap.h>
#include <JPLSpatial/DistanceAttenuation.h>
#include <JPLSpatial/Math/MinimalVec3.h>
#include <JPLSpatial/SpatialManager.h>

#include <array>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace
{
    struct PositionExample
    {
        std::string_view Name;
        JPL::MinimalVec3 Location;
    };

    constexpr std::array cPositionExamples{
        PositionExample{ "front", JPL::MinimalVec3{  0.0f, 0.0f, -5.0f } },
        PositionExample{ "right", JPL::MinimalVec3{  5.0f, 0.0f,  0.0f } },
        PositionExample{ "back",  JPL::MinimalVec3{  0.0f, 0.0f,  5.0f } },
        PositionExample{ "left",  JPL::MinimalVec3{ -5.0f, 0.0f,  0.0f } },
    };

    constexpr std::array cChannels{
        JPL::EChannel::FrontLeft,
        JPL::EChannel::FrontRight,
        JPL::EChannel::BackLeft,
        JPL::EChannel::BackRight
    };
}

int main()
{
    using Vec3 = JPL::MinimalVec3;
    using Spatializer = JPL::Spatial::SpatialManager<Vec3>;

    const auto outputLayout = JPL::ChannelMap::FromChannelMask(JPL::ChannelMask::Quad);

    auto distanceCurve = JPL::make_pmr_shared<JPL::AttenuationBaseModel>();
    distanceCurve->Model = JPL::AttenuationModel::Inverse;
    distanceCurve->Parameters = {
        .MinDistance = 1.0f,
        .MaxDistance = 10.0f,
        .Rolloff = 1.0f
    };
    const JPL::AttenuationCurveRef distanceCurveRef = distanceCurve;

    Spatializer spatializer;
    const JPL::Spatial::SourceId source = spatializer.CreateSource({
        .NumChannels = 1,
        .NumTargetChannels = outputLayout.GetNumChannels(),
        .PanParameters = { .Focus = 0.0f, .Spread = 0.0f },
        .DistanceAttenuationCurve = distanceCurveRef
    });

    if (not source.IsValid())
    {
        std::cerr << "Could not create the spatialized source.\n";
        return 1;
    }

    const JPL::Position<Vec3> listenerPosition{
        .Location = Vec3{ 0.0f, 0.0f, 0.0f },
        .Orientation = JPL::OrientationData<Vec3>::IdentityForward()
    };
    if (not spatializer.SetListenerPosition(spatializer.GetDefaultListener(), listenerPosition))
    {
        std::cerr << "Could not update the listener position.\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(4);

    for (const auto& example : cPositionExamples)
    {
        const JPL::Position<Vec3> sourcePosition{
            .Location = example.Location,
            .Orientation = JPL::OrientationData<Vec3>::IdentityForward()
        };

        if (not spatializer.SetSourcePosition(source, sourcePosition))
        {
            std::cerr << "Could not update the source position.\n";
            return 1;
        }
        spatializer.AdvanceSimulation();

        const auto gains = spatializer.GetChannelGains(source, outputLayout);
        if (gains.size() != cChannels.size())
        {
            std::cerr << "Could not retrieve the source channel gains.\n";
            return 1;
        }

        const float attenuation = spatializer.GetDistanceAttenuation(source, distanceCurveRef);

        std::cout << std::left << std::setw(7) << example.Name
                  << "attenuation=" << attenuation << "  ";
        for (std::size_t channelIdx = 0; channelIdx < gains.size(); ++channelIdx)
        {
            std::cout << JPL::ToStringShort(cChannels[channelIdx]) << '=' << gains[channelIdx];
            if (channelIdx + 1 != gains.size())
                std::cout << "  ";
        }
        std::cout << '\n';
    }

    spatializer.DeleteSource(source);
}
