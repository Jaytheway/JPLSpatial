#include <JPLSpatial/ChannelMap.h>
#include <JPLSpatial/Math/MinimalVec3.h>
#include <JPLSpatial/Panning/VBAPanning2D.h>

#include <array>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace
{
    struct DirectionExample
    {
        std::string_view Name;
        JPL::MinimalVec3 Direction;
    };

    constexpr std::array cDirectionExamples{
        DirectionExample{ "front", JPL::MinimalVec3{  0.0f, 0.0f, -1.0f } },
        DirectionExample{ "right", JPL::MinimalVec3{  1.0f, 0.0f,  0.0f } },
        DirectionExample{ "back",  JPL::MinimalVec3{  0.0f, 0.0f,  1.0f } },
        DirectionExample{ "left",  JPL::MinimalVec3{ -1.0f, 0.0f,  0.0f } },
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
    const auto outputLayout = JPL::ChannelMap::FromChannelMask(JPL::ChannelMask::Quad);

    JPL::StandardPanner2D panner;
    if (!panner.Initialize(outputLayout))
    {
        std::cerr << "Could not initialize the quad VBAP panner.\n";
        return 1;
    }

    std::array<float, cChannels.size()> gains{};

    std::cout << std::fixed << std::setprecision(4);

    for (const auto& example : cDirectionExamples)
    {
        panner.GetSpeakerGains(example.Direction, gains);

        std::cout << std::left << std::setw(7) << example.Name;
        for (std::size_t channelIdx = 0; channelIdx < gains.size(); ++channelIdx)
        {
            std::cout << JPL::ToStringShort(cChannels[channelIdx]) << '=' << gains[channelIdx];
            if (channelIdx + 1 != gains.size())
                std::cout << "  ";
        }
        std::cout << '\n';
    }
}
