
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

#include "EnergyHistogramIO.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../../vendor/stb_image_write.h"

#include <vector>
#include <algorithm>
#include <numeric>

#define JPL_LOG_SCALE 1

namespace JPL
{
	// Helper for drawing line between two points in an RGBA image
	void DrawLineRGBA(unsigned char* img, int w, int h, int x0, int y0, int x1, int y1,
					  unsigned char r, unsigned char g, unsigned char b, unsigned char a)
	{
		// Bresenham's line algorithm (basic)
		const int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
		const int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
		int err = dx + dy, e2;

		while (true)
		{
			if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h)
			{
				const int idx = 4 * (y0 * w + x0);
				img[idx + 0] = r;
                img[idx + 1] = g;
                img[idx + 2] = b;
                img[idx + 3] = a;
			}
			
            if (x0 == x1 && y0 == y1)
                break;
			
            e2 = 2 * err;
			
            if (e2 >= dy)
            {
                err += dy;
                x0 += sx;
            }

			if (e2 <= dx)
            {
                err += dx;
                y0 += sy;
            }
		}
	}

    void SaveHistogramWithSNRAsPNG(
        const EnergyHistogram& hist,
        const std::string& filename,
        int width,
        int height)
    {
        static constexpr int margin = 8;
        const std::vector<float>& bins = hist.GetBins();
        const std::vector<OnlineVariance>& variances = hist.GetVariance();
        const auto numBins = static_cast<int>(bins.size());
        if (numBins <= 0 || variances.size() != bins.size())
            return;

        const float numBinsInv = 1.0f / numBins;

        std::vector<float> snrVec(variances.size());  
        std::transform(variances.begin(), variances.end(), snrVec.begin(), [](const OnlineVariance& v)
        {
            return v.GetSNR();
        });

        const int snrH = (height - margin) / 2;
        const int histH = height - margin - snrH;

        // Prepare image
        std::vector<unsigned char> img(width * height * 4, 80); // grey background

        // --- Draw Energy Histogram (Bottom) ---
        // Find max for dB normalization
        float maxVal = *std::max_element(bins.begin(), bins.end());
        if (maxVal <= 0.0f)
            maxVal = 1.0f;
        
        static constexpr const float minDB = -96.0f; // Minimum decibel for display

        auto todBFS = [](float value, float maxVal)
        {
            const float dB = std::max(minDB, (value > 0.0f) ? 20.0f * std::log10(value / maxVal) : minDB);
            return (dB - minDB) / (0.0f - minDB);
        };

        for (int i = 0; i < numBins; ++i)
        {
            const float value = bins[i];
            const float v = todBFS(value, maxVal);

            int barHeight = int(v * histH);
            const int x0 = int(i * (width * numBinsInv));
            const int x1 = int((i + 1) * (width * numBinsInv));

            bool isNan = std::isnan(value);
            if (isNan /*|| (value == 0.0f)*/)
                barHeight = histH;

            const unsigned char R = isNan * 255; // colour NaNs with red
            const unsigned char G = 0;
            const unsigned char B = /*value == 0.0f ? 255 : */0;

            for (int x = x0; x < x1; ++x)
            {
                for (int y = height - 1; y >= height - barHeight; --y)
                {
                    if (y < snrH + margin)
                        continue; // draw only in lower half

                    const int idx = 4 * (y * width + x);
                    img[idx + 0] = R;
                    img[idx + 1] = G;
                    img[idx + 2] = B;
                    img[idx + 3] = 255;
                }
            }
        }

        auto todB = [](float v)
        {
            const float dB = std::max(minDB, (v > 0.0f) ? 10.0f * std::log10(v) : -10.0f);
            return (dB - minDB) / (20.0f - (-10.0f));
        };

        // --- Draw SNR Curve (Top) ---
        // Find SNR range for normalization
        const float snrMin = todB(*std::min_element(snrVec.begin(), snrVec.end()));
        const float snrMax = todB(*std::max_element(snrVec.begin(), snrVec.end()));
        const float snrRange = (snrMax - snrMin);

        // SNR curve: orange
        for (int i = 1; i < numBins; ++i)
        {
            const float prev = todB(snrVec[i - 1]);
            const float next = todB(snrVec[i]);

            const int x0 = int((i - 1) * (width * numBinsInv));
            const int x1 = int(i * (width * numBinsInv));

            // y=0 is top of the image
            const int y0 = int((1.0f - (prev - snrMin) / snrRange) * (snrH - 1));
            const int y1 = int((1.0f - (next - snrMin) / snrRange) * (snrH - 1));

            DrawLineRGBA(img.data(), width, height,
                         x0, y0, x1, y1,
                         255, 180, 30, 255); // Orange
        }

        // Optional: draw margin line
        for (int y = snrH; y < snrH + margin; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const int idx = 4 * (y * width + x);
                img[idx + 0] = 20; img[idx + 1] = 20; img[idx + 2] = 20; img[idx + 3] = 20;
            }
        }

        // Write to file
        stbi_write_png(filename.c_str(), width, height, 4, img.data(), width * 4);
    }

	void SaveEnergyHistogramAsPNG(std::span<const float> bins, const std::string& filename, int width, int height)
	{
		const auto numBins = static_cast<int>(bins.size());
		if (numBins <= 0)
			return;

        const float numBinsInv = 1.0f / numBins;

		// Find max value for normalization
		float maxVal = *std::max_element(bins.begin(), bins.end());
		if (maxVal <= 0.0f)
			maxVal = 1.0f; // Avoid div by zero

		// Create an image buffer (RGBA, 4 bytes per pixel)
		std::vector<unsigned char> image(width * height * 4, 255); // White background

		static constexpr const float minDB = -96.0f; // Minimum decibel for display

		// Draw bars (black)
		for (int i = 0; i < numBins; ++i)
		{
#if JPL_LOG_SCALE
			const float value = bins[i];
			float dB = (value > 0.0f) ? 20.0f * std::log10(value / maxVal) : minDB;
			if (dB < minDB)
				dB = minDB;
			const float v = (dB - minDB) / (0.0f - minDB); // v in [0..1], 0 is -60dB, 1 is 0dB
#else
			const float v = bins[i] / maxVal;
#endif

			const bool isNan = std::isnan(v);

			const int barHeight = isNan ? height : int(v * (height));
			const int x0 = int(i * (width * numBinsInv));
            const int x1 = int((i + 1) * (width * numBinsInv));

			const unsigned char R = isNan * 255; // colour NaNs with red
			const unsigned char G = 0;
			const unsigned char B = 0;

			for (int x = x0; x < x1; ++x)
			{
				for (int y = height - 1; y >= height - barHeight; --y)
				{
					const int idx = 4 * (y * width + x);
					image[idx + 0] = R;
					image[idx + 1] = G;
					image[idx + 2] = B;
					image[idx + 3] = 255; // A
				}
			}
		}

		stbi_write_png(filename.c_str(), width, height, 4, image.data(), width * 4);
	}
} // namespace JPL