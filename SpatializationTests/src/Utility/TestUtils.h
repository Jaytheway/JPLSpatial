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

#include <ranges>
#include <algorithm>
#include <vector>
#include <span>
#include <format>
#include <string>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace JPL
{
	struct Counters
	{
		std::vector<int> Count;
		std::vector<float> CountNormalized;
	};

	// TODO: maybe abstract away the variances and just use a predicate extraction
	static void ExtractCountersFromVariances(std::span<const OnlineVariance> sampleBinVariances, Counters& counters)
	{
		counters.Count.resize(sampleBinVariances.size());
		std::ranges::transform(sampleBinVariances, counters.Count.begin(), [](const OnlineVariance& v)
		{
			return v.Count;
		});

		const int max = *std::ranges::max_element(counters.Count);
		const float maxInv = max == 0 ? 0.0f : 1.0f / max;

		counters.CountNormalized.resize(counters.Count.size());
		std::ranges::transform(counters.Count, counters.CountNormalized.begin(), [maxInv](int count)
		{
			return count * maxInv;
		});
	}

	static auto GetAverageOf(const auto& dataVector)
	{
		if (dataVector.empty())
			return 0.0f;

		using ElementType = std::remove_cvref_t<decltype(dataVector[0])>;

		const float countInv = 1.0f / dataVector.size();
		return countInv * std::accumulate(dataVector.begin(), dataVector.end(), ElementType(0));
	}

	static float CalculateAverageSNR(std::span<const OnlineVariance> variances)
	{
		if (variances.empty())
			return 0.0f;

		const float numBinsInv = 1.0f / variances.size();
		return numBinsInv * std::accumulate(variances.begin(), variances.end(), 0.0f, [](float acc, const OnlineVariance& v)
		{
			return acc + v.GetSNR();
		});
	}

	struct MinMaxAvgSum
	{
		float Min{ -1.0f };
		float Max{ -1.0f };
		float Avg{ -1.0f };
		float Sum{ -1.0f };

		template<class GetValuePredicate = std::identity>
		static MinMaxAvgSum Of(const auto& dataContainer, GetValuePredicate getValue = {})
		{
			const size_t count = dataContainer.size();
			if (count == 0)
				return MinMaxAvgSum{ 0.0f, 0.0f, 0.0f, 0.0f };

			MinMaxAvgSum data{ FLT_MAX, -FLT_MAX, 0.0f, 0.0f };
			float sum = 0.0f;
			for (const auto& it : dataContainer)
			{
				const float value = getValue(it);
				sum += value;
				data.Min = std::min(data.Min, value);
				data.Max = std::max(data.Max, value);
			}

			data.Avg = sum / count;
			data.Sum = sum;
			return data;
		}

		inline std::string ToString() const
		{
			return std::format("min: {:10.5} | max: {:10.5} | avg: {:10.5} | sum: {:10.5}", Min, Max, Avg, Sum);
		}

		inline std::string ToStringLowPrecision() const
		{
			return std::format("min: {:5.3} | max: {:5.3} | avg: {:5.3} | sum: {:5.3}", Min, Max, Avg, Sum);
		}
	};


	template<class PrinterType>
	class Printer : public PrinterType
	{
	public:
		Printer() = default;

		// TODO: maybe make it stateful RAII

		template<class GetDataPredicate = std::identity>
		static void PrintData(const auto& dataContainer, std::string_view formatString, GetDataPredicate getData = {}, std::string_view iterationStr = "Frame ")
		{
			//std::cout << std::endl;
			uint32_t i = 0;
			for (auto it : dataContainer)
			{
				const auto data = getData(it);
				PrinterType::PrintLine(std::format("{}{}, {}", iterationStr, i + 1, std::vformat(formatString, std::make_format_args(data))));
				++i;
			}

			PrinterType::EndPrint();
		}
	};

	class PrintToCout
	{
	public:
		static void PrintLine(std::string_view line)
		{
			std::cout << line << std::endl;
		}

		static void EndPrint() {}
	};
	using CoutPrinter = Printer<PrintToCout>;

	// TODO: when needed
	//class PrintToFile
	
	
	
	template<class Vec3, class Vec3i>
	void SaveOBJ(const std::vector<Vec3>& vertices,
				 const std::vector<Vec3i>& faces,
				 const std::string& path)
	{
		std::ofstream out(path);
		if (!out.is_open())
		{
			std::cerr << "Failed to open vectors.csv for writing\n";
			return;
		}

		for (const auto& v : vertices)
			out << "v " << v.X << ' ' << v.Y << ' ' << v.Z << '\n';

		for (const auto& face : faces)
			out << "f " << face[0] + 1 << ' ' // OBJ is 1-based
			<< face[1] + 1 << ' '
			<< face[2] + 1 << '\n';

		out.close();
	}

	template<class Vec3>
	struct GroupedPoint { Vec3 Point; int Group; };
	template<class Vec3>
	struct IntencityPoint { Vec3 Direction; float Intensity; };


	template<class Container, class ToRowString>
	void SaveCSV(const Container& data, ToRowString wirteStringRow, const std::string& path, std::string_view header = {})
	{
		std::ofstream file(path);
		if (!file.is_open())
		{
			std::cerr << "Failed to open " << path << " for writing\n";
			return;
		}

		// Write header (optional)
		if(!header.empty())
			file << header << "\n";

		for (const auto& d : data)
		{
			wirteStringRow(std::ref(file), d);
			file << "\n";
		}

		file.close();
		std::cout << "Exported " << data.size() << " data points to " << path << "\n";
	}

	template<class Vec3>
	void SaveCSV(const std::vector<GroupedPoint<Vec3>>& points, const std::string& path)
	{
		SaveCSV(points, [](std::ofstream& file, const GroupedPoint<Vec3>& p)
		{
			file << p.Point.X << "," << p.Point.Y << "," << p.Point.Z << "," << p.Group;
		}, path, "x,y,z,group");
	}

	template<class Vec3>
	void SaveToJSON(const std::vector<GroupedPoint<Vec3>>& groupPoints,
					  const std::vector<IntencityPoint<Vec3>>& channelVis,
					  const std::string& filename)
	{
		std::ofstream out(filename);
		out << std::fixed << std::setprecision(6);
		out << "{\n";
		// Write vbap_points
		out << "  \"vbap_points\": [\n";
		for (size_t i = 0; i < groupPoints.size(); ++i)
		{
			const auto& p = groupPoints[i];
			out << "    {\"x\":" << p.Point.X << ",\"y\":" << p.Point.Y << ",\"z\":" << p.Point.Z
				<< ",\"group\":" << p.Group << "}";
			if (i + 1 != groupPoints.size()) out << ",";
			out << "\n";
		}
		out << "  ],\n";
		// Write speakers
		out << "  \"speakers\": [\n";
		for (size_t i = 0; i < channelVis.size(); ++i)
		{
			const auto& s = channelVis[i];
			out << "    {\"x\":" << s.Direction.X << ",\"y\":" << s.Direction.Y << ",\"z\":" << s.Direction.Z
				<< ",\"intensity\":" << s.Intensity << "}";
			if (i + 1 != channelVis.size()) out << ",";
			out << "\n";
		}
		out << "  ]\n";
		out << "}\n";
		out.close();
	}

} // namespace JPL