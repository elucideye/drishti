/*! -*-c++-*-
  @file   DRISHTI.cpp
  @author David Hirvonen
  @brief  High level routines for parsing DRISHTI data.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "landmarks/DRISHTI.h"
#include "drishti/core/Line.h"

// ====== DRISHTI =======

void parseDRISHTI(const std::string& filename, FACE::record& output)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (fs.isOpened())
    {
        {
            auto node = fs["points"];
            if (!node.empty())
            {
                drishti::core::AnnotatedContourImage data;
                data.contours.resize(1);
                node >> data.contours[0];
                if (data.contours[0].contour.size())
                {
                    output.filename = filename;
                    output.points = data.contours[0].getPoints();
                    output.roi = cv::boundingRect(output.points);
                }
            }
        }

        {
            auto node = fs["tight"];
            if (!node.empty())
            {
                drishti::core::AnnotatedContourImage data;
                data.contours.resize(1);
                node >> data.contours[0];
                if (data.contours[0].contour.size())
                {
                    output.filename = filename;
                    output.glasses = data.contours[0].getPoints();
                    output.roi = cv::boundingRect(output.glasses);
                }
            }
        }
    }
}

FACE::Table parseDRISHTI(const std::string& filename)
{
    using drishti::core::Line;

    FACE::Table table;

    // Simple 5 point model:
    table.eyeR = { 0 };
    table.eyeL = { 1 };
    table.nose = { 2 };
    table.mouth = { 3, 4 };

    std::vector<std::pair<std::string, std::string>> filenames;

    std::vector<std::string> lines;
    std::ifstream file(filename.c_str());
    if (file.is_open())
    {
        std::copy(std::istream_iterator<Line>(file), std::istream_iterator<Line>(), std::back_inserter(lines));
        for (auto& l : lines)
        {
            std::stringstream iss(l);
            std::vector<std::string> tokens;
            std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(tokens));

            //for(auto &t : tokens) std::cout << t << " "; std::cout << std::endl;

            if (tokens.size() == 2)
            {
                FACE::record record;
                parseDRISHTI(tokens[1], record);

                record.filename = tokens[0];
#if 0
                FACE::record record_;
                record_.filename = tokens[0];
                record_.roi = record.roi;
                record_.points = record.points;
                record_.glasses = record.glasses;
                table.lines.push_back(record_);
#else
                table.lines.push_back(record);
#endif
            }
        }
    }

    return table;
}
