/*! -*-c++-*-
 @file   LFPW.cpp
 @author David Hirvonen
 @brief  High level routines for parsing LFPW data.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "landmarks/BIOID.h"

FACE::Table parseLFPW(const std::string& filename)
{
    FACE::Table table = parseBIOID(filename);

    table.browR = { 17, 18, 19, 20, 21 };
    table.browL = { 22, 23, 24, 25, 26 };
    table.eyeR = { 36, 37, 38, 39, 40, 41 };
    table.eyeL = { 42, 43, 44, 45, 46, 47 };
    table.nose = { 27, 28, 29, 30, 31, 32, 33, 34, 35 };
    table.mouth = { 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59 };
    table.mouthR = { 48 };
    table.mouthL = { 54 };
    table.brow = {};

    table.flopper = [](std::vector<cv::Point2f>& points) {
        assert(points.size() == 68);
        // http://ibug.doc.ic.ac.uk/resources/facial-point-annotations/

        // Contour
        std::swap(points[0], points[16]);
        std::swap(points[1], points[15]);
        std::swap(points[2], points[14]);
        std::swap(points[3], points[13]);
        std::swap(points[4], points[12]);
        std::swap(points[5], points[11]);
        std::swap(points[6], points[10]);
        std::swap(points[7], points[9]);
        std::swap(points[8], points[8]);

        // Eyebrow
        std::swap(points[17], points[26]);
        std::swap(points[18], points[25]);
        std::swap(points[19], points[24]);
        std::swap(points[20], points[23]);
        std::swap(points[21], points[22]);

        // Nose
        std::swap(points[27], points[27]);
        std::swap(points[28], points[28]);
        std::swap(points[29], points[29]);
        std::swap(points[30], points[30]);

        std::swap(points[31], points[35]);
        std::swap(points[32], points[34]);
        std::swap(points[33], points[33]);

        // Eye
        std::swap(points[39], points[42]);
        std::swap(points[38], points[43]);
        std::swap(points[37], points[44]);
        std::swap(points[36], points[45]);
        std::swap(points[40], points[47]);
        std::swap(points[41], points[46]);

        // Mouth
        std::swap(points[48], points[54]);
        std::swap(points[49], points[53]);
        std::swap(points[50], points[52]);
        std::swap(points[51], points[51]);

        std::swap(points[59], points[55]);
        std::swap(points[58], points[56]);
        std::swap(points[57], points[57]);

        std::swap(points[60], points[64]);
        std::swap(points[61], points[63]);
        std::swap(points[62], points[62]);

        std::swap(points[67], points[65]);
        std::swap(points[66], points[66]);
    };

    return table;
}
