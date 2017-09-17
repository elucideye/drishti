/*! -*-c++-*-
  @file   FACE.h
  @author David Hirvonen
  @brief  High level routines for parsing face data.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_landmarks_FACE_h__
#define __drishti_landmarks_FACE_h__

#include "drishti/core/Shape.h"
#include "drishti/face/Face.h"

#include <opencv2/core.hpp>

#include <boost/spirit/include/qi.hpp>
#include <boost/fusion/include/adapt_struct.hpp>

#include <fstream>

// ####################################################
// ##################### FACE #########################
// ####################################################

//namespace ph = boost::phoenix;
//namespace qi = boost::spirit::qi;
//namespace ascii = boost::spirit::ascii;

//#define CEREAL_NVP(name, value) ::cereal::make_nvp<Archive>(name, value)

namespace qi = boost::spirit::qi;

DRISHTI_BEGIN_NAMESPACE(FACE)

struct record
{
    std::string filename;
    int index;

    float angle = 0.f;                             // angle from frontal
    cv::Vec4f quaternion = { 0.f, 0.f, 0.f, 1.f }; // quaternion

    std::vector<cv::Point2f> points;
    std::vector<cv::Point2f> glasses;
    cv::Rect roi;

    cv::Mat image; // allow preloading of image data
    friend std::ostream& operator<<(const std::ostream& os, const record& r);
};

struct Table
{
    std::vector<std::string> header;
    std::vector<record> lines;

    std::function<void(std::vector<cv::Point2f>& points)> flopper; // callback for mirror

    // Populate indices for eyes and nose
    std::vector<int> eyeL;
    std::vector<int> eyeR;
    std::vector<int> nose;
    std::vector<int> mouthL;
    std::vector<int> mouthR;

    std::vector<int> brow;
    std::vector<int> mouth;
    std::vector<int> sideL;
    std::vector<int> sideR;
    std::vector<int> browL;
    std::vector<int> browR;

    std::vector<int> getInnerLandmarks(bool withMouth = false, bool withBrows = false, bool withSides = false) const;
};

inline std::ostream& operator<<(std::ostream& os, const record& r)
{
    os << r.filename << ' ';
    os << r.index << ' ';
    for (auto& i : r.points)
    {
        os << i << ' ';
    }
    return os;
}

DRISHTI_END_NAMESPACE(FACE)

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    cv::Point2f,
    (float, x)
    (float, y)
)
// clang-format on

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    FACE::Table,
    (std::vector<std::string>, header)
    (std::vector<FACE::record>, lines)
)
// clang-format on

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    FACE::record,
    (std::string, filename)
    (int, index)
    (std::vector<cv::Point2f>, points)
)
// clang-format on

FACE::Table parseDRISHTI(const std::string& filename);

std::vector<cv::Vec3b> getRainbow();

#endif // __drishti_landmarks_FACE_h__
