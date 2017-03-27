/*!
  @file   FaceSpecification.h
  @author David Hirvonen
  @brief  Specificatino for mean face image.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __facecrop_FaceSpecification_h__
#define __facecrop_FaceSpecification_h__

#include "drishti/core/drishti_stdlib_string.h" // android workaround
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/geometry/motion.h"

#include <opencv2/core.hpp>

#include <array>

struct FaceSpecification
{
    // Map normalized point set to specified coordinate system:
    cv::Matx33f operator()() const
    {
        const cv::Point2f c = transformation::denormalize(center, size);
        const cv::Matx33f S = transformation::scale(scale * size.width);
        const cv::Matx33f T = transformation::translate(c);
        return T * S;
    }

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & GENERIC_NVP("scale", scale);
        ar & GENERIC_NVP("center", center);
        ar & GENERIC_NVP("size", size);
    }
    
    float scale = 0.33f;
    cv::Point2f center = { 0.5f, 0.5f };
    cv::Size size = { 64, 64 };
};

#endif // __facecrop_FaceSpecification_h__
