/*! -*-c++-*-
  @file   QtFaceDetectorFactoryCereal
  @author David Hirvonen
  @brief  Implementation of cereal specific components of QtFaceDetectorFactory

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "QtFaceDetectorFactory.h"

#include "drishti/core/drishti_stdlib_string.h" // FIRST
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/face/Face.h"

#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>

#include <cereal/archives/json.hpp>

drishti::face::FaceModel QtFaceDetectorFactory::getMeanFace()
{
    drishti::face::FaceModel face;

    // clang-format off
    LoaderFunction loader = [&](std::istream& is, const std::string& hint)
    {
        if (is)
        {
            try
            {
                cereal::JSONInputArchive ia(is);
                typedef decltype(ia) Archive;

                std::vector<cv::Point2f> landmarks;
                ia(GENERIC_NVP("landmarks", landmarks));
                CV_Assert(landmarks.size() == 5);

                face.eyeRightCenter = landmarks[0];
                face.eyeLeftCenter = landmarks[1];
                face.noseTip = landmarks[2];
                face.mouthCornerRight = landmarks[3];
                face.mouthCornerLeft = landmarks[4];

                return true;
            }
            catch (std::exception& e)
            {
                return false;
            }
        }
        return false;
    };
    // clang-format on
    load(sFaceDetectorMean, loader);
    return face;
}
