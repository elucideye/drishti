/*! -*-c++-*-
  @file   FaceModelEstimator.h
  @author David Hirvonen
  @brief  Internal declaration of a class to estimate 3D models from 2D face landmarks.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  NOTE: This currently provide a single depth estimate based on a
  mean population interocular distance model and two eye positions.

*/

#ifndef __drishti_face_FaceModelEstimator_h__
#define __drishti_face_FaceModelEstimator_h__

#include "drishti/face/drishti_face.h"
#include "drishti/face/Face.h"
#include "drishti/sensor/Sensor.h"

#include <opencv2/core/core.hpp>

DRISHTI_FACE_NAMESPACE_BEGIN

/*

   This class will convert a 2D face model to a 3D face model.
   It may be beneficial to introduce a dimension specific template parameter
   to both the EyeModel and FaceModel classes to facilitate this.  For now
   we just estimate a single depth point based on simple interocular distance
   constants.

*/

class FaceModelEstimator
{
public:
    FaceModelEstimator(const sensor::SensorModel& sensor);

    cv::Point3f operator()(const face::FaceModel& face);

protected:
    cv::Point3f getDepth(const DRISHTI_EYE::EyeModel& eyeR, const DRISHTI_EYE::EyeModel& eyeL);

    sensor::SensorModel m_sensor;
};

DRISHTI_FACE_NAMESPACE_END

#endif /* defined(__drishti_face_FaceModelEstimator_h__) */
