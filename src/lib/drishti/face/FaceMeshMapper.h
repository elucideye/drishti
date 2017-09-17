/*! -*-c++-*-
  @file   FaceMeshMapper.h
  @author David Hirvonen (from original code by Patrik Huber)
  @brief  Declaration of a FaceMeshMapper interface to the EOS library.

  This is based on sample code provided with the EOS library.
  See: https://github.com/patrikhuber/eos

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceMeshMapper_h__
#define __drishti_face_FaceMeshMapper_h__ 1

#include "drishti/face/drishti_face.h"
#include "drishti/face/Face.h"
#include "drishti/core/drishti_stdlib_string.h" // first!

// Note: This class currently exposes EOS types
#include "eos/core/Landmark.hpp"   // LandmarkCollection<>
#include "eos/fitting/fitting.hpp" // RenderingParameters
#include "eos/core/Mesh.hpp"       // Mesh

#include "opencv2/core/core.hpp"

#include <iostream>
#include <memory>

DRISHTI_FACE_NAMESPACE_BEGIN

class FaceMeshMapper
{
public:
    using LandmarkCollection2d = eos::core::LandmarkCollection<cv::Vec2f>;

    struct Result
    {
        // TODO: Use generic (non-eos types) or pimpl
        eos::core::Mesh mesh;
        eos::fitting::RenderingParameters rendering_params;
        cv::Mat affine_from_ortho; // affine_from_ortho
    };

    FaceMeshMapper() {}
    virtual Result operator()(const std::vector<cv::Point2f>& landmarks, const cv::Mat& image) = 0;
    virtual Result operator()(const FaceModel& face, const cv::Mat& image) = 0;
};

// Euler: pitch, yaw, roll
cv::Point3f getRotation(const eos::fitting::RenderingParameters& rendering_parameters);
cv::Mat extractTexture(const FaceMeshMapper::Result& result, const cv::Mat& image);
void serialize(const eos::core::Mesh& mesh, const std::string& filename);
void drawWireFrameOnIso(cv::Mat& iso, const eos::core::Mesh& meshIn);
void drawWireFrame(cv::Mat& iso, const FaceMeshMapper::Result& result);
eos::core::LandmarkCollection<cv::Vec2f> convertLandmarks(const std::vector<cv::Point2f>& points);
eos::core::LandmarkCollection<cv::Vec2f> extractLandmarks(const FaceModel& face);

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceMeshMapper_h__
