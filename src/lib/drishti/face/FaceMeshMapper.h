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
#include "drishti/face/FaceMesh.h"
#include "drishti/graphics/meshtex.h"
#include "drishti/core/drishti_stdlib_string.h" // first!

#include "opencv2/core/core.hpp"

#include <iostream>
#include <memory>

DRISHTI_FACE_NAMESPACE_BEGIN

struct Frustum
{
    Frustum() = default;
    Frustum(float l, float r, float b, float t) : l(l), r(r), b(b), t(t) {}
    float l = -1.f, r = 1.f, b = -1.f, t = 1.f;
};

class FaceMeshContainer
{
public:
    
    FaceMeshContainer() = default;
    ~FaceMeshContainer() = default;

    virtual Frustum getFrustum() const = 0;
    virtual cv::Matx44f getModelViewProjection() const = 0;
    virtual void getFaceMesh(drishti::graphics::MeshTex &mesh) const = 0;
    virtual cv::Point3f getRotation() const = 0; // Euler: pitch, yaw, roll
    virtual void setRotation(const cv::Point3f &R) {}
    virtual cv::Vec4f getQuaternion() const = 0;
    virtual void setQuaternion(const cv::Vec4f &Q) {}
    virtual cv::Mat extractTexture(const cv::Mat& image) = 0;
    virtual void serialize(const std::string &filename) = 0;
    virtual void drawWireFrame(cv::Mat &iso) = 0;    
    virtual void drawWireFrameOnIso(cv::Mat &iso) = 0;
};

class FaceMeshMapper
{
public:
    using FaceMeshContainerPtr = std::shared_ptr<FaceMeshContainer>;

    FaceMeshMapper() = default;
    ~FaceMeshMapper() = default;
    virtual FaceMeshContainerPtr operator()(const std::vector<cv::Point2f>& landmarks, const cv::Mat& image) = 0;
    virtual FaceMeshContainerPtr operator()(const FaceModel& face, const cv::Mat& image) = 0;
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceMeshMapper_h__
