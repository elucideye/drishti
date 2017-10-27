/*! -*-c++-*-
  @file   FaceMeshMapperEOS.h
  @author David Hirvonen (from original code by Patrik Huber)
  @brief  Declaration of a FaceMeshMapper interface to the EOS library.

  PRIVATE HEADER
 
  This is based on sample code provided with the EOS library.
  See: https://github.com/patrikhuber/eos

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceMeshMapper.h"

#include "eos/core/Landmark.hpp"   // LandmarkCollection<>
#include "eos/fitting/fitting.hpp" // RenderingParameters
#include "eos/core/Mesh.hpp"       // Mesh

#ifndef __drishti_face_FaceMeshMapperEOS_h__
#define __drishti_face_FaceMeshMapperEOS_h__ 1

DRISHTI_FACE_NAMESPACE_BEGIN

class FaceMeshContainerEOS : public FaceMeshContainer
{
public:

    using Mesh = eos::core::Mesh;
    using RenderingParameters = eos::fitting::RenderingParameters;
    
    FaceMeshContainerEOS(const Mesh &mesh, const RenderingParameters &params, const cv::Mat &affine)
        : mesh(mesh)
        , rendering_params(params)
        , affine_from_ortho(affine)
    {}

    ~FaceMeshContainerEOS() = default;

    virtual void getFaceMesh(drishti::graphics::MeshTex &mesh) const;
    virtual Frustum getFrustum() const;
    virtual cv::Matx44f getModelViewProjection() const;
    virtual cv::Point3f getRotation() const; // Euler: pitch, yaw, roll
    virtual void setRotation(const cv::Point3f &R);
    virtual cv::Vec4f getQuaternion() const;
    virtual void setQuaternion(const cv::Vec4f &Q);
    virtual cv::Mat extractTexture(const cv::Mat& image);
    virtual void serialize(const std::string &filename);
    virtual void drawWireFrame(cv::Mat &iso);
    virtual void drawWireFrameOnIso(cv::Mat &iso);
    
protected:
    
    eos::core::Mesh mesh;
    eos::fitting::RenderingParameters rendering_params;
    cv::Mat affine_from_ortho; // affine_from_ortho
};

eos::core::LandmarkCollection<cv::Vec2f> convertLandmarks(const std::vector<cv::Point2f>& points);
eos::core::LandmarkCollection<cv::Vec2f> extractLandmarks(const FaceModel& face);

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceMeshMapperEOS_h__

