/*!
  @file   FaceLandmarkMeshMapper.h
  @author David Hirvonen
  @brief  Declaration of a FaceMeshMapper interface to the EOS library.

  This is based on sample code provided with the EOS library.

  See: https://github.com/patrikhuber/eos

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef FACE_LANDMARK_MESH_MAPPER_H
#define FACE_LANDMARK_MESH_MAPPER_H 1

#include "face/drishti_face.h"
#include "face/Face.h"

// experimental eos stuff
#include "eos/core/Landmark.hpp"
#include "eos/core/LandmarkMapper.hpp"
#include "eos/render/Mesh.hpp"

#include "opencv2/core/core.hpp"

#include <iostream>
#include <memory>

BEGIN_FACE_NAMESPACE

class FaceLandmarkMeshMapper
{
public:

    struct Impl;

    using LandmarkCollection2d = eos::core::LandmarkCollection<cv::Vec2f>;

    FaceLandmarkMeshMapper(const std::string &modelfile, const std::string &mappingsfile);

    cv::Point3f operator()(const LandmarkCollection2d &landmarks, const cv::Mat &image, eos::render::Mesh &mesh, cv::Mat &isomap);

    cv::Point3f operator()(const FaceModel &face, const cv::Mat &image, eos::render::Mesh &mesh, cv::Mat &isomap);

    static void save(const eos::render::Mesh &mesh, const std::string &filename);

protected:

    std::shared_ptr<Impl> m_pImpl;
};

END_FACE_NAMESPACE

#endif // FACE_LANDMARK_MESH_MAPPER_H 
