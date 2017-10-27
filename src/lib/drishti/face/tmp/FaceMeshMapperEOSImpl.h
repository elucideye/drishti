/*! -*-c++-*-
  @file   FaceMeshMapperEOSImpl.h
  @author David Hirvonen (from original code by Patrik Huber)
  @brief  Private implementation file for EOS 3D mesh fitting.

  PRIVATE implementation file.

  This is based on sample code provided with the EOS library.
  See: https://github.com/patrikhuber/eos

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "eos/core/Landmark.hpp"   // LandmarkCollection<>
#include "eos/core/Mesh.hpp"       // Mesh
#include "eos/fitting/fitting.hpp" // RenderingParameters
#include "eos/render/texture_extraction.hpp"
#include "eos/render/utils.hpp"

#include <opencv2/core.hpp>

#ifndef __drishti_face_FaceMeshMapperEOSImpl_h__
#define __drishti_face_FaceMeshMapperEOSImpl_h__ 1

struct FaceMeshMapper::Result::Impl
{
    Impl() = default;
    ~Impl() = default;

    eos::core::Mesh mesh;
    eos::fitting::RenderingParameters rendering_params;
    cv::Mat affine_from_ortho; // affine_from_ortho    
};

#endif // __drishti_face_FaceMeshMapperEOSImpl_h__
