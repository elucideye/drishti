/*! -*-c++-*-
  @file   FaceMeshMapperEOSContainerImpl.h
  @author David Hirvonen (from original code by Patrik Huber)
  @brief  Declaration of a FaceMeshMapper interface to the EOS library.

  This is based on sample code provided with the EOS library.
  See: https://github.com/patrikhuber/eos

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// Note: This class currently exposes EOS types
#include "eos/core/Landmark.hpp"   // LandmarkCollection<>
#include "eos/core/Mesh.hpp"       // Mesh
#include "eos/fitting/fitting.hpp" // RenderingParameters
#include "eos/render/texture_extraction.hpp"
#include "eos/render/utils.hpp"

#ifndef __drishti_face_FaceMeshMapperImpl_h__
#define __drishti_face_FaceMeshMapperImpl_h__ 1

struct FaceMeshMapperEOSCo::Result::Impl
{
    Impl() = default;
    ~Impl() = default;

    eos::core::Mesh mesh;
    eos::fitting::RenderingParameters rendering_params;
    cv::Mat affine_from_ortho; // affine_from_ortho    
};

#endif // __drishti_face_FaceMeshMapperImpl_h__
