/*! -*-c++-*-
  @file   FaceMeshMapperEOS.h
  @author David Hirvonen (from original code by Patrik Huber)
  @brief  Declaration of a FaceMeshMapper interface to the EOS library.

  This is based on sample code provided with the EOS library.
  See: https://github.com/patrikhuber/eos

  \copyright Copyright 2014-2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceMeshMapperEOS_h__
#define __drishti_face_FaceMeshMapperEOS_h__ 1

#include "drishti/face/FaceMeshMapper.h"

#include <memory>

DRISHTI_FACE_NAMESPACE_BEGIN

class FaceMeshMapperEOS : public FaceMeshMapper
{
public:
    using FaceMeshContainerPtr = std::shared_ptr<FaceMeshContainer>;
    
    FaceMeshMapperEOS() = default;
    ~FaceMeshMapperEOS() = default;
    virtual FaceMeshContainerPtr operator()(const std::vector<cv::Point2f>& landmarks, const cv::Mat& image) = 0;
    virtual FaceMeshContainerPtr operator()(const FaceModel& face, const cv::Mat& image) = 0;
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceMeshMapper_h__
