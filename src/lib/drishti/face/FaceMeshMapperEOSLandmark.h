/*! -*-c++-*-
  @file   FaceMeshMapperEOSLandmark.h
  @author David Hirvonen (from original code by Patrik Huber)
  @brief  Declaration of a FaceMeshMapper interface to the EOS library.

  This is based on sample code provided with the EOS library.
  See: https://github.com/patrikhuber/eos

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceMeshMapperLandmark_h__
#define __drishti_face_FaceMeshMapperLandmark_h__ 1

#include "drishti/face/drishti_face.h"
#include "drishti/face/Face.h"
#include "drishti/face/FaceMeshMapper.h"

#include "opencv2/core/core.hpp"

#include <iostream>
#include <memory>

DRISHTI_FACE_NAMESPACE_BEGIN

class FaceMeshMapperEOSLandmark : public FaceMeshMapper
{
public:
    using FaceMeshContainerPtr = std::shared_ptr<FaceMeshContainer>;

    FaceMeshMapperEOSLandmark(const std::string& modelfile, const std::string& mappingsfile);
    ~FaceMeshMapperEOSLandmark() = default;
    
    virtual FaceMeshContainerPtr operator()(const std::vector<cv::Point2f>& landmarks, const cv::Mat& image);
    virtual FaceMeshContainerPtr operator()(const FaceModel& face, const cv::Mat& image);

protected:
    
    struct Impl;
    std::shared_ptr<Impl> m_pImpl;
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceMeshMapperLandmark_h__
