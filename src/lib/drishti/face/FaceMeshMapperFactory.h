/*! -*-c++-*-
  @file   FaceMeshMapperFactory.h
  @author David Hirvonen
  @brief  Internal declaration of a factory for FaceMeshMapper modules

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceMeshMapperFactory_h__
#define __drishti_face_FaceMeshMapperFactory_h__

#include "drishti/face/drishti_face.h"
#include "drishti/face/FaceMeshMapper.h"

#include <memory>
#include <string>

DRISHTI_FACE_NAMESPACE_BEGIN

class FaceMeshMapperFactory
{
public:

    enum AlignmentStrategy
    {
        kLandmarks,
        kLandmarksContours
    };
    
    FaceMeshMapperFactory(const std::string &filename);
    ~FaceMeshMapperFactory();
    std::shared_ptr<drishti::face::FaceMeshMapper> create(AlignmentStrategy kind=kLandmarks);
    
    static void serialize(const std::string &filename);
    
protected:
    
    struct Impl;
    std::unique_ptr<Impl> impl;
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceMeshMapperFactory_h__
