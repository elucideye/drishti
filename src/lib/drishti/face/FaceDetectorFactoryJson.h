/*!
  @file   FaceDetectorFactoryJson.h
  @author David Hirvonen
  @brief  Utility class declaration for loading FaceDetectorFactory from a JSON descriptor.
 
  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
*/

#ifndef __drishti_face_FaceDetectorFactoryJson_h__
#define __drishti_face_FaceDetectorFactoryJson_h__

#include <drishti/face/FaceDetectorFactory.h>

#include <string>
#include <memory>

DRISHTI_FACE_NAMESPACE_BEGIN

class FaceDetectorFactoryJson : public FaceDetectorFactory
{
public:
    FaceDetectorFactoryJson(const std::string &sModels);
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceDetectorFactory_h__
