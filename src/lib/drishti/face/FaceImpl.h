/*! -*-c++-*-
  @file   FaceImpl.h
  @author David Hirvonen
  @brief  Private face declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceImpl_h__
#define __drishti_face_FaceImpl_h__

#include "drishti/face/Face.h"

DRISHTI_FACE_NAMESPACE_BEGIN

// ((( IO )))
template <class Archive>
void FaceModel::serialize(Archive& ar, const unsigned int version)
{
    ar& GENERIC_NVP("eye-full-left", eyeFullL);
    ar& GENERIC_NVP("eye-full-right", eyeFullR);
    ar& GENERIC_NVP("eye-left-inner", eyeLeftInner);
    ar& GENERIC_NVP("eye-left-center", eyeLeftCenter);
    ar& GENERIC_NVP("eye-left-outer", eyeLeftOuter);
    ar& GENERIC_NVP("eyebrow-left-inner", eyebrowLeftInner);
    ar& GENERIC_NVP("eyebrow-left-inner", eyebrowLeftOuter);

    ar& GENERIC_NVP("eye-right-inner", eyeRightInner);
    ar& GENERIC_NVP("eye-right-center", eyeRightCenter);
    ar& GENERIC_NVP("eye-right-outer", eyeRightOuter);
    ar& GENERIC_NVP("eyebrow-right-inner", eyebrowRightInner);
    ar& GENERIC_NVP("eyebrow-right-inner", eyebrowRightOuter);

    ar& GENERIC_NVP("nose-tip", noseTip);
    ar& GENERIC_NVP("nose-nostril-left", noseNostrilLeft);
    ar& GENERIC_NVP("nose-nostril-right", noseNostrilRight);

    ar& GENERIC_NVP("mouth-corner-left", mouthCornerLeft);
    ar& GENERIC_NVP("mouth-corner-right", mouthCornerRight);

    ar& GENERIC_NVP("mouth", mouth);
    ar& GENERIC_NVP("mouthOuter", mouthOuter);
    ar& GENERIC_NVP("mouthInner", mouthInner);

    ar& GENERIC_NVP("nose", nose);
    ar& GENERIC_NVP("eye-left", eyeLeft);
    ar& GENERIC_NVP("eyebrow-left", eyebrowLeft);
    ar& GENERIC_NVP("eye-right", eyeRight);
    ar& GENERIC_NVP("eyebrow-right", eyebrowRight);
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceImpl_h__
