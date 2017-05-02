#ifndef __drishti_eye_EyeImpl_h__
#define __drishti_eye_EyeImpl_h__ 1

#include "drishti/eye/Eye.h"

DRISHTI_EYE_NAMESPACE_BEGIN

template <class Archive>
void EyeModel::serialize(Archive& ar, const unsigned int version)
{
    ar& GENERIC_NVP("roi", roi);

    ar& GENERIC_NVP("eyelids", eyelids);
    ar& GENERIC_NVP("crease", crease);

    ar& GENERIC_NVP("iris", irisEllipse);
    ar& GENERIC_NVP("pupil", pupilEllipse);

    ar& GENERIC_NVP("inner", innerCorner);
    ar& GENERIC_NVP("outer", outerCorner);

    ar& GENERIC_NVP("irisCenter", irisCenter);
    ar& GENERIC_NVP("irisInner", irisInner);
    ar& GENERIC_NVP("irisOuter", irisOuter); // was irisInner
}

DRISHTI_EYE_NAMESPACE_END

#endif // __drishti_eye_EyeImpl_h__ 1
