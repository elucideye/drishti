/*! -*-c++-*-
  @file   drishti_cv_cereal.h
  @author David Hirvonen
  @brief  Serialization of common opencv types.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_drishti_cv_cereal_h__
#define __drishti_core_drishti_cv_cereal_h__

#include "drishti/core/drishti_core.h"

// see: https://github.com/USCiLab/cereal/issues/104

// clang-format off
#ifdef __ASSERT_MACROS_DEFINE_VERSIONS_WITHOUT_UNDERSCORES
#  undef __ASSERT_MACROS_DEFINE_VERSIONS_WITHOUT_UNDERSCORES
#  define __ASSERT_MACROS_DEFINE_VERSIONS_WITHOUT_UNDERSCORES 0
#endif
// clang-format on

#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>

// If we include these and add CEREEAL_REGISTER_TYPE(MyClass) then it has the unintended
// consequence of requiring MyClass to also export these archive types.

#define DRISHTI_CEREAL_XML_JSON 0

// clang-format off
#if DRISHTI_CEREAL_XML_JSON
#  include <cereal/archives/json.hpp>
#  include <cereal/archives/xml.hpp>
#endif
// clang-format on

#define GENERIC_NVP(name, value) ::cereal::make_nvp<Archive>(name, value)

#include <opencv2/core.hpp>

DRISHTI_BEGIN_NAMESPACE(cv)

template <class Archive>
void serialize(Archive& ar, cv::Rect& rect, const unsigned int version)
{
    ar& GENERIC_NVP("x", rect.x);
    ar& GENERIC_NVP("y", rect.y);
    ar& GENERIC_NVP("width", rect.width);
    ar& GENERIC_NVP("height", rect.height);
}

template <class Archive>
void serialize(Archive& ar, cv::Range& range, const unsigned int version)
{
    ar& GENERIC_NVP("start", range.start);
    ar& GENERIC_NVP("end", range.end);
}

template <class Archive>
void serialize(Archive& ar, cv::Size& size, const unsigned int version)
{
    ar& GENERIC_NVP("width", size.width);
    ar& GENERIC_NVP("height", size.height);
}

template <class Archive>
void serialize(Archive& ar, cv::Size2f& size, const unsigned int version)
{
    ar& GENERIC_NVP("width", size.width);
    ar& GENERIC_NVP("height", size.height);
}

template <class Archive>
void serialize(Archive& ar, cv::Point2f& p, const unsigned int version)
{
    ar& GENERIC_NVP("x", p.x);
    ar& GENERIC_NVP("y", p.y);
}

template <class Archive>
void serialize(Archive& ar, cv::Vec2f& v, const unsigned int version)
{
    ar& GENERIC_NVP("0", v[0]);
    ar& GENERIC_NVP("1", v[1]);
}

template <class Archive>
void serialize(Archive& ar, cv::Vec2i& v, const unsigned int version)
{
    ar& GENERIC_NVP("0", v[0]);
    ar& GENERIC_NVP("1", v[1]);
}

template <class Archive>
void serialize(Archive& ar, cv::RotatedRect& e, const unsigned int version)
{
    ar& GENERIC_NVP("center", e.center);
    ar& GENERIC_NVP("size", e.size);
    ar& GENERIC_NVP("angle", e.angle);
}

DRISHTI_END_NAMESPACE(cv)

#endif // __drishti_core_drishti_cv_cereal_h__
