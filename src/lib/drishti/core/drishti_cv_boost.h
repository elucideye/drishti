/*!
  @file   drishti_cv_boost.h
  @author David Hirvonen
  @brief  Serialization of common opencv types.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef DRISHTI_CV_BOOST_H
#define DRISHTI_CV_BOOST_H

#include "drishti/core/drishti_core.h"

// see: https://github.com/USCiLab/cereal/issues/104
#ifdef __ASSERT_MACROS_DEFINE_VERSIONS_WITHOUT_UNDERSCORES
#  undef __ASSERT_MACROS_DEFINE_VERSIONS_WITHOUT_UNDERSCORES
#  define __ASSERT_MACROS_DEFINE_VERSIONS_WITHOUT_UNDERSCORES 0
#endif

// Boost serialization:
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/nvp.hpp>

//#include <boost/archive/xml_oarchive.hpp>

#define GENERIC_NVP(name, value) boost::serialization::make_nvp(name, value)

#include <opencv2/core.hpp>

DRISHTI_BEGIN_NAMESPACE(cv)

template< class Archive >
void serialize(Archive & ar, cv::Rect &rect, const unsigned int version)
{
    ar & GENERIC_NVP("x", rect.x);
    ar & GENERIC_NVP("y", rect.y);
    ar & GENERIC_NVP("width", rect.width);
    ar & GENERIC_NVP("height", rect.height);
}

template< class Archive >
void serialize(Archive & ar, cv::Size &size, const unsigned int version)
{
    ar & GENERIC_NVP("width", size.width);
    ar & GENERIC_NVP("height", size.height);
}

template< class Archive >
void serialize(Archive & ar, cv::Size2f &size, const unsigned int version)
{
    ar & GENERIC_NVP("width", size.width);
    ar & GENERIC_NVP("height", size.height);
}

template<class Archive>
void serialize(Archive & ar, cv::Point2f & p, const unsigned int version)
{
    ar & GENERIC_NVP("x", p.x);
    ar & GENERIC_NVP("y", p.y);
}

template<class Archive>
void serialize(Archive & ar, cv::RotatedRect & e, const unsigned int version)
{
    ar & GENERIC_NVP ("center", e.center);
    ar & GENERIC_NVP ("size", e.size);
    ar & GENERIC_NVP ("angle", e.angle);
}

DRISHTI_END_NAMESPACE(cv)

#endif // DRISHTI_BOOST_CEREAL_H
