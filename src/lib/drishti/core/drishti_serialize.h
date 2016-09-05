/*!
  @file   drishti_serialize.h
  @author David Hirvonen
  @brief  Declaration of boost/cereal (de)serialization for common classes. This file also contains
  implementations of standard library routines that are required by CEREAL and that are missing
  on some platforms (notably ANDROID).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef drishtisdk_drishti_serialize_h
#define drishtisdk_drishti_serialize_h

#include "drishti/core/drishti_core.h"

#define USE_CEREAL 1

#if USE_CEREAL
// see: https://github.com/USCiLab/cereal/issues/104
#ifdef __ASSERT_MACROS_DEFINE_VERSIONS_WITHOUT_UNDERSCORES
#  undef __ASSERT_MACROS_DEFINE_VERSIONS_WITHOUT_UNDERSCORES
#  define __ASSERT_MACROS_DEFINE_VERSIONS_WITHOUT_UNDERSCORES 0
#endif

#include <opencv2/core/core.hpp>

#if ANDROID
DRISHTI_BEGIN_NAMESPACE(std)

template <typename T> inline std::string to_string(T value)
{
    std::ostringstream os ;
    os << value ;
    return os.str() ;
}

template <typename T> inline T stringTo(const std::string &s)
{
    std::stringstream conv;
    conv << s;
    T t;
    conv >> t;
    return t;
}

inline long long stoll(const std::string &s)
{
    return stringTo<long long>(s);
}
inline int stoi(const std::string &s)
{
    return stringTo<int>(s);
}
inline unsigned long stoul(const std::string &s)
{
    return stringTo<unsigned long>(s);
}
inline unsigned long long stoull(const std::string &s)
{
    return stringTo<unsigned long long>(s);
}
inline float stof(const std::string &s)
{
    return stringTo<float>(s);
}
inline long stol(const std::string &s)
{
    return stringTo<long>(s);
}
inline double stod(const std::string &s)
{
    return stringTo<double>(s);
}
inline long double stold(const std::string &s)
{
    return stringTo<long double>(s);
}

DRISHTI_END_NAMESPACE(std)
#endif

# include <cereal/types/vector.hpp>
# include <cereal/types/polymorphic.hpp>
# include <cereal/archives/json.hpp>
# include <cereal/archives/xml.hpp>

# define GENERIC_NVP(name, value) ::cereal::make_nvp<Archive>(name, value)
#else
// Boost serialization:
# include <boost/serialization/string.hpp>
# include <boost/serialization/version.hpp>
# include <boost/serialization/vector.hpp>
# include <boost/archive/xml_oarchive.hpp>
# define GENERIC_NVP(name, value) boost::serialization::make_nvp(name, value)
#endif

/// Serialization:

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

#endif
