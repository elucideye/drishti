/*! -*-c++-*-
  @file   EyeIO.cpp
  @author David Hirvonen
  @brief  Eye serialization.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Contains serialization routines for Eye models.
*/

#include "drishti/EyeIO.hpp"
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/ArrayCereal.h"

// clang-format off
#ifdef DRISHTI_CEREAL_XML_JSON
#  undef DRISHTI_CEREAL_XML_JSON
#  define DRISHTI_CEREAL_XML_JSON 1
#endif
// clang-format on

// clang-format off
#if DRISHTI_CEREAL_XML_JSON
#  include <cereal/archives/xml.hpp>
#  include <cereal/archives/json.hpp>
#endif
// clang-format on

CEREAL_CLASS_VERSION(drishti::sdk::Eye, 1);

_DRISHTI_SDK_BEGIN

// ### Vec2

template <class Archive>
void serialize(Archive& ar, drishti::sdk::Vec2i& v, const unsigned int version)
{
    ar& GENERIC_NVP("x", v[0]);
    ar& GENERIC_NVP("y", v[1]);
}

template <class Archive>
void serialize(Archive& ar, drishti::sdk::Vec2f& v, const unsigned int version)
{
    ar& GENERIC_NVP("x", v[0]);
    ar& GENERIC_NVP("y", v[1]);
}

// ### Size2

template <class Archive>
void serialize(Archive& ar, drishti::sdk::Size2i& s, const unsigned int version)
{
    ar& GENERIC_NVP("width", s.width);
    ar& GENERIC_NVP("height", s.height);
}

template <class Archive>
void serialize(Archive& ar, drishti::sdk::Size2f& s, const unsigned int version)
{
    ar& GENERIC_NVP("width", s.width);
    ar& GENERIC_NVP("height", s.height);
}

// ### Rect

template <class Archive>
void serialize(Archive& ar, drishti::sdk::Recti& r, const unsigned int version)
{
    ar& GENERIC_NVP("x", r.x);
    ar& GENERIC_NVP("y", r.y);
    ar& GENERIC_NVP("width", r.width);
    ar& GENERIC_NVP("height", r.height);
}

template <class Archive>
void serialize(Archive& ar, drishti::sdk::Rectf& r, const unsigned int version)
{
    ar& GENERIC_NVP("x", r.x);
    ar& GENERIC_NVP("y", r.y);
    ar& GENERIC_NVP("width", r.width);
    ar& GENERIC_NVP("height", r.height);
}

// ### Ellipse

template <class Archive>
void serialize(Archive& ar, drishti::sdk::Eye::Ellipse& e, const unsigned int version)
{
    ar& GENERIC_NVP("center", e.center);
    ar& GENERIC_NVP("size", e.size);
    ar& GENERIC_NVP("angle", e.angle);
}

template <class Archive>
void serialize(Archive& ar, drishti::sdk::Eye& eye, const unsigned int version)
{
    ar& GENERIC_NVP("roi", eye.getRoi());
    ar& GENERIC_NVP("eyelids", eye.getEyelids());
    ar& GENERIC_NVP("crease", eye.getCrease());
    ar& GENERIC_NVP("iris", eye.getIris());
    ar& GENERIC_NVP("pupil", eye.getPupil());
    ar& GENERIC_NVP("inner", eye.getInnerCorner());
    ar& GENERIC_NVP("outer", eye.getOuterCorner());
}

std::ostream& operator<<(std::ostream& os, const Vec2f& v)
{
    os << '{' << v[0] << ", " << v[1] << '}';
    return os;
}

std::ostream& operator<<(std::ostream& os, const Size2f& s)
{
    os << '{' << s.width << ", " << s.height << '}';
    return os;
}

std::ostream& operator<<(std::ostream& os, const Eye::Ellipse& e)
{
    os << '{' << e.center << ',' << e.size << ',' << e.angle * 180.0 / M_PI << '}';
    return os;
}

std::ostream& operator<<(std::ostream& os, const EyeOStream& eye)
{
#if DRISHTI_CEREAL_XML_JSON
    switch (eye.format)
    {
        case EyeStream::XML:
        {
            cereal::XMLOutputArchive oa(os);
            typedef decltype(oa) Archive;
            oa << GENERIC_NVP("eye", eye.eye);
            break;
        }
        case EyeStream::JSON:
        {
            cereal::JSONOutputArchive oa(os);
            typedef decltype(oa) Archive;
            oa << GENERIC_NVP("eye", eye.eye);
            break;
        }
    }
#else
    std::cerr << "Skipping JSON archive" << std::endl;
#endif

    return os;
}

std::istream& operator>>(std::istream& is, EyeIStream& eye)
{
#if DRISHTI_CEREAL_XML_JSON
    switch (eye.format)
    {
        case EyeStream::XML:
        {
            cereal::XMLInputArchive ia(is);
            typedef decltype(ia) Archive;
            ia(GENERIC_NVP("eye", eye.eye));
            break;
        }
        case EyeStream::JSON:
        {
            cereal::JSONInputArchive ia(is);
            typedef decltype(ia) Archive;
            ia(GENERIC_NVP("eye", eye.eye));
            break;
        }
    }
#else
    std::cerr << "Skipping JSON archive" << std::endl;
#endif
    return is;
}

std::ostream& operator<<(std::ostream& os, const Eye& eye)
{
    os << "pupil: {" << eye.getPupil() << "}\n";
    os << "iris: {" << eye.getIris() << "}\n";
    os << "inner: " << eye.getInnerCorner() << "\n";
    os << "outer: " << eye.getOuterCorner() << "\n";
    os << "eyelids[" << eye.getEyelids().size() << "] : \n";
    for (int i = 0; i < eye.getEyelids().size(); i += 10)
    {
        os << eye.getEyelids()[i] << "\n";
    }
    os << "crease[" << eye.getCrease().size() << "] : \n";
    for (int i = 0; i < eye.getCrease().size(); i += 10)
    {
        os << eye.getCrease()[i] << "\n";
    }

    return os;
}

std::string EyeStream::ext() const
{
    switch (format)
    {
        case EyeStream::XML:
            return "xml";
        case EyeStream::JSON:
            return "json";
    }
}

_DRISHTI_SDK_END
