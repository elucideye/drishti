/*!
  @file   Eye.cpp
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Top level API eye model implementation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of the eye model structure used
  to report results for the top level SDK.
*/

#include "drishti/Eye.hpp"
#include "core/drishti_serialize.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "drishti/drishti_cv.hpp"

#include <iostream>
#include <sstream>
#include <cmath>

CEREAL_CLASS_VERSION(drishti::sdk::Eye, 1);

_DRISHTI_SDK_BEGIN

// TODO: Better way to serialize templates non intrusively (without specialization)

// ### Vec2

template<class Archive>
void serialize(Archive & ar, drishti::sdk::Vec2i & v, const unsigned int version)
{
    ar & GENERIC_NVP("x", v[0]);
    ar & GENERIC_NVP("y", v[1]);
}

template<class Archive>
void serialize(Archive & ar, drishti::sdk::Vec2f & v, const unsigned int version)
{
    ar & GENERIC_NVP("x", v[0]);
    ar & GENERIC_NVP("y", v[1]);
}

// ### Size2

template< class Archive >
void serialize(Archive & ar, drishti::sdk::Size2i & s, const unsigned int version)
{
    ar & GENERIC_NVP("width", s.width);
    ar & GENERIC_NVP("height", s.height);
}

template< class Archive >
void serialize(Archive & ar, drishti::sdk::Size2f & s, const unsigned int version)
{
    ar & GENERIC_NVP("width", s.width);
    ar & GENERIC_NVP("height", s.height);
}

// ### Rect

template< class Archive >
void serialize(Archive & ar, drishti::sdk::Recti & r, const unsigned int version)
{
    ar & GENERIC_NVP("x", r.x);
    ar & GENERIC_NVP("y", r.y);
    ar & GENERIC_NVP("width", r.width);
    ar & GENERIC_NVP("height", r.height);
}

template< class Archive >
void serialize(Archive & ar, drishti::sdk::Rectf & r, const unsigned int version)
{
    ar & GENERIC_NVP("x", r.x);
    ar & GENERIC_NVP("y", r.y);
    ar & GENERIC_NVP("width", r.width);
    ar & GENERIC_NVP("height", r.height);
}

// ### Ellipse

template<class Archive>
void serialize(Archive & ar, drishti::sdk::Eye::Ellipse & e, const unsigned int version)
{
    ar & GENERIC_NVP ("center", e.center);
    ar & GENERIC_NVP ("size", e.size);
    ar & GENERIC_NVP ("angle", e.angle);
}

template<class Archive>
void serialize(Archive & ar, drishti::sdk::Eye & eye, const unsigned int version)
{
    ar & GENERIC_NVP ("roi",eye.getRoi());
    ar & GENERIC_NVP ("eyelids", eye.getEyelids());
    ar & GENERIC_NVP ("crease", eye.getCrease());
    ar & GENERIC_NVP ("iris", eye.getIris());
    ar & GENERIC_NVP ("pupil", eye.getPupil());
    ar & GENERIC_NVP ("inner", eye.getInnerCorner());
    ar & GENERIC_NVP ("outer", eye.getOuterCorner());
}

/*
 * Eye
 */

Eye::Eye()
{}

Eye::Eye(const Eye &src)
    : iris(src.iris)
    , pupil(src.pupil)
    , eyelids(src.eyelids)
    , crease(src.crease)
    , innerCorner(src.innerCorner)
    , outerCorner(src.outerCorner)
    , roi(src.roi)
{}

std::ostream& operator<<(std::ostream &os, const Vec2f &v)
{
    os << '{' << v[0] << ", " << v[1] << '}';
    return os;
}

std::ostream& operator<<(std::ostream &os, const Size2f &s)
{
    os << '{' << s.width << ", " << s.height << '}';
    return os;
}

std::ostream& operator<<(std::ostream &os, const Eye::Ellipse &e)
{
    os << '{' << e.center << ',' << e.size << ',' << e.angle * 180.0/M_PI << '}';
    return os;
}

std::ostream& operator<<(std::ostream &os, const EyeOStream &eye)
{
    switch(eye.format)
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
    return os;
}

std::istream& operator>>(std::istream &is, EyeIStream &eye)
{
    switch(eye.format)
    {
        case EyeStream::XML:
        {
            cereal::XMLInputArchive ia(is);
            typedef decltype(ia) Archive;
            ia( GENERIC_NVP("eye", eye.eye) );
            break;
        }
        case EyeStream::JSON:
        {
            cereal::JSONInputArchive ia(is);
            typedef decltype(ia) Archive;
            ia( GENERIC_NVP("eye", eye.eye) );
            break;
        }
    }
    return is;
}

std::ostream& operator<<(std::ostream &os, const Eye &eye)
{
    os << "pupil: {" << eye.getPupil() << "}\n";
    os << "iris: {" << eye.getIris() << "}\n";
    os << "inner: " << eye.getInnerCorner() << "\n";
    os << "outer: " << eye.getOuterCorner() << "\n";
    os << "eyelids[" << eye.getEyelids().size() << "] : \n";
    for(int i = 0; i < eye.getEyelids().size(); i+= 10)
    {
        os << eye.getEyelids()[i] << "\n";
    }
    os << "crease[" << eye.getCrease().size() << "] : \n";
    for(int i = 0; i < eye.getCrease().size(); i+= 10)
    {
        os << eye.getCrease()[i] << "\n";
    }

    return os;
}

std::string EyeStream::ext() const
{
    switch(format)
    {
        case EyeStream::XML:
            return "xml";
        case EyeStream::JSON:
            return "json";
    }
}

void createMask(Image1b &mask, const Eye &eye, int components)
{
    if(mask.getRows() && mask.getCols() && eye.getEyelids().size())
    {
        // Convert eyelids to opencv contour:
        auto eyelids = drishtiToCv(eye.getEyelids());
        std::vector<std::vector<cv::Point>> contours(1);
        std::copy(eyelids.begin(), eyelids.end(), std::back_inserter(contours[0]));

        cv::Mat maskHandle = drishtiToCv<uint8_t, uint8_t>(mask); // wrapper (shallow copy)
        if((components & kScleraRegion) && (components & kIrisRegion) && (components & kPupilRegion))
        {
            cv::fillPoly(maskHandle, contours, 255, 4);
            return;
        }

        cv::Mat1b eyeMask(mask.getRows(), mask.getCols(), uint8_t(0));
        cv::fillPoly(eyeMask, contours, 2550, 4);
        if(components & kScleraRegion)
        {
            maskHandle.setTo(255, eyeMask);
        }
        if(eye.getIris().size.width > 0.f && eye.getIris().size.height > 0.f)
        {
            cv::ellipse(maskHandle, drishtiToCv(eye.getIris()), (components & kIrisRegion) ? 255 : 0, -1, 4);
        }
        if(eye.getPupil().size.width > 0.f && eye.getPupil().size.height > 0.f)
        {
            cv::ellipse(maskHandle, drishtiToCv(eye.getPupil()), (components & kPupilRegion) ? 255 : 0, -1, 4);
        }
        maskHandle.setTo(0, ~eyeMask);
    }
}

_DRISHTI_SDK_END
