/*! -*-c++-*-
  @file   drishti_cv.hpp
  @author David Hirvonen
  @brief  Public API header only OpenCV interop for drishti types.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_drishti_drishti_cv_hpp__
#define __drishti_drishti_drishti_cv_hpp__

// TODO: preprocessor test for opencv?

#include "drishti/Image.hpp"
#include "drishti/drishti_sdk.hpp"
#include "drishti/Eye.hpp"

#include <opencv2/core.hpp>

#include <algorithm>

_DRISHTI_SDK_BEGIN

template <typename T1, typename T2>
cv::Mat_<T2> drishtiToCv(const Image<T1>& src)
{
    return cv::Mat_<T2>(int(src.getRows()), int(src.getCols()), const_cast<T2*>(src.template ptr<T2>()), src.getStride());
}

template <typename T1, typename T2>
Image<T2> cvToDrishti(const cv::Mat_<T1>& src)
{
    return Image<T2>(src.rows, src.cols, const_cast<T2*>(src.template ptr<T2>()), src.step[0]);
}

// Rect

template <typename T>
inline cv::Rect drishtiToCv(const drishti::sdk::Rect<T>& r)
{
    return cv::Rect_<T>(r.x, r.y, r.width, r.height);
}

template <typename T>
inline drishti::sdk::Recti cvToDrishti(const cv::Rect_<T>& r)
{
    return drishti::sdk::Rect<T>(r.x, r.y, r.width, r.height);
}

// Size

template <typename T>
inline cv::Size drishtiToCv(const drishti::sdk::Size2<T>& r)
{
    return cv::Size_<T>(r.width, r.height);
}

template <typename T>
inline drishti::sdk::Size2<T> cvToDrishti(const cv::Size_<T>& r)
{
    return drishti::sdk::Size2<T>(r.width, r.height);
}

// Point

inline cv::Point2f drishtiToCv(const drishti::sdk::Vec2f& v)
{
    return cv::Point2f(v[0], v[1]);
}

inline drishti::sdk::Vec2f cvToDrishti(const cv::Point2f& p)
{
    return drishti::sdk::Vec2f(p.x, p.y);
}

inline cv::Point3f drishtiToCv(const drishti::sdk::Vec3f& v)
{
    return cv::Point3f(v[0], v[1], v[2]);
}

inline drishti::sdk::Vec3f cvToDrishti(const cv::Point3f& p)
{
    return drishti::sdk::Vec3f(p.x, p.y, p.z);
}

// vector<Point>

template <std::size_t N = 128>
inline std::vector<cv::Point2f> drishtiToCv(const Array<drishti::sdk::Vec2f, N>& v)
{
    std::vector<cv::Point2f> p(v.size());

#ifdef _MSC_VER
    // MSVC standard library pecularities require specialization of iterator_traits
    // for custom iterators that are not derived from std::iterator, since this is
    // a lightweight interface class, we simply avoid the use of MSVC standard library
    // calls here:
    // https://groups.google.com/forum/#!topic/microsoft.public.vc.stl/4BTPkpGhzDQ
    for (int i = 0; i < v.size(); i++)
    {
        p[i] = drishtiToCv(v[i]);
    }
#else
    std::transform(v.begin(), v.end(), p.begin(), [](const drishti::sdk::Vec2f& v) {
        return drishtiToCv(v);
    });
#endif
    return p;
}

template <std::size_t N = 128>
inline Array<drishti::sdk::Vec2f, N> cvToDrishti(const std::vector<cv::Point2f>& p)
{
    Array<drishti::sdk::Vec2f, N> v(p.size());
    std::transform(p.begin(), p.end(), v.begin(), [](const cv::Point2f& p) {
        return cvToDrishti(p);
    });
    return v;
}

// Ellipse

inline cv::RotatedRect drishtiToCv(const drishti::sdk::Eye::Ellipse& src)
{
    cv::RotatedRect ellipse;
    ellipse.center = { src.center[0], src.center[1] };
    ellipse.size = { src.size.width, src.size.height };
    ellipse.angle = src.angle * 180.f / static_cast<float>(M_PI);
    return ellipse;
}

inline drishti::sdk::Eye::Ellipse cvToDrishti(const cv::RotatedRect& src)
{
    drishti::sdk::Eye::Ellipse ellipse;
    ellipse.center = { src.center.x, src.center.y };
    ellipse.size = { src.size.width, src.size.height };
    ellipse.angle = src.angle * static_cast<float>(M_PI) / 180.f;
    return ellipse;
}

_DRISHTI_SDK_END

#endif // __drishti_drishti_drishti_cv_hpp__
