/*!
  @file   drishti_cv.hpp
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Public API header only OpenCV interop for drishti types.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishtisdk__cv__
#define __drishtisdk__cv__

// TODO: preprocessor test for opencv?

#include "drishti/Image.hpp"
#include "drishti/drishti_sdk.hpp"
#include "drishti/Eye.hpp"

#include <algorithm>

_DRISHTI_SDK_BEGIN

template <typename T1, typename T2>
cv::Mat_<T2> drishtiToCv(const Image<T1> &src)
{
    return cv::Mat_<T2>(int(src.getRows()), int(src.getCols()), const_cast<T2*>(src.template ptr<T2>()), src.getStride());
}

template <typename T1, typename T2>
Image<T2> cvToDrishti(const cv::Mat_<T1> &src)
{
    return Image<T2>(src.rows, src.cols, const_cast<T2*>(src.template ptr<T2>()), src.step[0]);
}

// Rect

template <typename T>
inline cv::Rect drishtiToCv(const drishti::sdk::Rect<T> &r)
{
    return cv::Rect_<T>(r.x, r.y, r.width, r.height);
}

template <typename T>
inline drishti::sdk::Recti cvToDrishti(const cv::Rect_<T> &r)
{
    return drishti::sdk::Rect<T>(r.x, r.y, r.width, r.height);
}

// Size

template <typename T>
inline cv::Size drishtiToCv(const drishti::sdk::Size2<T> &r)
{
    return cv::Size_<T>(r.width, r.height);
}

template <typename T>
inline drishti::sdk::Size2<T> cvToDrishti(const cv::Size_<T> &r)
{
    return drishti::sdk::Size2<T>(r.width, r.height);
}

// Point

inline cv::Point2f drishtiToCv(const drishti::sdk::Vec2f &v)
{
    return cv::Point2f(v[0], v[1]);
}

inline drishti::sdk::Vec2f cvToDrishti(const cv::Point2f &p)
{
    return drishti::sdk::Vec2f(p.x, p.y);
}

// vector<Point>

inline std::vector<cv::Point2f> drishtiToCv(const std::vector<drishti::sdk::Vec2f> &v)
{
    std::vector<cv::Point2f> p(v.size());
    std::transform(v.begin(), v.end(), p.begin(), [](const drishti::sdk::Vec2f &v)
    {
        return drishtiToCv(v);
    });
    return p;
}

inline std::vector<drishti::sdk::Vec2f> cvToDrishti(const std::vector<cv::Point2f> &p)
{
    std::vector<drishti::sdk::Vec2f> v(p.size());
    std::transform(p.begin(), p.end(), v.begin(), [](const cv::Point2f &p)
    {
        return cvToDrishti(p);
    });
    return v;
}

// Ellipse

inline cv::RotatedRect drishtiToCv(const drishti::sdk::Eye::Ellipse &src)
{
    cv::RotatedRect ellipse;
    ellipse.center = { src.center[0], src.center[1] };
    ellipse.size = { src.size.width, src.size.height };
    ellipse.angle = src.angle * 180.f / float(M_PI);
    return ellipse;
}

inline drishti::sdk::Eye::Ellipse cvToDrishti(const cv::RotatedRect &src)
{
    drishti::sdk::Eye::Ellipse ellipse;
    ellipse.center = { src.center.x, src.center.y };
    ellipse.size = { src.size.width, src.size.height };
    ellipse.angle = src.angle * float(M_PI) / 180.f;
    return ellipse;
}

_DRISHTI_SDK_END

#endif // __drishtisdk__cv__
