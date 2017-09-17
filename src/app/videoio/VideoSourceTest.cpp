/*! -*-c++-*-
 @file   videoio/VideoSourceTest.cpp
 @author David Hirvonen
 @brief  Simple implementation of a list-of-files VideoSource.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "videoio/VideoSourceTest.h"
#include "drishti/core/drishti_stdlib_string.h"

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

VideoSourceTest::VideoSourceTest(const std::string& filename)
{
}

auto VideoSourceTest::operator()(int i) -> Frame
{
    int type = CV_8UC4;
    switch (format)
    {
        case RGB:
        case BGR:
            type = CV_8UC3;
            break;
        case RGBA:
        case BGRA:
        case ARGB:
        case ABGR:
        default:
            type = CV_8UC4;
            break;
    }

    Frame frame;
    frame.image = cv::Mat(480, 640, type, cv::Scalar(rand() % 255, rand() % 255, rand() % 255, 255));
    frame.index = counter++;
    frame.name = std::to_string(frame.index);
    return frame;
}

bool VideoSourceTest::good() const
{
    return true;
}

std::size_t VideoSourceTest::count() const
{
    return std::numeric_limits<std::size_t>::max();
}

bool VideoSourceTest::isRandomAccess() const
{
    return false;
}

void VideoSourceTest::setOutputFormat(PixelFormat value)
{
    format = value;
}

DRISHTI_VIDEOIO_NAMESPACE_END
