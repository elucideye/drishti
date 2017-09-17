/*! -*-c++-*-
 @file   videoio/VideoSinkCV.h
 @author David Hirvonen
 @brief  Simple OpenCV cv::Mat VideoSink interface declaration.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#ifndef __videoio_VideoSinkCV_h__
#define __videoio_VideoSinkCV_h__

#include "videoio/VideoSinkCV.h"
#include "videoio/drishti_videoio.h"

#include <opencv2/core.hpp>

#include <memory>
#include <functional>

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

class VideoSinkCV
{
public:
    using CompletionHandler = std::function<void()>;

    struct Properties
    {
        int width;
        int height;
    };

    VideoSinkCV() {}
    ~VideoSinkCV() {}
    virtual bool good() = 0;
    virtual bool begin() = 0;
    virtual bool operator()(const cv::Mat& image) = 0;
    virtual bool end(const CompletionHandler& handler) = 0;
    virtual void setProperties(const Properties& properties) {}
    static std::shared_ptr<VideoSinkCV> create(const std::string& filename, const std::string& hint = {});
};

DRISHTI_VIDEOIO_NAMESPACE_END

#endif // __videoio_VideoSinkCV_h__
