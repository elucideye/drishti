/*! -*-c++-*-
 @file   videoio/VideoSourceCV.h
 @author David Hirvonen
 @brief  Simple OpenCV cv::Mat VideoSource interface declaration.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#ifndef __videoio_VideoSourceCV_h__
#define __videoio_VideoSourceCV_h__

#include "videoio/drishti_videoio.h"

#include <opencv2/core.hpp>

#include <memory>

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

class VideoSourceCV
{
public:
    struct Frame
    {
        Frame() {}
        Frame(const cv::Mat& image)
            : image(image)
        {
        }
        Frame(const cv::Mat& image, std::size_t index)
            : image(image)
            , index(index)
        {
        }
        Frame(const cv::Mat& image, std::size_t index, const std::string& name)
            : image(image)
            , index(index)
            , name(name)
        {
        }

        int rows() const { return image.rows; }
        int cols() const { return image.cols; }

        cv::Mat image;
        std::size_t index = 0;
        std::string name;
        std::string metadata;
    };

    enum PixelFormat
    {
        RGBA,
        BGRA,
        ARGB,
        ABGR,
        RGB,
        BGR,
        ANY,
    };

    VideoSourceCV() {}
    ~VideoSourceCV() {}

    virtual Frame operator()(int i = -1) = 0;
    virtual bool good() const { return true; }
    virtual std::size_t count() const = 0;
    virtual bool isRandomAccess() const = 0;
    virtual void setOutputFormat(PixelFormat) {}

    static std::shared_ptr<VideoSourceCV> create(const std::string& filename);

protected:
};

DRISHTI_VIDEOIO_NAMESPACE_END

#endif // __videoio_VideoSourceCV_h__
