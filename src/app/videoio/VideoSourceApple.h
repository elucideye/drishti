/*! -*-c++-*-
 @file   videoio/VideoSourceCV.h
 @author David Hirvonen (C++ Adaptation)
 @brief  Simple OpenCV cv::Mat VideoSource interface declaration.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 This is a C++ interface for the MIMovieVideoSampleAccessor class.
 See the following files:
 * MIMovieVideoSampleAccessor.{h,m}
 * MICMSampleBuffer.{h,m}
 
 */

#ifndef __videoio_VideoSourceApple_h__
#define __videoio_VideoSourceApple_h__

#include "videoio/VideoSourceCV.h"
#include "videoio/drishti_videoio.h"

#include <string>
#include <memory>

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

class VideoSourceApple : public VideoSourceCV
{
public:
    VideoSourceApple(const std::string& filename);
    ~VideoSourceApple();
    virtual Frame operator()(int i = -1);
    virtual bool good() const;
    virtual std::size_t count() const;
    virtual bool isRandomAccess() const { return false; }
    virtual void setOutputFormat(PixelFormat format);

protected:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

DRISHTI_VIDEOIO_NAMESPACE_END

#endif // __videoio_VideoSourceApple_h__
