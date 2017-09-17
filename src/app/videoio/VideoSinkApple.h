/*! -*-c++-*-
  @file   videoio/VideoSinkApple.h
  @author David Hirvonen
  @brief  AVFoundation movie writer.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This is an adaption of original source by Brad Larson from GPUImage:
  * https://github.com/BradLarson/GPUImage/blob/master/License.txt
  * https://github.com/BradLarson/GPUImage/blob/master/framework/Source/Mac/GPUImageMovieWriter.{h,m}

*/

#ifndef __videoio_VideoSinkApple_h__
#define __videoio_VideoSinkApple_h__

#include "videoio/VideoSinkCV.h"
#include "videoio/drishti_videoio.h"

#include <string>
#include <memory>

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

class VideoSinkApple : public VideoSinkCV
{
public:
    VideoSinkApple(const std::string& filename, const std::string& hint = {});
    ~VideoSinkApple();

    virtual bool good();
    virtual bool begin();
    virtual bool operator()(const cv::Mat& image);
    virtual bool end(const CompletionHandler& handler);
    virtual void setProperties(const Properties& properties);

protected:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

DRISHTI_VIDEOIO_NAMESPACE_END

#endif // __videoio_VideoSinkApple_h__
