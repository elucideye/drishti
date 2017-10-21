/*! -*-c++-*-
  @file   ImageLogger.h
  @author David Hirvonen
  @brief  Declaration of a network image logger.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_qt_facefilter_ImageLogger_h__
#define __drishti_qt_facefilter_ImageLogger_h__

#include "drishti/core/drishti_core.h"

#include <opencv2/core.hpp>

#include <memory>

DRISHTI_CORE_NAMESPACE_BEGIN

class ImageLogger
{
public:
    ImageLogger(const std::string& host, const std::string& port);
    ~ImageLogger();
    const std::string& port() const;
    const std::string& host() const;
    void setMaxFramesPerSecond(float value);
    float getMaxFramesPerSecond() const;
    void operator()(const cv::Mat& image);

protected:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_qt_facefilter_ImageLogger_h__
