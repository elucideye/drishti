/*! -*-c++-*-
 @file   videoio/VideoSourceTest.h
 @author David Hirvonen
 @brief  Simple declaration of a list-of-files VideoSource.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#ifndef __videoio_VideoSourceTest_h__
#define __videoio_VideoSourceTest_h__

#include "videoio/VideoSourceCV.h"
#include "videoio/drishti_videoio.h"

#include <string>
#include <memory>

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

class VideoSourceTest : public VideoSourceCV
{
public:
    VideoSourceTest(const std::string& filename);
    Frame operator()(int i = -1) override;
    bool good() const override;
    std::size_t count() const override;
    bool isRandomAccess() const override;
    void setOutputFormat(PixelFormat value) override;

protected:
    std::size_t counter = 0;
    PixelFormat format = RGBA;
};

DRISHTI_VIDEOIO_NAMESPACE_END

#endif // __videoio_VideoSourceTest_h__
