/*! -*-c++-*-
 @file   videoio/VideoSourceStills.h
 @author David Hirvonen
 @brief  Simple declaration of a list-of-files VideoSource.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#ifndef __videoio_VideoSourceStills_h__
#define __videoio_VideoSourceStills_h__

#include "videoio/VideoSourceCV.h"
#include "videoio/drishti_videoio.h"

#include <string>
#include <memory>

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

// Random access VideoSourceCV
class VideoSourceStills : public VideoSourceCV
{
public:
    class Impl;

    explicit VideoSourceStills(const std::string& filename);
    explicit VideoSourceStills(const std::vector<std::string>& filenames);
    ~VideoSourceStills();

    VideoSourceStills(const VideoSourceStills&) = delete;
    VideoSourceStills(VideoSourceStills&&) = delete;
    VideoSourceStills& operator=(const VideoSourceStills&) = delete;
    VideoSourceStills& operator=(VideoSourceStills&&) = delete;

    Frame operator()(int i = -1) override;
    std::size_t count() const override;
    bool isRandomAccess() const override { return true; }

protected:
    std::unique_ptr<Impl> m_impl;
};

DRISHTI_VIDEOIO_NAMESPACE_END

#endif // __videoio_VideoSourceStills_h__
