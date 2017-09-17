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

    VideoSourceStills(const std::string& filename);
    VideoSourceStills(const std::vector<std::string>& filenames);
    ~VideoSourceStills();
    virtual Frame operator()(int i = -1);
    virtual std::size_t count() const;
    virtual bool isRandomAccess() const { return true; }

protected:
    std::unique_ptr<Impl> m_impl;
};

DRISHTI_VIDEOIO_NAMESPACE_END

#endif // __videoio_VideoSourceStills_h__
