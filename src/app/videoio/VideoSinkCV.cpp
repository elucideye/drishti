/*! -*-c++-*-
 @file   videoio/VideoSinkCV.h
 @author David Hirvonen
 @brief  Simple OpenCV cv::Mat VideoSink interface implementation.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "videoio/VideoSinkCV.h"

// clang-format off
#if defined(__APPLE__) && defined(DRISHTI_USE_AVFOUNDATION)
#  include "videoio/VideoSinkApple.h"
#endif
// clang-format on

#include "drishti/core/drishti_string_hash.h"

#include <boost/filesystem.hpp>

#include <algorithm>
#include <locale>
#include <memory>

namespace bfs = boost::filesystem;

using string_hash::operator"" _hash;

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

std::shared_ptr<VideoSinkCV> VideoSinkCV::create(const std::string& filename, const std::string& hint)
{
    std::string ext = bfs::path(filename).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](const unsigned char i) { return std::tolower(i); });

    switch (string_hash::hash(ext))
    {
#if defined(__APPLE__) && defined(DRISHTI_USE_AVFOUNDATION)
        case ".mov"_hash:
            return std::make_shared<VideoSinkApple>(filename);
            break;
#endif
        // not supported
        default:
            CV_Assert(false);
            break;
    }

    return nullptr;
}

DRISHTI_VIDEOIO_NAMESPACE_END
