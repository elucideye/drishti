/*! -*-c++-*-
 @file   videoio/VideoSourceCV.cpp
 @author David Hirvonen
 @brief  Simple OpenCV cv::Mat VideoSource interface implementation.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "videoio/VideoSourceCV.h"
#include "videoio/VideoSourceStills.h"
#include "videoio/VideoSourceTest.h"
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_string_hash.h"

// clang-format off
#if defined(__APPLE__) && defined(DRISHTI_USE_AVFOUNDATION)
#  include "videoio/VideoSourceApple.h"
#endif
// clang-format on

#include <boost/filesystem.hpp>

#include <algorithm>
#include <locale>

namespace bfs = boost::filesystem;

using string_hash::operator"" _hash;

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

std::shared_ptr<VideoSourceCV> VideoSourceCV::create(const std::string& filename)
{
    std::string ext = bfs::path(filename).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](const unsigned char i) { return std::tolower(i); });

    switch (string_hash::hash(ext))
    {
        case ".test"_hash:
            return std::make_shared<VideoSourceTest>(filename);
            break;

        case ".txt"_hash:
            return std::make_shared<VideoSourceStills>(filename);
            break;

#if defined(__APPLE__) && defined(DRISHTI_USE_AVFOUNDATION)
        case ".mov"_hash:
            return std::make_shared<VideoSourceApple>(filename);
            break;
#endif

        // Single image video:
        case ".png"_hash:
        case ".jpg"_hash:
        case ".jpeg"_hash:
            return std::make_shared<VideoSourceStills>(std::vector<std::string>{ filename });
            break;

        // not supported
        default:
            CV_Assert(false);
            break;
    }

    return nullptr;
}

DRISHTI_VIDEOIO_NAMESPACE_END
