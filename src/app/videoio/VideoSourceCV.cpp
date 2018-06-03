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

#include <opencv2/highgui.hpp>

#include <boost/filesystem.hpp>

#include <iostream>
#include <algorithm>
#include <locale>

namespace bfs = boost::filesystem;

using string_hash::operator"" _hash;

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

// Random access VideoSourceCV
class VideoSourceOpenCV : public VideoSourceCV
{
public:
    VideoSourceOpenCV(const std::string& filename, const cv::Size &size)
    {
        if (filename.find_first_not_of("0123456789") == std::string::npos)
        {
            video.open(cv::CAP_ANY + std::stoi(filename));
        }
        else
        {
            video.open(filename);
        }

        // Set to large # to force max operating resolution (this isn't otherwise exposed)
        // https://stackoverflow.com/a/31464688
        //video.set(cv::CAP_PROP_FRAME_WIDTH, 10000.0);
        //video.set(cv::CAP_PROP_FRAME_HEIGHT, 10000.0);

        if(size.area())
        {
            video.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(size.width));
            video.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(size.height));
        }
    }
    ~VideoSourceOpenCV() = default;
    Frame operator()(int i) override
    {
        cv::Mat frame;
        video >> frame;
        return { frame, static_cast<std::size_t>(i) };
    }
    std::size_t count() const override
    {
        return static_cast<int>(video.get(cv::CAP_PROP_FRAME_COUNT));
    }
    bool isRandomAccess() const override
    {
        return false;
    }
protected:
    cv::VideoCapture video;
};


std::shared_ptr<VideoSourceCV> VideoSourceCV::createCV(const std::string& filename, const cv::Size &size)
{
    return std::make_shared<VideoSourceOpenCV>(filename, size);
}

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
