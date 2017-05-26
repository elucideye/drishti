#ifndef __VideoSinkCV_h__
#define __VideoSinkCV_h__

#include <opencv2/core.hpp>

#include <memory>

class VideoSinkCV
{
public:
    VideoSinkCV() {}
    ~VideoSinkCV() {}
    virtual void operator()(const cv::Mat &image) = 0;
    static std::shared_ptr<VideoSinkCV> create(const std::string& filename, const std::string &hint={});
};

#endif // __VideoSinkCV_h__
