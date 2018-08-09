#include <opencv2/highgui.hpp>
#include <memory>

#ifndef __VideoCaptureList_h__
#define __VideoCaptureList_h__

class VideoCaptureList : public cv::VideoCapture
{
public:
    VideoCaptureList(const std::string& filename);
    VideoCaptureList(const std::vector<std::string>& filenames);
    ~VideoCaptureList() override;
    bool grab() override;
    bool isOpened() const override;
    void release() override;
    bool open(const cv::String& filename) override;
    bool read(cv::OutputArray image) override;
    double get(int propId) const override;

    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

#endif // __VideoCaptureList_h__
