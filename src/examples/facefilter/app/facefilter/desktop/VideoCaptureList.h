#ifndef __facefilter_desktop_VideoCaptureList_h__
#define __facefilter_desktop_VideoCaptureList_h__

#include <opencv2/highgui.hpp>
#include <memory>

class VideoCaptureList : public cv::VideoCapture
{
public:
    VideoCaptureList(const std::string& filename);
    VideoCaptureList(const std::vector<std::string>& filenames);
    ~VideoCaptureList() override;

    VideoCaptureList(const VideoCaptureList&) = delete;
    VideoCaptureList(VideoCaptureList&&) = delete;
    VideoCaptureList& operator=(const VideoCaptureList&) = delete;
    VideoCaptureList& operator=(VideoCaptureList&&) = delete;

    bool grab() override;
    bool isOpened() const override;
    void release() override;
    bool open(const cv::String& filename, int apiPreference = cv::CAP_ANY) override;
    bool read(cv::OutputArray image) override;
    double get(int propId) const override;

    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

#endif // __facefilter_desktop_VideoCaptureList_h__
