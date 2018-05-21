#include <opencv2/highgui.hpp>
#include <memory>

#ifndef __VideoCaptureList_h__
#define __VideoCaptureList_h__

class VideoCaptureList : public cv::VideoCapture
{
public:
    VideoCaptureList(const std::string& filename);
    VideoCaptureList(const std::vector<std::string>& filenames);
    virtual ~VideoCaptureList();
    virtual bool grab();
    virtual bool isOpened() const;
    virtual void release();
    virtual bool open(const cv::String& filename);
    virtual bool read(cv::OutputArray image);
    double get(int propId) const;

    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

#endif // __VideoCaptureList_h__
