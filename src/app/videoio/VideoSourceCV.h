#ifndef __VideoSourceCV_h__
#define __VideoSourceCV_h__

#include <opencv2/core.hpp>

#include <memory>

class VideoSourceCV
{
public:
    struct Frame
    {
        Frame() {}
        Frame(const cv::Mat& image)
            : image(image)
        {
        }
        Frame(const cv::Mat& image, std::size_t index)
            : image(image)
            , index(index)
        {
        }
        Frame(const cv::Mat& image, std::size_t index, const std::string& name)
            : image(image)
            , index(index)
            , name(name)
        {
        }

        cv::Mat image;
        std::size_t index = 0;
        std::string name;
        std::string metadata;
    };

    VideoSourceCV() {}
    ~VideoSourceCV() {}

    virtual Frame operator()(int i = -1) = 0;
    virtual bool good() const { return true; }
    virtual std::size_t count() const = 0;
    virtual bool isRandomAccess() const = 0;

    static std::shared_ptr<VideoSourceCV> create(const std::string& filename);

protected:
};

#endif // __VideoSourceCV_h__
