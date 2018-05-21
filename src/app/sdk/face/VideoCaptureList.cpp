#include "VideoCaptureList.h"

#include <fstream>
#include <istream>
#include <iterator>
#include <memory>

// https://stackoverflow.com/a/1567703
class Line
{
    std::string data;

public:
    friend std::istream& operator>>(std::istream& is, Line& l)
    {
        std::getline(is, l.data);
        return is;
    }
    operator std::string() const { return data; }
};

namespace detail
{
    template <typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args&&... args)
    {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}

static void expand(const std::string& filename, std::vector<std::string>& filenames)
{
    // Create input file list (or single image)
    filenames = { filename };
    if (filename.find(".txt") != std::string::npos)
    {
        std::ifstream ifs(filename);
        if (ifs)
        {
            filenames.clear();
            std::copy(std::istream_iterator<Line>(ifs), std::istream_iterator<Line>(), std::back_inserter(filenames));
        }
        else
        {
            throw std::runtime_error("Unable to open file: " + filename);
        }
    }
}

struct VideoCaptureList::Impl
{
    Impl(const std::string& filename)
    {
        expand(filename, filenames);
        init();
    }

    Impl(const std::vector<std::string>& filenames)
        : filenames(filenames)
    {
        init();
    }

    void next()
    {
        image = (++iter == filenames.end()) ? cv::imread(*iter) : cv::Mat();
    }

    void init()
    {
        iter = filenames.begin();
        image = cv::imread(*iter);
    }

    cv::Mat image;
    std::vector<std::string> filenames;
    std::vector<std::string>::iterator iter;
};

VideoCaptureList::VideoCaptureList(const std::string& filename)
{
    m_impl = detail::make_unique<Impl>(filename);
}

VideoCaptureList::VideoCaptureList(const std::vector<std::string>& filenames)
{
    m_impl = detail::make_unique<Impl>(filenames);
}

VideoCaptureList::~VideoCaptureList() = default;

bool VideoCaptureList::grab()
{
    return false;
}

bool VideoCaptureList::isOpened() const
{
    return m_impl->filenames.size() > 0;
}

void VideoCaptureList::release()
{
    m_impl->filenames.clear();
    m_impl->image.release();
}

bool VideoCaptureList::open(const cv::String& filename)
{
    m_impl = detail::make_unique<Impl>(filename);
    return !m_impl->image.empty();
}

bool VideoCaptureList::read(cv::OutputArray image)
{
    if (m_impl->iter != m_impl->filenames.end())
    {
        image.assign(m_impl->image);
        m_impl->next();
        return true;
    }
    return false;
}

double VideoCaptureList::get(int propId) const
{
    switch (propId)
    {
        case CV_CAP_PROP_FRAME_WIDTH:
            return static_cast<double>(m_impl->image.cols);
        case CV_CAP_PROP_FRAME_HEIGHT:
            return static_cast<double>(m_impl->image.rows);
        case CV_CAP_PROP_FRAME_COUNT:
            return static_cast<double>(m_impl->filenames.size());
        default:
            return 0.0;
    }
}
