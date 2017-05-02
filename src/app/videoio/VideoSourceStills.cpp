#include "VideoSourceStills.h"

#include "drishti/core/make_unique.h"
#include "drishti/testlib/drishti_cli.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class VideoSourceStills::Impl
{
public:
    Impl(const std::string &filename) : m_filename(filename)
    {
        m_filenames = drishti::cli::expand(m_filename);
    }
    
    Impl(const std::vector<std::string> &filenames) : m_filenames(filenames) {}
    
    ~Impl() {}

    std::size_t count() const
    {
        return m_filenames.size();
    }

    VideoSourceCV::Frame operator()(int i=-1)
    {
        // TODO: throw if range is incorrect
        return VideoSourceCV::Frame(cv::imread(m_filenames[i]), i, m_filenames[i]);
    }

    std::string m_filename;    
    std::vector<std::string> m_filenames;
};

VideoSourceStills::VideoSourceStills(const std::string &filename)
{
    m_impl = drishti::core::make_unique<Impl>(filename);
}

VideoSourceStills::VideoSourceStills(const std::vector<std::string> &filenames)
{
    m_impl = drishti::core::make_unique<Impl>(filenames);
}

VideoSourceStills::~VideoSourceStills()
{
    
}

std::size_t VideoSourceStills::count() const
{
    return m_impl->count();
}

VideoSourceCV::Frame VideoSourceStills::operator()(int i)
{
    return (*m_impl)(i);
}
