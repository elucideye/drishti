#ifndef __VideoSourceStills_h__
#define __VideoSourceStills_h__

#include "VideoSourceCV.h"

#include <string>
#include <memory>

// Random access VideoSourceCV
class VideoSourceStills : public VideoSourceCV
{
public:
    class Impl;

    VideoSourceStills(const std::string &filename);
    VideoSourceStills(const std::vector<std::string> &filenames);
    ~VideoSourceStills();
    virtual Frame operator()(int i=-1);
    virtual std::size_t count() const;
    virtual bool isRandomAccess() const { return true; }

protected:
    std::unique_ptr<Impl> m_impl;
};

#endif // __VideoSourceApple_h__
