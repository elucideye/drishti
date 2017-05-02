#ifndef __VideoSourceApple_h__
#define __VideoSourceApple_h__

#include "VideoSourceCV.h"

#include <string>
#include <memory>

class VideoSourceApple : public VideoSourceCV
{
public:
    class Impl;

    VideoSourceApple(const std::string& filename);
    ~VideoSourceApple();
    virtual Frame operator()(int i = -1);
    virtual bool good() const;
    virtual std::size_t count() const;
    virtual bool isRandomAccess() const { return false; }

protected:
    std::unique_ptr<Impl> m_impl;
};

#endif // __VideoSourceApple_h__
