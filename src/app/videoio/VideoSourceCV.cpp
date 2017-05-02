#include "VideoSourceCV.h"

#include "VideoSourceStills.h"
#if defined(__APPLE__) && defined(DRISHTI_USE_AVFOUNDATION)
#  include "VideoSourceApple.h"
#endif

#include "drishti/core/drishti_string_hash.h"

#include <boost/filesystem.hpp>

#include <algorithm>
#include <locale>

namespace bfs = boost::filesystem;

using string_hash::operator "" _hash;

std::shared_ptr<VideoSourceCV> VideoSourceCV::create(const std::string &filename)
{
    std::string ext = bfs::path(filename).extension().string();

    std::locale loc;
    for(auto &elem : ext)
    {
        elem = std::tolower(elem, loc);
    }

    switch(string_hash::hash(ext))
    {
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
        return std::make_shared<VideoSourceStills>(std::vector<std::string>{filename});
        break;

        // not supported
    default: CV_Assert(false);
        break;
    }

    return nullptr;
}
