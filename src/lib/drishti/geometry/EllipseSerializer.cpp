#include "drishti/geometry/EllipseSerializer.h"

DRISHTI_GEOMETRY_BEGIN

void EllipseSerializer::read(const cv::FileNode& node)
{
#if !DRISHTI_BUILD_MIN_SIZE
    node["center"] >> center;
    node["size"] >> size;
    node["angle"] >> angle;
#else
    CV_Assert(false);
#endif
};

void EllipseSerializer::write(cv::FileStorage& fs) const
{
#if !DRISHTI_BUILD_MIN_SIZE
    fs << "{"
       << "center" << center << "size" << size << "angle" << angle << "}";
#else
    CV_Assert(false);
#endif
}

void write(cv::FileStorage& fs, const std::string&, const EllipseSerializer& x)
{
#if !DRISHTI_BUILD_MIN_SIZE
    x.write(fs);
#else
    CV_Assert(false);
#endif
}

void read(const cv::FileNode& node, EllipseSerializer& x, const EllipseSerializer& default_value)
{
#if !DRISHTI_BUILD_MIN_SIZE
    if (node.empty())
    {
        x = default_value;
    }
    else
    {
        x.read(node);
    }
#else
    CV_Assert(false);
#endif
}

DRISHTI_GEOMETRY_END
