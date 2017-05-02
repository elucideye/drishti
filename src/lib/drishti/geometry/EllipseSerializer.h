#ifndef __drishti_geometry_EllipseSerializer_h__
#define __drishti_geometry_EllipseSerializer_h__ 1

#include "drishti/geometry/Ellipse.h"

DRISHTI_GEOMETRY_BEGIN

class EllipseSerializer : public cv::RotatedRect
{
public:
    EllipseSerializer() {}
    EllipseSerializer(const cv::RotatedRect& e)
        : cv::RotatedRect(e)
    {
    }
    void read(const cv::FileNode& node);
    void write(cv::FileStorage& fs) const;
};

void write(cv::FileStorage& fs, const std::string&, const EllipseSerializer& x);
void read(const cv::FileNode& node, EllipseSerializer& x, const EllipseSerializer& default_value);

DRISHTI_GEOMETRY_END

#endif // __drishti_geometry_EllipseSerializer_h__ 1
