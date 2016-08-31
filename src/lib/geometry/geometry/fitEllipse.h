#ifndef drishti_geometry_fit_ellipse_h
#define drishti_geometry_fit_ellipse_h 1

#include "geometry/drishti_geometry.h"

#include <opencv2/core.hpp>

#include <vector>

DRISHTI_GEOMETRY_BEGIN

template <typename T> T sign(T A)
{
    return T(int(A > 0) - int(A < 0));
}

template <typename T> T pow2(const T&x)
{
    return x*x;
}
cv::RotatedRect conicPar2Cen(const cv::Vec6d &par);
cv::Vec6d conicCen2Par(const cv::RotatedRect &cen);

#if !DRISHTI_BUILD_MIN_SIZE
double conicResidualSam(const cv::Point2d &P, const cv::Vec6d &a);
void conicResidualSam(const std::vector<cv::Point2d> &P, const cv::Vec6d &a, std::vector<double> &D);
cv::RotatedRect fitEllipse(const std::vector<cv::Point2d> &pts);
cv::RotatedRect fitEllipse(const std::vector<cv::Point2d> &points, const cv::Point2d &center);
#endif // !DRISHTI_BUILD_MIN_SIZE

DRISHTI_GEOMETRY_END

#endif // drishti_geometry_fit_ellipse_h 1
