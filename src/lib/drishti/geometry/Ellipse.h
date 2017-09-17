/*! -*-c++-*-
  @file   Ellipse.h
  @author David Hirvonen
  @brief  Declaration of an Ellipse class with various geometric operations.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This file is released under the 3 Clause BSD License (exceptions noted below)}

  1) class Conicsection_  
     License: MIT 
     https://github.com/LeszekSwirski/pupiltracker/

  2) cv::RotatedRect ConicSection_<T>::getEllipse()
     License: MIT
     http://research.microsoft.com/en-us/um/people/awf/ellipse/fitellipse.html

*/

#ifndef __drishti_geometry_Ellipse_h__
#define __drishti_geometry_Ellipse_h__

#include "drishti/geometry/drishti_geometry.h"
#include "drishti/geometry/fitEllipse.h"
#include "drishti/geometry/ConicSection.h"

#include <opencv2/core/core.hpp>

#include <vector>
#include <iostream>

DRISHTI_GEOMETRY_BEGIN

class Ellipse : public cv::RotatedRect
{
public:
    Ellipse() {}
    Ellipse(const Ellipse& src);
    Ellipse(const cv::Vec6d& par);
    Ellipse(const cv::RotatedRect& cen);
    Ellipse(const cv::RotatedRect& cen, const cv::Vec6d& par);

    const cv::Vec6d& getParametric() const
    {
        return m_par;
    }
    operator cv::Vec6d() const
    {
        return m_par;
    }

    cv::RotatedRect getEllipse() const;
    cv::Point2f getMajorAxisPos() const;
    cv::Point2f getMajorAxisNeg() const;
    cv::Point2f getMinorAxisPos() const;
    cv::Point2f getMinorAxisNeg() const;

protected:
    cv::Vec6d m_par; // homogeneous
};

void ellipse(cv::Mat& image, const Ellipse& e, const cv::Scalar& color, int width, int type);

std::vector<float> pointsToPhi(const std::vector<cv::Point2f>& points);
std::vector<float> ellipseToPhi(const cv::RotatedRect& e);
cv::RotatedRect phiToEllipse(const std::vector<float>& phi);

typedef std::vector<cv::Point2f> PointVec;
void ellipse2Poly(const cv::RotatedRect& ellipse, float delta, std::vector<cv::Point2f>& points);

// see: http://research.microsoft.com/en-us/um/people/awf/ellipse/fitellipse.html
template <typename T>
cv::RotatedRect getEllipse(const ConicSection_<T>& C)
{
    return conicPar2Cen({ C.A, C.B, C.C, C.D, C.E, C.F });
}

// TODO: There is still ambiguity in orientation when aspect ratio is 1
template <typename T>
inline cv::RotatedRect operator*(const cv::Matx<T, 3, 3>& H, const cv::RotatedRect& e)
{
    cv::Matx<T, 3, 3> Hinv = H.inv();
    cv::Matx<T, 3, 3> C = ConicSection_<T>(e).getMatrix();
    C = Hinv.t() * C * Hinv;

    cv::RotatedRect e2 = getEllipse(ConicSection_<float>(C));
    cv::RotatedRect d[4] = { e2, e2, e2, e2 };
    std::swap(d[2].size.width, d[2].size.height);
    std::swap(d[3].size.width, d[3].size.height);

    d[1].angle = std::fmod(d[1].angle + 180.0, 360.0);
    d[2].angle = std::fmod(d[2].angle + 90.00, 360.0);
    d[3].angle = std::fmod(d[3].angle + 270.0, 360.0);

    // The conic transformation doesn't preserve orientation, so we fix that here:
    // TOOD: more elegant solution?
    cv::Point2f p1 = Ellipse(e).getMajorAxisPos();
    cv::Point2f c1 = e.center;
    cv::Point3f p2 = H * cv::Point3f(p1.x, p1.y, 1.f);
    cv::Point3f c2 = H * cv::Point3f(c1.x, c1.y, 1.f);
    cv::Point2f v2 = cv::Point2f(p2.x / p2.z, p2.y / p2.z) - cv::Point2f(c2.x / c2.z, c2.y / c2.z);

    // Here v2 represents the angle of the transformed (previously major) axis:
    v2 *= (1.0 / cv::norm(v2));

    std::pair<int, float> best(0, 0);

    // Orientation singularity for aspect ratio of 1.0
    if (std::abs(e.size.width / e.size.height - 1.f) < 1e-6f)
    {
        best.first = 0;
        d[0].angle = std::atan2(v2.y, v2.x);
    }
    else
    {
        // v1 : orientation of ellipse from conic
        // v2 : orientation of ellipse major axis from homography
        for (int i = 0; i < 4; i++)
        {
            cv::Point2f v1 = Ellipse(d[i]).getMajorAxisPos() - d[i].center; // orientation of transformed ellipse
            v1 *= (1.0 / cv::norm(v1));
            float score = v1.dot(v2);
            if (score > best.second)
            {
                best = std::make_pair(i, score);
            }
        }
    }

    return d[best.first];
}

inline std::vector<float> ellipseToVector(const cv::RotatedRect& e)
{
    return std::vector<float>{ e.center.x, e.center.y, e.size.width, e.size.height, e.angle };
}

inline std::vector<cv::Point2f> ellipseToPoints(const cv::RotatedRect& e)
{
    return std::vector<cv::Point2f>{ { e.center.x, 0 }, { e.center.y, 0 }, { e.size.width, 0 }, { e.size.height }, { e.angle, 0 } };
}

inline cv::RotatedRect pointsToEllipse(const cv::Point2f* p)
{
    return cv::RotatedRect({ p[0].x, p[1].x }, { p[2].x, p[3].x }, p[4].x);
}

inline cv::RotatedRect pointsToEllipse(const std::vector<cv::Point2f>& p)
{
    return cv::RotatedRect({ p[0].x, p[1].x }, { p[2].x, p[3].x }, p[4].x);
}

inline std::vector<cv::RotatedRect> pointsToEllipses(const std::vector<cv::Point2f>& p)
{
    std::vector<cv::RotatedRect> ellipses;
    size_t n = p.size() / 5;
    for (size_t i = 0; i < n; i++)
    {
        ellipses.push_back(pointsToEllipse(&p[i * 5]));
    }
    return ellipses;
}

DRISHTI_GEOMETRY_END

#endif /* defined(__drishti_geometry_Ellipse_h__) */
