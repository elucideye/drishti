/*!
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

#ifndef __drishtisdk__Ellipse__
#define __drishtisdk__Ellipse__

#include "geometry/drishti_geometry.h"

#include <opencv2/core/core.hpp>

#include <vector>

#include <iostream>

DRISHTI_GEOMETRY_BEGIN


template <typename T> T sign(T A)
{
    return T(int(A > 0) - int(A < 0));
}

// https://github.com/LeszekSwirski/pupiltracker/blob/master/lib/pupiltracker/ConicSection.h
// License: MIT License (MIT)

template <typename T>
class ConicSection_
{
public:
    T A,B,C,D,E,F;

    ConicSection_(cv::RotatedRect r)
    {
        cv::Point_<T> axis((T)std::cos(CV_PI/180.0 * r.angle), (T)std::sin(CV_PI/180.0 * r.angle));
        cv::Point_<T> centre(r.center);
        T a = r.size.width/2;
        T b = r.size.height/2;

        initFromEllipse(axis, centre, a, b);
    }

    ConicSection_(const cv::Matx<T, 3, 3> &C_)
    {
        A = C_(0,0);
        B = C_(0,1)*T(2);
        C = C_(1,1);
        D = C_(0,2)*T(2);
        E = C_(1,2)*T(2);
        F = C_(2,2);
    }

    T algebraicDistance(cv::Point_<T> p)
    {
        return A*p.x*p.x + B*p.x*p.y + C*p.y*p.y + D*p.x + E*p.y + F;
    }

    T distance(cv::Point_<T> p)
    {
        //    dist
        // -----------
        // |grad|^0.45

        T dist = algebraicDistance(p);
        cv::Point_<T> grad = algebraicGradient(p);

        T sqgrad = grad.dot(grad);

        return dist / std::pow(sqgrad, T(0.45/2));
    }

    cv::Point_<T> algebraicGradient(cv::Point_<T> p)
    {
        return cv::Point_<T>(2*A*p.x + B*p.y + D, B*p.x + 2*C*p.y + E);
    }

    cv::Point_<T> algebraicGradientDir(cv::Point_<T> p)
    {
        cv::Point_<T> grad = algebraicGradient(p);
        T len = std::sqrt(grad.ddot(grad));
        grad.x /= len;
        grad.y /= len;
        return grad;
    }

    cv::Matx33f getMatrix() const
    {
        return cv::Matx33f(A, B/2.0, D/2.0, B/2.0, C, E/2.0, D/2.0, E/2.0, F);
    }

    // Copyright (c) 1999, Andrew Fitzgibbon, Maurizio Pilu, Bob Fisher
    // 
    // Permission is hereby granted, free of charge, to any person obtaining a
    // copy of this software and associated documentation files (the "Software"),
    // to deal in the Software without restriction, including without limitation
    // the rights to use, copy, modify, merge, publish, distribute, sublicense,
    // and/or sell copies of the Software, and to permit persons to whom the
    // Software is furnished to do so, subject to the following conditions:
    // 
    // The above copyright notice and this permission notice shall be included in
    // all copies or substantial portions of the Software.
    // 
    // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    // IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    // FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    // AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    // LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    // FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    // DEALINGS IN THE SOFTWARE.

    // see: http://research.microsoft.com/en-us/um/people/awf/ellipse/fitellipse.html    
                                  
    cv::RotatedRect getEllipse() const
    {
        // Convert to geometric radii, and centers
        double a = A, b = B, c = C, d = D, e = E, f = F;
        double thetarad = 0.5*atan2(b,a-c);
        double cost = cos(thetarad);
        double sint = sin(thetarad);
        double sin_squared = sint*sint;
        double cos_squared = cost*cost;
        double cos_sin = sint*cost;
        double Ao = f;
        double Au = +d * cost + e * sint;
        double Av = -d * sint + e * cost;
        double Auu = a*cos_squared+c*sin_squared+b*cos_sin;
        double Avv = a*sin_squared+c*cos_squared-b*cos_sin;

        // ellipse = (centrex,centrey,ax,ay,orientation)
        if(Auu==0 || Avv==0)
        {
            return cv::RotatedRect(cv::Point2f(0,0), cv::Size2f(0,0), 0 );
        }

        // ROTATED = [Ao Au Av Auu Avv]
        double tuCentre = -Au/(2.*Auu);
        double tvCentre = -Av/(2.*Avv);
        double wCentre = Ao-Auu*tuCentre*tuCentre-Avv*tvCentre*tvCentre;
        double uCentre = tuCentre*cost-tvCentre*sint;
        double vCentre = tuCentre*sint+tvCentre*cost;
        double Ru = -wCentre/Auu;
        double Rv = -wCentre/Avv;
        Ru = std::sqrt(std::abs(Ru)) * sign(Ru);
        Rv = std::sqrt(std::abs(Rv)) * sign(Rv);

        // OpenCV format:
        cv::Point2f center(uCentre, vCentre);
        cv::Size2f size(Ru*2.0, Rv*2.0);
        return cv::RotatedRect(center, size, thetarad * 180.0/M_PI);
    }

protected:

    void initFromEllipse(const cv::Point_<T> &axis, const cv::Point_<T> &centre, T a, T b)
    {
        T a2 = a * a;
        T b2 = b * b;

        T axx = axis.x*axis.x, cxx = centre.x * centre.x;
        T ayy = axis.y*axis.y, cyy = centre.y * centre.y;
        T axy = axis.x*axis.y, cxy = centre.x * centre.y;

        A = axx / a2 + ayy / b2;
        B = 2*axy / a2 - 2*axy / b2;
        C = ayy / a2 + axx / b2;
        D = (-2*axy*centre.y - 2*axx*centre.x) / a2 + (2*axy*centre.y - 2*ayy*centre.x) / b2;
        E = (-2*axy*centre.x - 2*ayy*centre.y) / a2 + (2*axy*centre.x - 2*axx*centre.y) / b2;
        F = (2*axy*cxy + axx*cxx + ayy*cyy) / a2 + (-2*axy*cxy + ayy*cxx + axx*cyy) / b2 - 1;
    }
};

class Ellipse : public cv::RotatedRect
{
public:

    Ellipse() {}
    Ellipse(const Ellipse &src);
    Ellipse(const cv::Vec6d &par);
    Ellipse(const cv::RotatedRect &cen);
    Ellipse(const cv::RotatedRect &cen, const cv::Vec6d &par);

    const cv::Vec6d & getParametric() const
    {
        return m_par;
    }
    operator cv::Vec6d() const
    {
        return m_par;
    }

    static cv::Vec6d conicCen2Par(const cv::RotatedRect &cen);

    static double conicResidualSam(const cv::Point2d &P, const cv::Vec6d &a);
    static void conicResidualSam(const std::vector<cv::Point2d> &P, const cv::Vec6d &a, std::vector<double> &D);

    static cv::RotatedRect fitEllipse(const std::vector<cv::Point2d> &pts);
    static cv::RotatedRect fitEllipse(const std::vector<cv::Point2d> &points, const cv::Point2d &center);

    cv::Point2f getMajorAxisPos() const;
    cv::Point2f getMajorAxisNeg() const;
    cv::Point2f getMinorAxisPos() const;
    cv::Point2f getMinorAxisNeg() const;

protected:

    // NOTE: Not inverse of conicCen2Par() TODO
    static cv::RotatedRect conicPar2Cen(const cv::Vec6d &par);

    cv::Vec6d m_par; // homogeneous
};

class EllipseSerializer : public cv::RotatedRect
{
public:

    EllipseSerializer() {}
    EllipseSerializer(const cv::RotatedRect &e) : cv::RotatedRect(e) {}
    void read(const cv::FileNode& node);
    void write(cv::FileStorage& fs) const;
};

void write(cv::FileStorage& fs, const std::string&, const EllipseSerializer& x);
void read(const cv::FileNode& node, EllipseSerializer& x, const EllipseSerializer & default_value);

void ellipse(cv::Mat &image, const Ellipse &e, const cv::Scalar &color, int width, int type);

// intersectConicLine - given a line and a conic detect the real intersection
// points
//          - Pierluigi Taddei (pierluigi.taddei@polimi.it)
//
// Usage:   P = intersectConicLine(C, l)
//
// Arguments:
//           C - homogeneous conic matrix
//           l - homogeneous line vector
//
//           P - matrix of homogeneous intersection points (each column is a
//           point)
// 08.3.2007 : Created

int intersectConicLine(const cv::Matx33f &C, const cv::Vec3f &l, cv::Vec3f P[2]);
void getPointsOnLine(const cv::Vec3f &l, cv::Vec3f &p1, cv::Vec3f &p2);

std::vector<float> pointsToPhi(const std::vector<cv::Point2f> &points);
std::vector<float> ellipseToPhi(const cv::RotatedRect &e);
cv::RotatedRect phiToEllipse(const std::vector<float> &phi, bool transpose=false);

typedef std::vector<cv::Point2f> PointVec;
void ellipse2Poly( const cv::RotatedRect &ellipse, float delta, std::vector<cv::Point2f>& points );

// TODO: There is still ambiguity in orientation when aspect ratio is 1
template <typename T>
inline cv::RotatedRect operator *(const cv::Matx<T,3,3> &H, const cv::RotatedRect &e)
{
    cv::Matx<T,3,3> Hinv = H.inv();
    cv::Matx<T,3,3> C = ConicSection_<T>(e).getMatrix();
    C = Hinv.t() * C * Hinv;

    cv::RotatedRect e2 = drishti::geometry::ConicSection_<float>(C).getEllipse();
    cv::RotatedRect d[4] = { e2,e2,e2,e2 };
    std::swap(d[2].size.width, d[2].size.height);
    std::swap(d[3].size.width, d[3].size.height);

    d[1].angle = std::fmod(d[1].angle + 180.0, 360.0);
    d[2].angle = std::fmod(d[2].angle + 90.00, 360.0);
    d[3].angle = std::fmod(d[3].angle + 270.0, 360.0);

    // The conic transformation doesn't preserve orientation, so we fix that here:
    // TOOD: more elegant solution?
    cv::Point2f p1 = Ellipse(e).getMajorAxisPos();
    cv::Point2f c1 = e.center;
    cv::Point3f p2 = H * cv::Point3f(p1.x,p1.y,1.f);
    cv::Point3f c2 = H * cv::Point3f(c1.x,c1.y,1.f);
    cv::Point2f v2 = cv::Point2f(p2.x/p2.z,p2.y/p2.z) - cv::Point2f(c2.x/c2.z,c2.y/c2.z);

    // Here v2 represents the angle of the transformed (previously major) axis:
    v2 *= (1.0 / cv::norm(v2));

    std::pair<int, float> best(0, 0);

    // Orientation singularity for aspect ratio of 1.0
    if(std::abs(e.size.width/e.size.height - 1.f) < 1e-6f)
    {
        best.first = 0;
        d[0].angle = std::atan2(v2.y, v2.x);
    }
    else
    {
        // v1 : orientation of ellipse from conic
        // v2 : orientation of ellipse major axis from homography
        for(int i = 0; i < 4; i++)
        {
            cv::Point2f v1 = Ellipse(d[i]).getMajorAxisPos() - d[i].center; // orientation of transformed ellipse
            v1 *= (1.0 / cv::norm(v1));
            float score = v1.dot(v2);
            if(score > best.second)
            {
                best = std::make_pair(i, score);
            }
        }
    }

    return d[best.first];
}

inline std::vector<float> ellipseToVector(const cv::RotatedRect &e)
{
    return std::vector<float> {e.center.x,e.center.y,e.size.width,e.size.height,e.angle};
}

inline std::vector<cv::Point2f> ellipseToPoints(const cv::RotatedRect &e)
{
    return std::vector<cv::Point2f> {{e.center.x,0},{e.center.y,0},{e.size.width,0},{e.size.height},{e.angle,0}};
}

inline cv::RotatedRect pointsToEllipse(const cv::Point2f *p)
{
    return cv::RotatedRect({p[0].x,p[1].x}, {p[2].x,p[3].x}, p[4].x);
}

inline cv::RotatedRect pointsToEllipse(const std::vector<cv::Point2f> &p)
{
    return cv::RotatedRect({p[0].x,p[1].x}, {p[2].x,p[3].x}, p[4].x);
}

inline std::vector<cv::RotatedRect> pointsToEllipses(const std::vector<cv::Point2f> &p)
{
    std::vector<cv::RotatedRect> ellipses;
    size_t n = p.size() / 5;
    for(size_t i = 0; i < n; i++)
    {
        ellipses.push_back(pointsToEllipse(&p[i*5]));
    }
    return ellipses;
}

DRISHTI_GEOMETRY_END

#endif /* defined(__drishtisdk__Ellipse__) */
