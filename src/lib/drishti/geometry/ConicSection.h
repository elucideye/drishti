#ifndef __drishti_geometry_ConicSection_h__
#define __drishti_geometry_ConicSection_h__ 1

#include "drishti/geometry/drishti_geometry.h"

#include <opencv2/core/core.hpp>

#include <cmath>

DRISHTI_GEOMETRY_BEGIN

// Source:
// https://github.com/LeszekSwirski/pupiltracker/blob/master/lib/pupiltracker/ConicSection.h
//
// License:
// https://raw.githubusercontent.com/LeszekSwirski/pupiltracker/master/LICENSE.md
//
// The MIT License (MIT)
//
// Copyright (c) 2014 Lech Swirski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

template <typename T>
class ConicSection_
{
public:
    T A, B, C, D, E, F;

    ConicSection_(cv::RotatedRect r)
    {
        cv::Point_<T> axis((T)std::cos(CV_PI / 180.0 * r.angle), (T)std::sin(CV_PI / 180.0 * r.angle));
        cv::Point_<T> centre(r.center);
        T a = r.size.width / 2;
        T b = r.size.height / 2;

        initFromEllipse(axis, centre, a, b);
    }

    ConicSection_(const cv::Matx<T, 3, 3>& C_)
    {
        A = C_(0, 0);
        B = C_(0, 1) * T(2);
        C = C_(1, 1);
        D = C_(0, 2) * T(2);
        E = C_(1, 2) * T(2);
        F = C_(2, 2);
    }

    T algebraicDistance(const cv::Point_<T>& p)
    {
        return A * p.x * p.x + B * p.x * p.y + C * p.y * p.y + D * p.x + E * p.y + F;
    }

    T distance(const cv::Point_<T>& p)
    {
        //    dist
        // -----------
        // |grad|^0.45

        T dist = algebraicDistance(p);
        cv::Point_<T> grad = algebraicGradient(p);

        T sqgrad = grad.dot(grad);

        return dist / std::pow(sqgrad, T(0.45 / 2));
    }

    cv::Point_<T> algebraicGradient(const cv::Point_<T>& p)
    {
        return cv::Point_<T>(2 * A * p.x + B * p.y + D, B * p.x + 2 * C * p.y + E);
    }

    cv::Point_<T> algebraicGradientDir(const cv::Point_<T>& p)
    {
        cv::Point_<T> grad = algebraicGradient(p);
        T len = std::sqrt(grad.ddot(grad));
        grad.x /= len;
        grad.y /= len;
        return grad;
    }

    cv::Matx33f getMatrix() const
    {
        return cv::Matx33f(A, B / 2.0, D / 2.0, B / 2.0, C, E / 2.0, D / 2.0, E / 2.0, F);
    }

protected:
    void initFromEllipse(const cv::Point_<T>& axis, const cv::Point_<T>& centre, T a, T b)
    {
        T a2 = a * a;
        T b2 = b * b;

        T axx = axis.x * axis.x, cxx = centre.x * centre.x;
        T ayy = axis.y * axis.y, cyy = centre.y * centre.y;
        T axy = axis.x * axis.y, cxy = centre.x * centre.y;

        A = axx / a2 + ayy / b2;
        B = 2 * axy / a2 - 2 * axy / b2;
        C = ayy / a2 + axx / b2;
        D = (-2 * axy * centre.y - 2 * axx * centre.x) / a2 + (2 * axy * centre.y - 2 * ayy * centre.x) / b2;
        E = (-2 * axy * centre.x - 2 * ayy * centre.y) / a2 + (2 * axy * centre.x - 2 * axx * centre.y) / b2;
        F = (2 * axy * cxy + axx * cxx + ayy * cyy) / a2 + (-2 * axy * cxy + ayy * cxx + axx * cyy) / b2 - 1;
    }
};

DRISHTI_GEOMETRY_END

#endif // __drishti_geometry_ConicSection_h__ 1
