#include "drishti/geometry/fitEllipse.h"

#include <cmath>

// Refactored from the original code below:
// http://research.microsoft.com/en-us/um/people/awf/ellipse/fitellipse.html
//
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

DRISHTI_GEOMETRY_BEGIN

cv::RotatedRect conicPar2Cen(const cv::Vec6d& par)
{
    // Convert to geometric radii, and centers
    double thetarad = 0.5 * std::atan2(par[1], (par[0] - par[2]));
    double cost = cos(thetarad);
    double sint = sin(thetarad);
    double sin_squared = sint * sint;
    double cos_squared = cost * cost;
    double cos_sin = sint * cost;

    double Ao = par[5];
    double Au = par[3] * cost + par[4] * sint;
    double Av = -par[3] * sint + par[4] * cost;
    double Auu = par[0] * cos_squared + par[2] * sin_squared + par[1] * cos_sin;
    double Avv = par[0] * sin_squared + par[2] * cos_squared - par[1] * cos_sin;

    // ellipse = (centrex,centrey,ax,ay,orientation)
    if (Auu == 0 || Avv == 0)
    {
        return cv::RotatedRect(cv::Point2f(0.f, 0.f), cv::Size2f(0.f, 0.f), 0.f);
    }

    // ROTATED = [Ao Au Av Auu Avv]
    double tuCentre = -Au / (2.0 * Auu);
    double tvCentre = -Av / (2.0 * Avv);
    double wCentre = Ao - Auu * tuCentre * tuCentre - Avv * tvCentre * tvCentre;

    double uCentre = tuCentre * cost - tvCentre * sint;
    double vCentre = tuCentre * sint + tvCentre * cost;

    double Ru = -wCentre / Auu;
    double Rv = -wCentre / Avv;

    Ru = std::sqrt(std::abs(Ru)) * sign(Ru);
    Rv = std::sqrt(std::abs(Rv)) * sign(Rv);

    // OpenCV format:
    cv::Point2f center(uCentre, vCentre);
    cv::Size2f size(Ru * 2.0, Rv * 2.0);
    return cv::RotatedRect(center, size, thetarad * 180.0 / M_PI);
}

DRISHTI_GEOMETRY_END
