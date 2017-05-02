#include "drishti/geometry/fitEllipse.h"

// see:
// http://research.microsoft.com/en-us/um/people/awf/ellipse/fitellipse.html
// http://research.microsoft.com/en-us/um/people/awf/ellipse/fitting.tar.gz
// fitting/conic_cen2par.m

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

// CONIC_CEN2PAR Convert central conic (cx, cy, rx, ry, theta) to parametric
cv::Vec6d conicCen2Par(const cv::RotatedRect& cen)
{
    double cx = cen.center.x;
    double cy = cen.center.y;
    double rx = cen.size.width / 2.0;
    double ry = cen.size.height / 2.0;
    double theta = -(cen.angle) * M_PI / 180.0; // cv::Rect rotation wrt Y

    double Rx = sign(rx) / pow2(rx);
    double Ry = sign(ry) / pow2(ry);
    double cost = cos(theta);
    double sint = sin(theta);
    double cost2 = pow2(cost);
    double sint2 = pow2(sint);
    double sin2t = 2 * sint * cost;
    double Axx = cost2 * Rx + sint2 * Ry;
    double Axy = sin2t * (Ry - Rx);
    double Ayy = Rx * sint2 + Ry * cost2;
    double Ax = sin2t * (Rx - Ry) * cy - 2.0 * cx * (Ry * sint2 + Rx * cost2);
    double Ay = sin2t * (Rx - Ry) * cx - 2.0 * cy * (Ry * cost2 + Rx * sint2);
    double Ao = Ry * pow2(cx * sint + cy * cost) + Rx * pow2(cx * cost - cy * sint) - 1.0;

    cv::Vec6d par(Ao, Ax, Ay, Axx, Ayy, Axy);
    return par;
}

DRISHTI_GEOMETRY_END
