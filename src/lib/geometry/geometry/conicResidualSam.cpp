#include "geometry/fitEllipse.h"

// see:
// http://research.microsoft.com/en-us/um/people/awf/ellipse/fitellipse.html
// http://research.microsoft.com/en-us/um/people/awf/ellipse/fitting.tar.gz
// fitting/conic_residuals_sam.m
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

// TODO: Taubin's GEV fit (faster)
//
// function R = conic_residuals_sam(P, a)
// CONIC_RESIDUALS_SAM Sampson distance from conic to points
//
// Author: Andrew Fitzgibbon <awf@robots.ox.ac.uk>
// Date: 01 Oct 96

DRISHTI_GEOMETRY_BEGIN

double conicResidualSam(const cv::Point2d &P, const cv::Vec6d &a)
{
    const auto & x = P.x;
    const auto & y = P.y;

    double xx = x*x;
    double xy = x*y;
    double yy = y*y;
    double R = a[0] + (a[1] * x) + (a[2] * y) + (a[3] * xx) + (a[4] * yy) + (a[5] * xy);
    double Dx = a[1] + (2.0 * a[3] * x) + a[5] * y;
    double Dy = a[2] + (2.0 * a[4] * y) + a[5] * x;
    double W = Dx*Dx + Dy*Dy;
    double D2 = (R*R) / W;

    return D2;
}

void conicResidualSam(const std::vector<cv::Point2d> &P, const cv::Vec6d &a, std::vector<double> &D)
{
    D.resize(P.size());
    for(int i = 0; i < P.size(); i++)
    {
        D[i] = conicResidualSam(P[i], a);
    }
}

DRISHTI_GEOMETRY_END
