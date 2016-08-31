#include "geometry/fitEllipse.h"

#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/Polynomials>

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

// function D = conic_residuals_geo(P, pvec)
// CONIC_RESIDUALS_GEO Geometric distance from conic to points.
//              Returns the vector of squared geometric distances
//              to points.
//
// Author: Andrew Fitzgibbon <awf@robots.ox.ac.uk>
// Date: 01 Oct 96

DRISHTI_GEOMETRY_BEGIN

void conicResiduals(const std::vector<cv::Point2d> &P, const cv::Vec6d &pvec, std::vector<double> &D)
{
    D.resize(P.size());

    const auto &Ao = pvec[0];
    const auto &Ax = pvec[1];
    const auto &Ay = pvec[2];
    const auto &Axx = pvec[3];
    const auto &Ayy = pvec[4];
    const auto &Axy = pvec[5];

    // Rotate the conic
    const double theta = 0.5 * std::atan2(Axy, Axx - Ayy); // TODO: optimize
    const double cost = std::cos(theta);
    const double sint = std::sin(theta);
    const double sin2 = sint*sint;
    const double cos2 = cost*cost;

    // Rotation - free
    double Auu = Axx * cos2 + Ayy * sin2 + Axy * sint * cost;
    double Avv = Axx * sin2 + Ayy * cos2 - Axy * sint * cost;
    const double Au =   Ax * cost + Ay * sint;
    const double Av = - Ax * sint + Ay * cost;

    // calculate translation
    const double tu = Au/(2*Auu);
    const double tv = Av/(2*Avv);
    const double C = Ao - Auu*tu*tu - Avv*tv*tv;
    Auu = -Auu / C;
    Avv = -Avv / C;

    for(int i = 0; i < P.size(); i++)
    {
        const auto & X0 = P[i].x;
        const auto & Y0 = P[i].y;

        // transform start points
        auto U0 = cost * X0 + sint * Y0 + tu;
        auto V0 =-sint * X0 + cost * Y0 + tv;

        // temps
        auto U02 = U0 * U0;
        auto V02 = V0 * V0;
        auto ai = 1.0 / Auu;
        auto bi = 1.0 / Avv;

        // polynomial coeffs
        //typedef Eigen::Matrix<double,5,1,Eigen::RowMajor> Vector5d; Vector5d C(1,5);

        Eigen::Matrix<double,5,1> C(5,1);
        C(0) = 1.0;
        C(1) = -2.0*(ai+bi);
        C(2) = -(U02*ai+V02*bi-ai*ai-4.0*ai*bi-bi*bi);
        C(3) = 2.0*(U02+V02-ai-bi)*ai*bi;
        C(4) = -(U02*bi+V02*ai-ai*bi)*ai*bi;

        Eigen::PolynomialSolver<double, 4> psolve(C);
        auto r = psolve.roots();

        // imagplaces = find(abs(imag(r)) > eps);
        // r(imagplaces) = imagplaces*nan;
        // r = sort(r);
        // R(n,:) = r';

        for(int j = 0; j < r.size(); j++)
        {
            if( std::abs(r[j].imag()) > std::numeric_limits<double>::epsilon() )
            {
                r[j] = std::complex<double>(std::numeric_limits<double>::quiet_NaN());
            }
        }

        // std::sort(r.begin(), r.end());

        // Calculate distances:
        //
        //  U0wide = U0(:, [1 1 1 1]);
        //  V0wide = V0(:, [1 1 1 1]);
        //
        //  Uwide = U0wide ./ (1 - R*Auu);
        //  Vwide = V0wide ./ (1 - R*Avv);
        //
        //  du = Uwide - U0wide;
        //  dv = Vwide - V0wide;
        //  dists = du.*du + dv.*dv;

        // TODO: Is this all needed?

        // cv::Vec4d dists;
        // for(int j = 0; j < 4; j++)
        // {
        //    double du = (U0 / (1.0 - r[j] * Auu))
        //    double dv = (V0 / (1.0 - r[j] * Avv));
        //    dists[j] = (du*du + dv*dv);
        // }

        // sum minimum distances to get error
        // D = dists(:,1);
        // I = find(~finite(D));
        // if ~isempty(I)
        //     fprintf(1, 'I: %d ', I);
        // D(I) = 1e10*ones(size(I));
        // end

        // D[i] = dists[0];

        // TODO
    }
}

DRISHTI_GEOMETRY_END
