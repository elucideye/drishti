#include "drishti/geometry/fitEllipse.h"

#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/Polynomials>

DRISHTI_GEOMETRY_BEGIN

// http://research.microsoft.com/en-us/um/people/awf/ellipse/fitellipse.html

// FITELLIPSE  Least-squares fit of ellipse to 2D points.
//        A = FITELLIPSE(X,Y) returns the parameters of the best-fit
//        ellipse to 2D points (X,Y).
//        The returned vector A contains the center, radii, and orientation
//        of the ellipse, stored as (Cx, Cy, Rx, Ry, theta_radians)
// 
//        Example:  Run fitellipse without any arguments to get a demo
//
// Authors: Andrew Fitzgibbon, Maurizio Pilu, Bob Fisher
// Reference: "Direct Least Squares Fitting of Ellipses", IEEE T-PAMI, 1999
//
//  @Article{Fitzgibbon99,
//   author = "Fitzgibbon, A.~W.and Pilu, M. and Fisher, R.~B.",
//   title = "Direct least-squares fitting of ellipses",
//   journal = pami,
//   year = 1999,
//   volume = 21,
//   number = 5,
//   month = may,
//   pages = "476--480"
//  }
// 
// This is a more bulletproof version than that in the paper, incorporating
// scaling to reduce roundoff error, correction of behaviour when the input 
// data are on a perfect hyperbola, and returns the geometric parameters
// of the ellipse, rather than the coefficients of the quadratic form.
//
// Copyright (c) 1999, Andrew Fitzgibbon, Maurizio Pilu, Bob Fisher
// 
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


// It's important to scale the image coordinates to be near one before running the algorithm or you'll get numerical instability.
// See the improved matlab code for an implementation which includes this step.

// If you're implementing it in a language other than matlab, there's a better way to solve the eigenvalue problem than we knew when we wrote the paper,
// which involves dividing the matrix into blocks.

// AAAB  [x y]'   = l * DDD0 * [x y]'
// AAAB                 DDD0
// AAAB                 DDD0
// BBBC                 0000

// A * x3 + B * y = l * D * x3
// B * x3 + C * y = 0


// [A  B] [x] = l  [D 0] [x]
// [B' C] [y]      [0 0] [y]
//
// is the same as the pair of equations
//
// A x + B y = l D x
// B' x + C y = 0         ==> y = -inv(C) B' x
//
// so the first eqn is
// A x + B (-inv(C) B') x = l D x
//
// or
//
// inv(D) * (A - B*inv(C)*B') x = l x
// What this means is that you can solve the system by breaking the original matrices up into the blocks A,B,C,D and writing
//
// [allx, allD] = eig(inv(D) * (A - B*inv(C)*B'))
//
//    x = "column of allx corresponding to the positive eigenvalue";
//    y = -inv(C) * B' * x;

//
// [V,D] = eig(A,B) :  A*V = B*V*D
//
// returns diagonal matrix D of generalized eigenvalues and full matrix V whose columns are the corresponding right eigenvectors,
// so that A*V = B*V*D.


// A * V = (lambda) * B * V
// inv(B) * A * V = lambda * V
// C * V = lambda * V where C = inv(B) * A
// Now, the problem has the same for as
//
//    A*V=(Lambda) *V
//
//which means V is an eigen vector of C.

cv::RotatedRect fitEllipse(const std::vector<cv::Point2d> &pts)
{
    std::vector<cv::Point2d> points(pts.size());

    // normalize data
    //   mx = mean(X);
    //   my = mean(Y);
    //   sx = (max(X)-min(X))/2;
    //   sy = (max(Y)-min(Y))/2;
    //   x = (X-mx)/sx;
    //   y = (Y-my)/sy;

    cv::Scalar mu = cv::mean(pts);
    cv::Point2d min = pts[0], max = pts[0];
    for(int i = 1; i < pts.size(); i++)
    {
        if(pts[i].x < min.x)
        {
            min.x = pts[i].x;
        }
        if(pts[i].y < min.y)
        {
            min.y = pts[i].y;
        }
        if(pts[i].x > max.x)
        {
            max.x = pts[i].x;
        }
        if(pts[i].y > max.y)
        {
            max.y = pts[i].y;
        }
    }

    double mx = mu[0];
    double my = mu[1];
    double sx = (max.x - min.x) / 2.0;
    double sy = (max.y - min.y) / 2.0;
    for(int i = 0; i < pts.size(); i++)
    {
        points[i] = cv::Point2d( (pts[i].x - mu[0]) / sx, (pts[i].y - mu[1]) / sy);
    }

    // Build design matrix
    cv::Mat1d D(int(points.size()), 6);
    for (int i = 0; i < points.size(); ++i)
    {
        const auto & p = points[i];
        D(i, 0) = p.x * p.x;
        D(i, 1) = p.x * p.y;
        D(i, 2) = p.y * p.y;
        D(i, 3) = p.x;
        D(i, 4) = p.y;
        D(i, 5) = 1.0;
    }

    // Build scatter matrix, and design matrix:
    cv::Mat1d S = D.t() * D;

    // std::cout << S << std::endl;

    cv::Matx33d C = cv::Matx33d::zeros();
    C(0,2) = C(2, 0) = -2.0;
    C(1,1) = 1.0;

    // Break into blocks
    cv::Matx33d tmpA = S(cv::Range(0,3), cv::Range(0,3)); // tmpA = S(1:3,1:3);
    cv::Matx33d tmpB = S(cv::Range(0,3), cv::Range(3,6)); // tmpB = S(1:3,4:6);
    cv::Matx33d tmpC = S(cv::Range(3,6), cv::Range(3,6)); // tmpC = S(4:6,4:6);
    cv::Matx33d tmpD = C;                                 // tmpD = C(1:3,1:3);
    cv::Matx33d tmpE = tmpC.inv() * tmpB.t();
    cv::Matx33d final = tmpD.inv() * (tmpA - tmpB * tmpE);

    // [evec_x, eval_x] = eig(inv(tmpD) * (tmpA - tmpB*tmpE));
    cv::Matx33d evecs;
    cv::Vec3d evals;
    typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix33;
    Matrix33 F;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            F(i,j) = final(i,j);
        }
    }

    Eigen::EigenSolver<Matrix33> es(F);
    auto evals_ = es.eigenvalues().real();
    auto evecs_ = es.eigenvectors().real();

    for(int i = 0; i < 3; i++)
    {
        evals[i] = evals_[i];
        for(int j = 0; j < 3; j++)
        {
            evecs(i, j) = evecs_(i, j);
        }
    }

    // Find the positive (as det(tmpD) < 0) eigenvalue
    //    I = find(real(diag(eval_x)) < 1e-8 & ~isinf(diag(eval_x)));

    // Extract eigenvector corresponding to negative eigenvalue
    //    A = real(evec_x(:,I));

    std::pair<int, double> best(-1, 1e-8 );
    for(int i = 0; i < 3; i++)
    {
        if( !std::isinf(evals[i]) && (evals[i] < best.second) )
        {
            best = std::make_pair(i, evals[i]);
        }
    }

    if(best.first < 0)
    {
        return cv::RotatedRect();
    }

    cv::Matx31d evec_x = evecs.col(best.first);

    // Recover the bottom half...
    //   evec_y = -tmpE * A;
    //    A = [A; evec_y];

    cv::Matx31d evec_y = -tmpE * evec_x;

    cv::Vec6d A, par;
    for(int i = 0; i < 3; i++)
    {
        A(i+0) = evec_x(i);
        A(i+3) = evec_y(i);
    }

    double sy2 = sy * sy;
    double sxy = sx * sy;
    double sx2 = sx * sx;
    double sxy2 = sx * sy2;
    double sx2y = sx2 * sy;
    double sx2y2 = sx2 * sy2;

    double mx2 = mx * mx;
    double my2 = my * my;
    double mxy = mx * my;

    par[0] = A(0) * sy2;
    par[1] = A(1) * sxy;
    par[2] = A(2) * sx2;
    par[3] = -2.0 * A(0) * sy2 * mx - A(1) * sxy * my + A(3) * sx * sy2;
    par[4] = -A(1) * sxy * mx - 2.0 * A(2) * sx2 * my + A(4) * sx2 * sy;
    par[5] = A(0) * sy2 * mx2 + A(1) * sxy * mxy + A(2) * sx2 * my2 - A(3) * sxy2 * mx - A(4) * sx2y * my + A(5) * sx2y2;

    cv::RotatedRect e = conicPar2Cen( par );

    return e;
}

cv::RotatedRect fitEllipse(const std::vector<cv::Point2d> &points, const cv::Point2d &center)
{
    assert(points.size() >= 3);
    
    // TODO: add normalization
    typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Matrix44d;
    typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix33d;
    typedef Eigen::Matrix<double, 3, 1, Eigen::RowMajor> Matrix31d;
    typedef Eigen::Matrix<double, 1, 3, Eigen::RowMajor> Matrix13d;
    typedef double Matrix11d;

    Eigen::Matrix<double, Eigen::Dynamic, 4, Eigen::RowMajor> D(points.size(), 4);

    // Build design matrix
    for (int i = 0; i < points.size(); ++i)
    {
        const auto  p = points[i] - center;
        D(i, 0) = p.x * p.x;
        D(i, 1) = p.x * p.y;
        D(i, 2) = p.y * p.y;
        D(i, 3) = 1.0;
    }

    // Build scatter matrix, and design matrix:
    Matrix44d S = D.transpose() * D;

    Matrix33d C(3,3);
    C.fill(0);

    C(0,2) = C(2, 0) = -2.0;
    C(1,1) = 1.0;

    // Break into blocks
    auto tmpA = S.block<3,3>(0,0); // tmpA = S(1:3,1:3);  (3x3)
    auto tmpB = S.block<3,1>(0,3); // tmpB = S(1:3,4);    (3x1)
    auto tmpC = S(3,3);            // tmpC = S(4,4);      (1x1)
    const auto & tmpD = C;

    Matrix13d tmpE = (1.0/tmpC) * tmpB.transpose();
    Matrix33d F = tmpD.inverse() * (tmpA - (tmpB * tmpE));

    Eigen::EigenSolver<Matrix33d> es(F);
    auto evals = es.eigenvalues().real();
    auto evecs = es.eigenvectors().real();

    // Find the positive (as det(tmpD) < 0) eigenvalue
    //    I = find(real(diag(eval_x)) < 1e-8 & ~isinf(diag(eval_x)));

    // Extract eigenvector corresponding to negative eigenvalue
    //    A = real(evec_x(:,I));

    std::pair<int, double> best(-1, 1e-8 );
    for(int i = 0; i < 3; i++)
    {
        if( !std::isinf(evals[i]) && (evals[i] < best.second) )
        {
            best = std::make_pair(i, evals[i]);
        }
    }

    if(best.first < 0)
    {
        return cv::RotatedRect();
    }

    auto evec_x = evecs.col(best.first);

    // Recover the bottom half...
    //   evec_y = -tmpE * A;
    //    A = [A; evec_y];

    double evec_y = (-tmpE * evec_x)(0,0);

    cv::Vec6d A;
    for(int i = 0; i < 3; i++)
    {
        A(i) = evec_x(i);
    }

    A(5) = evec_y;

    cv::RotatedRect e = conicPar2Cen(A);
    e.center += cv::Point2f(center.x, center.y);

    return e;
}

DRISHTI_GEOMETRY_END
