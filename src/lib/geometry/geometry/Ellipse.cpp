/*!
  @file   Ellipse.cpp
  @author David Hirvonen
  @brief  Implementation of an Ellipse class with various geometric operations.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "geometry/Ellipse.h"
#include "core/drishti_core.h"

#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/Polynomials>

#include <opencv2/imgproc.hpp>
#include <iostream>

DRISHTI_GEOMETRY_BEGIN

static cv::Point2f transpose(const cv::Point2f &p)
{
    return cv::Point2f(p.y, -p.x);
}
static cv::Point2f getVectorAtAngle(float angle, float length)
{
    float theta = angle * M_PI / 180.0;
    return cv::Point2f(std::cos(theta), std::sin(theta)) * length;
}

std::vector<float> pointsToPhi(const std::vector<cv::Point2f> &points)
{
    std::vector<float> phi { points[0].x, points[1].x, points[2].x, points[3].x, points[4].x };
    return phi;
}

// Provie common ellipse control points:
cv::Point2f Ellipse::getMajorAxisPos() const
{
    return (center + getVectorAtAngle(angle, size.width/2.0));
}
cv::Point2f Ellipse::getMajorAxisNeg() const
{
    return (center - getVectorAtAngle(angle, size.width/2.0));
}
cv::Point2f Ellipse::getMinorAxisPos() const
{
    return (center + transpose(getVectorAtAngle(angle, size.height/2.0)));
}
cv::Point2f Ellipse::getMinorAxisNeg() const
{
    return (center - transpose(getVectorAtAngle(angle, size.height/2.0)));
}

// (((((((( Ellipse ))))))))
Ellipse::Ellipse(const Ellipse &src) : cv::RotatedRect(src), m_par(src.m_par) {}
Ellipse::Ellipse(const cv::Vec6d &par) : cv::RotatedRect(Ellipse::conicPar2Cen(par)),  m_par(par) {}
Ellipse::Ellipse(const cv::RotatedRect &cen) : cv::RotatedRect(cen), m_par(Ellipse::conicCen2Par(cen)) {}
Ellipse::Ellipse(const cv::RotatedRect &cen, const cv::Vec6d &par) : cv::RotatedRect(cen), m_par(par)
{

}

template <typename T> T pow2(const T&x)
{
    return x*x;
}

// CONIC_CEN2PAR Convert central conic (cx, cy, rx, ry, theta) to parametric
cv::Vec6d Ellipse::conicCen2Par(const cv::RotatedRect &cen)
{
    double cx = cen.center.x;
    double cy = cen.center.y;
    double rx = cen.size.width / 2.0;
    double ry = cen.size.height / 2.0;
    double theta = -(cen.angle) * M_PI / 180.0; // cv::Rect rotation wrt Y

    double Rx = sign(rx)/pow2(rx);
    double Ry = sign(ry)/pow2(ry);
    double cost = cos(theta);
    double sint = sin(theta);
    double cost2 = pow2(cost);
    double sint2 = pow2(sint);
    double sin2t = 2*sint*cost;
    double Axx = cost2*Rx + sint2*Ry;
    double Axy = sin2t*(Ry-Rx);
    double Ayy = Rx*sint2 + Ry*cost2;
    double Ax = sin2t*(Rx - Ry)*cy - 2.0*cx*(Ry*sint2 + Rx*cost2);
    double Ay = sin2t*(Rx - Ry)*cx - 2.0*cy*(Ry*cost2 + Rx*sint2);
    double Ao = Ry*pow2(cx*sint + cy*cost) + Rx*pow2(cx*cost - cy*sint) - 1.0;

    cv::Vec6d par(Ao, Ax, Ay, Axx, Ayy, Axy);
    return par;
}

cv::RotatedRect Ellipse::conicPar2Cen(const cv::Vec6d &par)
{
    // Convert to geometric radii, and centers
    double thetarad = 0.5 * std::atan2(par[1], (par[0] - par[2]));
    double cost = cos(thetarad);
    double sint = sin(thetarad);
    double sin_squared = sint*sint;
    double cos_squared = cost*cost;
    double cos_sin = sint*cost;

    double Ao = par[5];
    double Au =   par[3] * cost + par[4] * sint;
    double Av = - par[3] * sint + par[4] * cost;
    double Auu = par[0] * cos_squared + par[2] * sin_squared + par[1] * cos_sin;
    double Avv = par[0] * sin_squared + par[2] * cos_squared - par[1] * cos_sin;

    // ROTATED = [Ao Au Av Auu Avv]
    double tuCentre = - Au / (2.0*Auu);
    double tvCentre = - Av / (2.0*Avv);
    double wCentre = Ao - Auu * tuCentre * tuCentre - Avv * tvCentre * tvCentre;

    double uCentre = tuCentre * cost - tvCentre * sint;
    double vCentre = tuCentre * sint + tvCentre * cost;

    double Ru = -wCentre / Auu;
    double Rv = -wCentre / Avv;

    Ru = std::sqrt(std::abs(Ru)) * sign(Ru);
    Rv = std::sqrt(std::abs(Rv)) * sign(Rv);

    cv::RotatedRect e( cv::Point2f(uCentre, vCentre), cv::Size2f(Ru, Rv) * 2.f, thetarad * 180.0/M_PI);

    return e;
}

cv::RotatedRect Ellipse::fitEllipse(const std::vector<cv::Point2d> &pts)
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

cv::RotatedRect Ellipse::fitEllipse(const std::vector<cv::Point2d> &points, const cv::Point2d &center)
{
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

// TODO: Taubin's GEV fit (faster)

// function R = conic_residuals_sam(P, a)
// CONIC_RESIDUALS_SAM Sampson distance from conic to points
//
// Author: Andrew Fitzgibbon <awf@robots.ox.ac.uk>
// Date: 01 Oct 96

double Ellipse::conicResidualSam(const cv::Point2d &P, const cv::Vec6d &a)
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

void Ellipse::conicResidualSam(const std::vector<cv::Point2d> &P, const cv::Vec6d &a, std::vector<double> &D)
{
    D.resize(P.size());
    for(int i = 0; i < P.size(); i++)
    {
        D[i] = conicResidualSam(P[i], a);
    }
}

// function D = conic_residuals_geo(P, pvec)
// CONIC_RESIDUALS_GEO Geometric distance from conic to points.
//              Returns the vector of squared geometric distances
//              to points.
//
// Author: Andrew Fitzgibbon <awf@robots.ox.ac.uk>
// Date: 01 Oct 96

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

void ellipse(cv::Mat &image, const Ellipse &e, const cv::Scalar &color, int width, int type)
{
    cv::ellipse(image, e, color, width, type);
    cv::line(image, e.center, e.getMajorAxisPos(), color, width, type);
}

// ######## IO ##########

void EllipseSerializer::read(const cv::FileNode& node)
{
    node["center"] >> center;
    node["size"] >> size;
    node["angle"] >> angle;
};

void EllipseSerializer::write(cv::FileStorage& fs) const
{
    fs << "{" << "center" << center << "size" << size << "angle" << angle << "}";
}

void write(cv::FileStorage& fs, const std::string&, const EllipseSerializer& x)
{
    x.write(fs);
}

void read(const cv::FileNode& node, EllipseSerializer& x, const EllipseSerializer & default_value)
{
    if(node.empty())
    {
        x = default_value;
    }
    else
    {
        x.read(node);
    }
}


void getPointsOnLine(const cv::Vec3f &l, cv::Vec3f &p1, cv::Vec3f &p2)
{
    if((l[0] == 0.f) && (l[1] == 0.f)) // line at infinity
    {
        p1 = {1, 0, 0};
        p2 = {0, 1, 0};
    }
    else
    {
        p2 = { -l[1], l[0], 0 };
        p1 = (std::abs(l[0]) < std::abs(l[1])) ? cv::Vec3f(0, -l[2], l[1]) : cv::Vec3f(-l[2], 0, l[0]);
    }
}

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

int intersectConicLine(const cv::Matx33f &C, const cv::Vec3f &l, cv::Vec3f P[2])
{
    int n = 0;
    cv::Vec3f p1, p2;
    getPointsOnLine(l, p1, p2);

    const auto p1Cp1 = (p1.t() * C * p1)[0];
    const auto p2Cp2 = (p2.t() * C * p2)[0];
    const auto p1Cp2 = (p1.t() * C * p2)[0];
    if(p2Cp2 == 0.f)
    {
        const auto k1 = -0.5 * p1Cp1 / p1Cp2;
        P[0] = (p1 + k1 * p2);
        n = 1;
    }
    else
    {
        const auto delta = p1Cp2*p1Cp2 - p1Cp1*p2Cp2;
        if (delta >= 0.f)
        {
            const auto deltaSqrt = std::sqrt(delta);
            const auto k1 = (-p1Cp2 + deltaSqrt)/p2Cp2;
            const auto k2 = (-p1Cp2 - deltaSqrt)/p2Cp2;
            P[0] = (p1 + k1*p2);
            P[1] = (p1 + k2*p2);
            n = 2;
        }
        else
        {
            // no intersection points
        }
    }
    return n;
}

std::vector<float> ellipseToPhi(const cv::RotatedRect &e)
{
    float cx = e.center.x;
    float cy = e.center.y;
    float angle = (e.angle) * M_PI/180.0;
    float scale = core::logN( e.size.width, 2.0f );
    float aspectRatio = core::logN( e.size.height / e.size.width, 2.0f );
    return std::vector<float> { cx, cy, angle, scale, aspectRatio };
}

// Note: This is for ellipse drawn in transposed image
cv::RotatedRect phiToEllipse(const std::vector<float> &phi, bool transpose)
{
    double width = std::pow(2.0, phi[3]);
    cv::Size2f size(width, std::pow(2.0, phi[4]) * width);
    cv::RotatedRect ellipse(cv::Point2f(phi[0], phi[1]), size, phi[2]*180.0/M_PI);

    if(transpose)
    {
        // Be explicit and apply transpose here:
        std::swap(ellipse.center.x, ellipse.center.y);
        ellipse.angle = atan2(std::cos(phi[2]),std::sin(phi[2])) * 180.0/M_PI;
    }

    return ellipse;
}

// ===== from opencv/imgproc/drawing.cpp:

/*
 constructs polygon that represents elliptic arc.
 */
void ellipse2Poly( const cv::RotatedRect &ellipse, float delta, std::vector<cv::Point2f>& points )
{
    delta *= M_PI/180.0f;
    float theta = ellipse.angle * M_PI/180.0f;
    const float b = ellipse.size.height/2.f;
    const float a = ellipse.size.width/2.f;
    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);
    for(float t = 0.0; t < 2.f*M_PI; t+= delta)
    {
        const float cos_t = std::cos(t);
        const float sin_t = std::sin(t);
        cv::Point2f p(a*cos_t*cos_theta - b*sin_t*sin_theta, a*cos_t*sin_theta + b*sin_t*cos_theta);
        points.push_back(ellipse.center + p);
    }
}

DRISHTI_GEOMETRY_END

