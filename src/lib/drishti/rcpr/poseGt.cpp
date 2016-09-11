/*!
  @file   poseGt.cpp
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @author P. Dollár (original matlab code (random ferns))
  @brief  Declaration of utility routines for Cascaded Pose Regression w/ ellipse models.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/drishti_core.h"
#include "drishti/rcpr/CPR.h"

#include <opencv2/imgproc.hpp>

#define DRISHTI_CPR_DO_FTR_DEBUG 0
#define DRISHTI_CPR_DO_FEATURE_MASK 1
#define DRISHTI_CPR_USE_FEATURE_SEPARATION_PRIOR 1

DRISHTI_RCPR_BEGIN

Vector1d operator*(const Vector1d &src, Vector1d::value_type value)
{
    Vector1d dst = src;
    for(auto &p : dst)
    {
        p = (p * value);
    }
    return dst;
}

Vector1d operator+(const Vector1d &a, const Vector1d &b)
{
    Vector1d c(a.size());
    for(int i = 0; i < a.size(); i++)
    {
        c[i] = a[i] + b[i];
    }

    return c;
}

Vector1d operator-(const Vector1d &a, const Vector1d &b)
{
    Vector1d c(a.size());
    for(int i = 0; i < a.size(); i++)
    {
        c[i] = a[i] - b[i];
    }

    return c;
}

Vector1d& operator+=(Vector1d &a, const Vector1d &b)
{
    for(int i = 0; i < a.size(); i++)
    {
        a[i] += b[i];
    }

    return a;
}

Vector1d& operator*=(Vector1d &a, Vector1d::value_type value)
{
    for(int i = 0; i < a.size(); i++)
    {
        a[i] *= value;
    }

    return a;
}

static Matx33Real getPose( const Vector1d &phi  );
static std::vector<uint32_t> xsToInds(const Matx33Real &HS, const PointVec &xs, int w, int h, int nChn, bool doTranspose, int stride);

// function part = createPart( parent, wts )
// % Create single part for model (parent==0 implies root).
// if(nargin<2), wts=[0.313 0.342 11.339 13.059 6.998]; end
// if( parent==0 )
//   part = struct('prn',parent,'lks',[0 0 0 0 1],'joint',[],'mus',[100 100 0 6 -1],'sigs',[10 10 pi .5 .5],'wts',wts);
// else
//   part = struct('prn',parent,'lks',[0 1 1],'joint',[0 -1],'mus',[pi -1 -1],'sigs',[pi/2 .75 .5],'wts',wts(3:5));
// end
// end

int createPart(int parent, CPR::Model::Parts &part)
{
    CV_Assert(parent == 0); // for now, just one part

    part.prn =  { "prn", parent };
    part.lks =  { "lks",  {0.0, 0.0, 0.0, 0.0, 1.0} };
    part.mus =  { "mus",  {100.0, 100.0, 0.0, 6.0, -1.0} };
    part.sigs = { "sigs", {10.0, 10.0, M_PI, 0.5, 0.5 } };
    //part.wts =  { "wts",  {0.313, 0.342, 11.339, 13.059, 6.998 } };
    part.wts =  { "wts",  {0.313, 0.342, 11.339, 13.059, 48.0 } };

    return 0;
}

int createModel(int type, CPR::Model &model)
{
    CPR::Model::Parts parts;
    createPart(0, parts);
    model.parts = { "parts", parts };
    return 0;
}

// function [ftrs,Vs] = ftrsComp( model, phis, Is, ftrData, imgIds )
// Compute pose indexed features on Is given phis.
//
// USAGE
//  [ftrs,Vs] = ftrsComp( model, phis, Is, ftrData, [imgIds] )
//
// INPUTS
//  model    - pose model
//  phis     - [MxR] relative pose for each image [tx ty sx sy theta]
//  Is       - [w x h x nCh x N] input images
//  ftrData  - define ftrs to actually compute, output of ftrsGen
//  imgIds   - [Mx1] image id for each phi (optional if M==N)
//
// OUTPUTS
//  ftrs     - [MxF] computed features
//  Vs       - [w x h x N] visualization of ftr locations
//
// EXAMPLE
//
// See also poseGt, poseGt>ftrsGen

// ########
// NOTE: This is much simpler than the matlab code, since we are only dealing with one image at a time
// ########

using FtrData = CPR::RegModel::Regs::FtrData;
int featuresComp(const CPR::Model &model, const Vector1d &phi, const ImageMaskPair &Im, const FtrData &ftrData, CPR::FeaturesResult &result, bool useNPD)
{
    const auto &I = Im.getImage();
    int h = I.rows;
    int w = I.cols;
    int stride = I.step1();
    int nChn = I.channels();

    // compute image inds from xs adjusted for pose

    Matx33Real HS = getPose( phi ); // just single component model for now (don't need multiple parts)
    //Matx33Real HS = phisToHs( phi );

    const auto &xs = *(ftrData.xs);
    //const auto &pids = *(ftrData.pids);

    // For multi-part regression we would use this
    //int s = pids[0];
    //int e = pids[1];
    //xs({s,e}, cv::Range::all())
    //CV_Assert(s <= e); // for now, this would normally be used for loop control

    std::vector<cv::Point> pts;
    auto && inds = xsToInds(HS, xs, w, h, nChn, DRISHTI_CPR_TRANSPOSE, stride); // TODO: rowStride != cols

#if DRISHTI_CPR_DO_FEATURE_MASK
    const auto &M = Im.getMask();
    auto &mask = result.ftrMask;
#endif

#if DRISHTI_CPR_DO_FTR_DEBUG
    cv::Mat tmp1(M.size(), CV_8UC1, cv::Scalar::all(0)), tmp2 = tmp1.clone();
#endif

    // Compute features:
    CV_Assert( !(inds.size() % 2) );
    auto &ftrs = result.ftrs;

    int size = (w * h);
    for(auto &i : inds)
    {
        i = std::max(std::min(int(i), int(size-1)), 0);
    }

    ftrs.resize(inds.size()/2);
    for(int j = 0, i = 0; i < inds.size(); j++, i+= 2)
    {
        double f1 = double(I.ptr()[inds[i+0]]) / 255.0;
        double f2 = double(I.ptr()[inds[i+1]]) / 255.0;

        double d;
        if(useNPD) // possibly use functor (opt)
        {
            double denom = (f1 + f2);
            d = (denom > 0.0) ? ((f1 - f2) / denom) : 0.0; // NPD
        }
        else
        {
            d = (f1 - f2);
        }

        ftrs[j] =  d;

#if DRISHTI_CPR_DO_FEATURE_MASK
        // Store occlusion estimate
        // TODO: test impact of full and partial occlusion
        if(!M.empty())
        {
            uint8_t m1 = M.ptr()[inds[i+0]];
            uint8_t m2 = M.ptr()[inds[i+1]];
            uint8_t valid = m1 & m2;
            mask.push_back( valid );

            if(!valid)
            {
                ftrs[j] = NAN;
            }

#if DRISHTI_CPR_DO_FTR_DEBUG
            m1 = m2 = 255;
            tmp1.ptr()[inds[i+0]] = 255 * int(m1 && m2);
            tmp1.ptr()[inds[i+1]] = 255 * int(m1 && m2);
#endif
        }
#endif
    }

#if DRISHTI_CPR_DO_FTR_DEBUG
    cv::imshow("tmp1", tmp1); // opt
    cv::imshow("M", M); // opt
    cv::imshow("I", I); // opt
    cv::waitKey(0); // opt
#endif

    // Create V for visualization:
    // TODO

    return 0;
}

//function ftrData = ftrsGen( model, varargin )
//% Generate random pose indexed features.
//%
//% USAGE
//%  ftrData = ftrsGen( model, varargin )
//%
//% INPUTS
//%  model    - pose model (see createModel())
//%  varargin - additional params (struct or name/value pairs)
//%   .type     - [2] feature type (1 or 2)
//%   .F        - [100] number of ftrs to generate
//%   .radius   - [2] sample initial x from circle of given radius
//%   .nChn     - [1] number of image channels (e.g. 3 for color images)
//%   .pids     - [] part ids for each x
//%
//% OUTPUTS
//%  ftrData  - struct containing generated ftrs
//%   .type     - feature type (1 or 2)
//%   .F        - total number of features
//%   .nChn     - number of image channels
//%   .xs       - feature locations relative to unit circle
//%   .pids     - part ids for each x
//%
//% EXAMPLE
//%
//% See also poseGt, poseGt>ftrsComp
// dfs={'type',2,'F',100,'radius',2,'nChn',1,'pids',[]};
// [type,F,radius,nChn,pids]=getPrmDflt(varargin,dfs,1);
// F1=F*type;
// F2=max(100,ceil(F1*1.5));
// xs=[];
// m=length(model.parts);
// while(size(xs,1)<F1),
//   xs=rand(F2,2)*2-1;
//   xs=xs(sum(xs.^2,2)<=1,:);
// end
// xs=xs(1:F1,:)*radius/2;
// if(nChn>1),
//   xs(:,3)=randint2(F1,1,[1 nChn]);
// end
// if(isempty(pids)),
//   pids=floor(linspace(0,F1,m+1));
// end
// ftrData=struct('type',type,'F',F,'nChn',nChn,'xs',xs,'pids',pids);
// end

int ftrsGen(const CPR::Model &model, const CPR::CprPrm::FtrPrm &ftrPrmIn, FtrData &ftrData, float lambda)
{
    CPR::CprPrm::FtrPrm dfs, ftrPrm;
    dfs.type = {"type", 2};
    dfs.F = {"F", 100};
    dfs.radius = {"radius", 2};
    dfs.nChn = {"nChn", 1};

    ftrPrm = ftrPrmIn;
    ftrPrm.merge(dfs, 1);

    const auto &type = *(ftrPrm.type);
    const auto &F = *(ftrPrm.F);
    const auto &radius = *(ftrPrm.radius);
    const auto &nChn = *(ftrPrm.nChn);

    int F1 = F * type;
    //std::cout << *(ftrData.xs) << std::endl;

    // currently only one model part:
    ftrData.xs = { "xs",  PointVec() };

#if DRISHTI_CPR_USE_FEATURE_SEPARATION_PRIOR
    // Generate a bunch of pixels:
    std::vector<cv::Point2f> points;
    cv::RNG rng;
    while(points.size() < (F1*4))
    {
        rng.state = rand();
        cv::Point2f p( rng.uniform(-1.0, +1.0), rng.uniform(-1.0, +1.0) );
        if(cv::norm(p) < 1.0)
        {
            points.push_back(p);
        }
    }

    // Now sample pixel pairs:
    float accept_prob = 0.0;
    float scale = radius / 2.0;

    cv::Mat1b mask(points.size(), points.size(), uint8_t(0));
    while(PointVecSize(*ftrData.xs) < F1)
    {
        int idx1, idx2;
        do
        {
            idx1 = rng.uniform(0, int(points.size()));
            idx2 = rng.uniform(0, int(points.size()));
            if(idx1 > idx2)
            {
                std::swap(idx1,idx2);
            }

            const double dist = cv::norm(points[idx1] - points[idx2]);
            accept_prob = std::exp(-dist/lambda);
        }
        while((idx1 == idx2) || !(accept_prob > rng.uniform(0.0, 1.0)) || mask(idx1, idx2));

        ftrData.xs->push_back( points[idx1] * scale );
        ftrData.xs->push_back( points[idx2] * scale );

        mask(idx1, idx2) = 1;

        //pairs.emplace_back( idx1, idx2 );
    }

#else
    int F2 = std::max(100.0, std::ceil(double(F1) * 1.5));

    // Uniform features
    while(PointVecSize(*ftrData.xs) < F1)
    {
        cv::Mat xs(F2, 2, CV_REAL_TYPE);
        cv::RNG().fill(xs, cv::RNG::UNIFORM, -1.0, +1.0);
        for(int i = 0; (i < F2) && (PointVecSize(*ftrData.xs) < F1); i++)
        {
            cv::Point_<RealType>  p = xs.at<cv::Point_<RealType>>(i, 0);
            if(cv::norm(p) <= 1.0)
            {
                ftrData.xs->push_back(p * RealType(radius / RealType(2.0)));
            }
        }
    }
#endif

    //std::cout << *(ftrData.xs) << std::endl;

    CV_Assert(nChn == 1); // for now just one channel
    ftrData.type = {"type", type};
    ftrData.F = {"F", F};
    ftrData.nChn = {"nChn", nChn};
    if(ftrData.pids->empty())
    {
        ftrData.pids = {"pids", {0.0, RealType(F1)}};
    }

    return 0;
}

static Matx33Real getPose( const Vector1d &phi  )
{
    // Currently only 1 part is supported in parametric mode:
    double x, y, ang, scl, asp, c, s, t;
    x = phi[0];
    y = phi[1];
    ang = phi[2];
    scl = phi[3];
    asp = phi[4];

    t = std::pow(2.0, scl);
    s = std::sin(-ang) * t;
    c = std::cos(-ang) * t;
    t = std::pow(2.0, asp);
    Matx33Real HS(c, -s*t, x, s, c*t, y, 0, 0, 1);

    return HS;
}

// Note: Base 0 with transposed image (column major)
static int index(RealType x, RealType y, int w, int h, int stride)
{
#if DRISHTI_CPR_TRANSPOSE
    RealType cs = std::max(RealType(1.0), std::min(RealType(h), RealType(x))); // note: tranpose
    RealType rs = std::max(RealType(1.0), std::min(RealType(w), RealType(y)));
    return (int(cs-1.0+0.5) * w + int(rs+0.5)) - 1; // base zero
#else
    return std::min(int(y+0.5f), h-1) * stride + std::min(int(x+0.5f), w-1);
#endif
}

static std::vector<uint32_t> xsToInds(const Matx33Real &HS, const PointVec &xs, int w, int h, int nChn, bool doTranspose, int stride)
{
    CV_Assert(nChn == 1);

#if DRISHTI_CPR_DO_LEAN
    int n = xs.size();
#else
    int n = xs.rows;
#endif

    std::vector<uint32_t> inds(n);
    for(int i = 0; i < inds.size(); i++)
    {
#if DRISHTI_CPR_DO_LEAN
        cv::Vec<RealType,2> p = xs[i];
#else
        cv::Vec<RealType,2> p = xs.at<cv::Vec<RealType,2>>(i,0);
#endif
        cv::Point3_<RealType> q = HS * cv::Point3_<RealType>(p[0], p[1], RealType(1.0) );
        inds[i] = index(q.x, q.y, w, h, stride);
    }
    return inds;
}

Vector1d identity(const CPR::Model &model)
{
    return Vector1d(5, 0.0);
}

Vector1d compose(const CPR::Model &mnodel, const Vector1d &phis0, const Vector1d &phis1 )
{
    bool isNew = true;
    Vector1d phis(phis0.size(),0);
    for(int i = 0; i < phis0.size(); i++)
    {
        phis[i] = phis0[i] + phis1[i];
        if(phis0[i] != RealType(0.0))
        {
            isNew = false;
        }
    }

    if(!isNew)
    {
        Matx33Real H0 = phisToHs(phis0);
        Matx33Real H1 = phisToHs(phis1);
        Matx33Real H = H0 * H1;
        Vector1d phis4 = phisFrHs(H);
        phis4.push_back(phis.back());
        std::swap(phis4, phis);
    }
    phis[2] = normAng(phis[2], DRISHTI_CPR_ANGLE_RANGE);

    return phis;
}

// function phis = inverse( model, phis0 ) %#ok<INUSL>
// % Compute inverse of phis0 so that phis0+phis1=phis1+phis0=identity.
// [N,R]=size(phis0);
// phis = -phis0;
// if( 1 && any(any(phis0(:,1:4)~=0)) )
//   H=phisToHs(phis0(:,1:4));
//   Hi=multiDiv(H,eye(3),1);
//   phis(:,1:4)=phisFrHs(Hi);
// end
// phis(:,3:3:R) = normAng(phis(:,3:3:R),2*pi);
// end

bool any(const Vector1d &phi)
{
    bool status = false;
    for(int i = 0; i < 4; i++)
    {
        if(phi[i] != 0.0)
        {
            status = true;
            break;
        }
    }
    return status;
}

// Compute inverse of phis0 so that phis0+phis1=phis1+phis0=identity.

// A\B is roughly the same as inv(A)*B

Vector1d inverse(const CPR::Model &mnodel, const Vector1d &phis0 )
{
    Vector1d phis = (phis0 * Vector1d::value_type(-1.0));
    if( any(phis0) )
    {
        Vector1d phis1 = phisFrHs(phisToHs(phis0).inv());
        for(int i = 0; i < 4; i++)
        {
            phis[i] = phis1[i];
        }
        phis[2] = normAng(phis[2], DRISHTI_CPR_ANGLE_RANGE); // update angle term
    }
    return phis;
}

Vector1d phisFrHs( const Matx33Real &Hs)
{
    double a, ss, sc, s, x, y;
    a = std::atan2(Hs(1,0), Hs(0,0));
    ss = Hs(0,0);
    sc = Hs(0,1);
    s = core::logN(std::sqrt(sc*sc+ss*ss),2.0);
    x = Hs(0,2);
    y = Hs(1,2);
    Vector1d phis { x, y, a, s };
    return phis;
}

Matx33Real phisToHs(const Vector1d &phis)
{
    double sc = std::pow(2.0, phis[3]);
    double c = std::cos(phis[2]) * sc;
    double s = std::sin(phis[2]) * sc;
    double x = phis[0];
    double y = phis[1];
    Matx33Real Hs(c, -s, x, s, c, y, 0, 0, 1);

    auto phis_ = phisFrHs(Hs);

    //print(phis); std::cout << std::endl;
    //print(phis_); std::cout << std::endl;

    return Hs;
}

//function phi = compPhiStar( model, phis ) %#ok<INUSL>
//  % Compute phi that minimizes sum of distances to phis.
//  phi=mean(phis,1);
//  [N,R]=size(phis);
//  M=1000;
//  as=linspace(0,2*pi,M);
//  if( 1 ) % numerical solution
//    for r=3:3:R,
//      del=normAng(phis(:,r*ones(1,M))-as(ones(1,N),:),2*pi);
//      [v,ind]=min(sum(del.^2,1));
//      phi(r)=as(ind);
//    end
//  else % closed form approximate solution
//    is=3:3:R;
//    s=mean(sin(phis(:,is)),1);
//    c=mean(cos(phis(:,is)),1);
//    phi(is)=mod(atan2(s,c),2*pi);
//  end
//end

Vector1d compPhiStar(const CPR::Model &model, const EllipseVec &phis)
{
    Vector1d phi(phis.front().size(), 0);
    for(int i = 0; i < phis.size(); i++)
    {
        phi += phis[i];
    }

    phi *= (Vector1d::value_type(1.0) / phis.size());

    // Now we search over all rotations:
    int M = 1000;
    std::pair<double, double> best(0.0, std::numeric_limits<double>::max());
    for(int j = 0; j < M; j++)
    {
        double t = (2.0 * M_PI) * double(j) / (M - 1), del = 0.0;
        for(int i = 0; i < phis.size(); i++)
        {
            del += std::pow(normAng(phis[i][2] - t, DRISHTI_CPR_ANGLE_RANGE), 2.0);
        }

        if(del < best.second)
        {
            best = {t, del};
        }
    }

    phi[2] = best.first; // update rotation

    return phi;
}

double normAng(double ang, double rng)
{
#if 1
    return ang;
#else
    ang = (ang/rng)+1000.0;
    ang = (ang - std::floor(ang)) * rng;
    return (ang > (rng/2.0)) ? (ang - rng) : ang;
#endif
}

void print(const Vector1d &p, bool eol)
{
    std::cout << "{" << p[0] << ' ' << p[1] << ' ' << p[2] << ' ' << p[3] << ' ' << p[4] << "}";
    if(eol)
    {
        std::cout << std::endl;
    }
}

//function [ds,dsAll] = dist( model, phis0, phis1 )
// % Compute distance between phis0(i,:,t) and phis1(i,:) for each i and t.
// [N,R,T]=size(phis0);
// wts=[model.parts.wts];
// del=diff(phis0,phis1);
// dsAll = del .* wts(ones(N,1),:,ones(T,1));
// dsAll = dsAll.^2;
// ds=sum(dsAll,2)/R;
//end

Vector1d diff( const Vector1d &phis0, const Vector1d &phis1 )
{
    Vector1d del(phis0.size());
    for(int i = 0; i < del.size(); i++)
    {
        del[i] = phis0[i] - phis1[i];
    }

    del[2] = normAng(del[2], DRISHTI_CPR_ANGLE_RANGE);
    return del;
}

double dist( const CPR::Model &model, const Vector1d &phis0, const Vector1d &phis1 )
{
    double ds = 0.0;
    Vector1d del = diff(phis0, phis1);
    for(int i = 0; i < del.size(); i++)
    {
        ds += std::pow(del[i]*(*model.parts->wts)[i], 2.0);
    }

    return ds / double(del.size());
}

Vector1d ellipseToPhi(const cv::RotatedRect &e)
{
    float cx = e.center.x;
    float cy = e.center.y;
    float angle = (e.angle) * M_PI/180.0; // NOTE: -e.angle
    float scale = core::logN( e.size.width, 2.0f );
    float aspectRatio = core::logN( e.size.height / e.size.width, 2.0f );
    return Vector1d { cx, cy, angle, scale, aspectRatio };
}

// Note: This is for ellipse drawn in transposed image
cv::RotatedRect phiToEllipse(const Vector1d &phi, bool transpose)
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

#if !DRISHTI_BUILD_MIN_SIZE
void drawFeatures(cv::Mat &canvas, const PointVec &xs, const Vector1d &phi, const std::vector<int> &features, float scale, bool doTranspose)
{
    // Draw the feature pairs
    if(scale != 1.f)
    {
        cv::resize(canvas, canvas, {}, scale, scale, cv::INTER_CUBIC);
    }

    Matx33Real Hs = getPose(phi);

    cv::RNG rng;
    rng.state = 100;

    for(const auto &i : features)
    {
        int k = (i * 2);
        cv::Scalar color(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));

#if DRISHTI_CPR_DO_LEAN
        const cv::Vec<RealType,2> p0 = xs[k+0];
        const cv::Vec<RealType,2> p1 = xs[k+1];
#else
        cv::Vec<RealType,2> p0 = xs.at<cv::Vec<RealType,2>>(k+0,0);
        cv::Vec<RealType,2> p1 = xs.at<cv::Vec<RealType,2>>(k+1,0);
#endif
        cv::Point3_<RealType> q0 = (Hs * cv::Point3_<RealType>(p0[0], p0[1], 1.0 )) * RealType(scale);
        cv::Point3_<RealType> q1 = (Hs * cv::Point3_<RealType>(p1[0], p1[1], 1.0 )) * RealType(scale);
        cv::Point f0(q0.x, q0.y);
        cv::Point f1(q1.x, q1.y);

        if(doTranspose)
        {
            std::swap(f0.x, f0.y);
            std::swap(f1.x, f1.y);
        }

        cv::circle(canvas, f0, 1, color, -1, 8);
        cv::circle(canvas, f1, 1, color, -1, 8);
        cv::line(canvas, f0, f1, color, 1, 8);
    }
};
#endif // !DRISHTI_BUILD_MIN_SIZE

DRISHTI_RCPR_END
