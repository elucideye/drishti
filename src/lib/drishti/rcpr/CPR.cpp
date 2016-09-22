/*!
  @file   CPR.cpp
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @author P. Dollár (original matlab code (random ferns))
  @brief  Declaration of Cascaded Pose Regression class (single ellipse model).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/drishti_core.h"
#include "drishti/rcpr/CPR.h"
#include "drishti/acf/ACFField.h"

// Deprecated backwards compatibility
#if !DRISHTI_CPR_DO_LEAN
#  include "cvmatio/MatlabIO.hpp"
#  include "cvmatio/MatlabIOContainer.hpp"
#  include "drishti/acf/ACFIO.h"
#endif

#include "drishti/core/serialization.h"

DRISHTI_RCPR_BEGIN

bool CPR::usesMask() const
{
    bool flag = false;
    for(const auto &r : cprPrm->cascadeRecipes)
    {
        if(r.doMask)
        {
            flag = true;
            break;
        }
    }
    return flag;
}

void CPR::setDoPreview(bool flag)
{
    m_doPreview = flag;
}

std::vector<cv::Point2f> CPR::getMeanShape() const
{
    Vector1d mu = regModel->pStar;
    cv::RotatedRect e = phiToEllipse(mu);
    return std::vector<cv::Point2f> {{e.center.x,0},{e.center.y,0},{e.size.width,0},{e.size.height},{e.angle,0}};
}

cv::RotatedRect CPR::getPStar() const
{
    Vector1d mu = regModel->pStar_;
    return phiToEllipse(mu);
}

void CPR::CprPrm::FtrPrm::merge(const CPR::CprPrm::FtrPrm &opts, int checkExtra)
{
    type.merge(opts.type, checkExtra);
    F.merge(opts.F, checkExtra);
    radius.merge(opts.radius, checkExtra);
    nChn.merge(opts.nChn, checkExtra);
}

CPR::CPR() {}
CPR::CPR(const CPR &src) {}

#if !DRISHTI_CPR_DO_LEAN
CPR::CPR(const std::string &filename)
{
    deserialize(filename);
}

CPR::CPR(const char * filename)
{
    deserialize(filename);
}
#endif

// TODO: rethink shape estimator API to better support parametric models
// For now we'll just store each parameter in the x coordinate of a 2D point

static cv::RotatedRect pointsToEllipse(const std::vector<cv::Point2f> &points)
{
    return cv::RotatedRect({points[0].x,points[1].x}, {points[2].x, points[3].x}, points[4].x);
}

static Vector1d pointsToPhi(const std::vector<cv::Point2f> &points)
{
    return ellipseToPhi(pointsToEllipse(points));
}

int CPR::operator()(const cv::Mat &I, const cv::Mat &M, Point2fVec &points, BoolVec &mask) const
{
    DRISHTI_STREAM_LOG_FUNC(8,2,m_streamLogger);

    CPRResult result;

    if(m_isMat)
    {
#if DRISHTI_CPR_DO_LEAN
        std::cerr << "Software configured without random fern regressors: see DRISHTI_CPR_DO_LEAN" << std::endl;
        CV_Assert(m_isMat == false);
#else
        CPROpts  opts;
        if(points.size() == 1)
        {
            opts.pInit = { "pInit", { RealType(points[0].x), RealType(points[0].y)} };
        }

        cprApply(I, (*regModel), result, opts);
#endif
    }
    else
    {
        ImageMaskPair Is { I, M };
        Vector1d pStar = (points.size() == 5) ? pointsToPhi(points) : (*regModel->pStar);
        cprApplyTree(Is, *regModel, pStar, result, m_doPreview);
    }

    if(result.p.size() == 5)
    {
        points.resize(5);
        cv::RotatedRect ellipse = phiToEllipse(result.p);
        points =
        {
            {ellipse.center.x, 0.f}, // tranpose center
            {ellipse.center.y, 0.f},
            {ellipse.size.width, 0.f}, // flip width and height
            {ellipse.size.height, 0.f},
            {ellipse.angle, 0.f}
        };

        if(DRISHTI_CPR_TRANSPOSE)
        {
            float theta = ellipse.angle * M_PI/180.0;
            std::swap(points[0].x, points[1].x);
            points[4].x = atan2(std::cos(theta),std::sin(theta)) * 180.0/M_PI;
        }
    }

    return 0;

}

int CPR::operator()(const cv::Mat &I, std::vector<cv::Point2f> &points, std::vector<bool> &mask) const
{
    DRISHTI_STREAM_LOG_FUNC(8,3,m_streamLogger);
    return (*this)(I, {}, points, mask);
}

// Move to CPRIO

// Boost serialization:
template<class Archive> void CPR::Model::Parts::serialize(Archive & ar, const unsigned int version)
{
    ar & prn;
    ar & lks;
    ar & joint;
    ar & mus;
    ar & sigs;
    ar & wts;
}

template<class Archive> void CPR::Model::serialize(Archive & ar, const unsigned int version)
{
    ar & parts;
}

template<class Archive> void CPR::CprPrm::FtrPrm::serialize(Archive & ar, const unsigned int version)
{
    ar & type;
    ar & F;
    ar & radius;
    ar & nChn;
}

template<class Archive> void CPR::CprPrm::FernPrm::serialize(Archive & ar, const unsigned int version)
{
    ar & thrr;
    ar & reg;
    ar & S;
    ar & M;
    ar & R;
    ar & eta;
}

template<class Archive> void CPR::CprPrm::Recipe::serialize(Archive & ar, const unsigned int version)
{
    ar & maxLeafNodes;
    ar & maxDepth;
    ar & treesPerLevel;
    ar & featurePoolSize;
    ar & featureSampleSize;
    ar & featureRadius;
    ar & learningRate;
    ar & dataSampleRatio;
    ar & useNPD;
    ar & doMask;
    ar & paramIndex;
    ar & lambda;
}

template<class Archive> void CPR::CprPrm::serialize(Archive & ar, const unsigned int version)
{
    ar & model;
    ar & T;
    ar & L;
    ar & ftrPrm;
#if !DRISHTI_CPR_DO_LEAN
    ar & fernPrm;
#endif
    ar & verbose;

    // New experimental features
    ar & cascadeRecipes;
}

template<class Archive> void CPR::RegModel::Regs::FtrData::serialize(Archive & ar, const unsigned int version)
{
    ar & type;
    ar & F;
    ar & nChn;

#if DRISHTI_CPR_DO_HALF_FLOAT
    std::vector<PointHalf> xs_;
    if(Archive::is_loading::value)
    {
        ar & xs_;
        copy(xs_, (*xs));
    }
    else
    {
        copy((*xs), xs_);
        ar & xs_;
    }
#else
    ar & xs;
#endif
    
    ar & pids;
}

template<class Archive> void CPR::RegModel::Regs::serialize(Archive & ar, const unsigned int version)
{
#if !DRISHTI_CPR_DO_LEAN
    ar & ferns;
#endif
    
    ar & ftrData;
    ar & r;
    ar & xgbdt;
}

template<class Archive> void CPR::RegModel::serialize(Archive & ar, const unsigned int version)
{
    ar & model;
    ar & pStar;
    ar & pDstr;
    ar & T;
    ar & regs;

    if(version >= 1)
    {
        ar & pStar_;
    }
}

template<class Archive> void CPR::serialize(Archive & ar, const unsigned int version)
{
    boost::serialization::void_cast_register<drishti::rcpr::CPR, drishti::ml::ShapeEstimator>();
    ar & cprPrm;
    ar & regModel;
}

// explicit instantiation:

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
template void CPR::Model::Parts::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void CPR::Model::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void CPR::CprPrm::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void CPR::RegModel::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void CPR::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
#endif

template void CPR::Model::Parts::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void CPR::Model::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void CPR::CprPrm::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void CPR::RegModel::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void CPR::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);

#if DRISHTI_USE_TEXT_ARCHIVES

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

template void CPR::Model::Parts::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
template void CPR::Model::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
template void CPR::CprPrm::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
template void CPR::RegModel::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
template void CPR::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);

template void CPR::Model::Parts::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
template void CPR::Model::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
template void CPR::CprPrm::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
template void CPR::RegModel::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
template void CPR::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);

#endif

DRISHTI_RCPR_END

BOOST_CLASS_EXPORT_IMPLEMENT(drishti::rcpr::CPR);
