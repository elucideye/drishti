/*! -*-c++-*-
  @file   CPR.h
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @author P. Dollár (original matlab code (random ferns))
  @brief  Declaration of Cascaded Pose Regression class (single ellipse model).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_rcpr_CPR_h__
#define __drishti_rcpr_CPR_h__

#define DRISHTI_CPR_DO_LEAN 1
#define DRISHTI_CPR_DO_HALF_FLOAT 1
#define DRISHTI_CPR_ANGLE_RANGE (2.0 * M_PI)

// clang-format off
#if defined(ANDROID)
#  define HALF_ENABLE_CPP11_CMATH 0
#endif
// clang-format on
#include "half/half.hpp"

#include "drishti/rcpr/drishti_rcpr.h"
#include <acf/ACFField.h>

// clang-format off
#if DRISHTI_CPR_DO_HALF_FLOAT
#  include "drishti/rcpr/PointHalf.h"
#endif
// clang-format on

#include "drishti/core/Logger.h"
#include "drishti/core/Field.h"
#include "drishti/rcpr/ImageMaskPair.h"
#include "drishti/rcpr/Vector1d.h"
#include "drishti/rcpr/Recipe.h"
#include "drishti/ml/ShapeEstimator.h"
#include "drishti/ml/XGBooster.h"

#include <memory>

DRISHTI_RCPR_NAMESPACE_BEGIN

#define CV_REAL_TYPE CV_32F
typedef float RealType;
typedef std::vector<cv::Point2f> PointVec;
inline int PointVecSize(const PointVec& v)
{
    return int(v.size());
}

typedef cv::Matx<RealType, 3, 3> Matx33Real;
typedef std::vector<RealType> Vector1d;
typedef std::vector<cv::Mat> ImageVec;
typedef std::vector<int> IntVec;
typedef std::vector<ImageMaskPair> ImageMaskPairVec;
typedef std::vector<Vector1d> EllipseVec;
typedef std::vector<cv::Matx33f> HVec;

class CPR : public drishti::ml::ShapeEstimator
{
public:
    using ViewFunc = std::function<void(const std::string& name, const cv::Mat& image)>;

    CPR();
    CPR(const CPR& src);

#if !DRISHTI_CPR_DO_LEAN
    CPR(const std::string& filename);
    CPR(const char* filename);
#endif

    ~CPR();

    void setViewer(ViewFunc& viewer)
    {
        m_viewer = viewer;
    }

    virtual std::vector<cv::Point2f> getMeanShape() const;

    cv::RotatedRect getPStar() const; // get mean normalized ellipse

    struct Ellipse
    {
        double xs, ys, ang, scl, asp;
    };

    virtual void setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
    {
        drishti::ml::ShapeEstimator::setStreamLogger(logger);
        for (auto& reg : (*regModel->regs))
        {
            for (auto& t : reg->xgbdt)
            {
                t.second->setStreamLogger(logger);
            }
        }
    }

    virtual void setStagesHint(int stages)
    {
        stagesHint = stages;
    };

    virtual int getStagesHint() const
    {
        return stagesHint;
    };

    struct Model // Currently used in both CprPrm and RegModel ???
    {
        struct Parts
        {
            core::Field<RealType> prn;
            core::Field<Vector1d> lks;
            core::Field<RealType> joint; // NA
            core::Field<Vector1d> mus;
            core::Field<Vector1d> sigs;
            core::Field<Vector1d> wts;

            template <class Archive>
            void serialize(Archive& ar, const unsigned int version);
        };
        core::Field<Parts> parts;

        template <class Archive>
        void serialize(Archive& ar, const unsigned int version);
    };

    struct CprPrm
    {
        core::Field<Model> model;
        core::Field<RealType> T;
        core::Field<RealType> L;

        struct FtrPrm
        {
            core::Field<RealType> type;
            core::Field<RealType> F;
            core::Field<RealType> radius;
            core::Field<int> nChn;

            void merge(const FtrPrm& opts, int checkExtra);

            template <class Archive>
            void serialize(Archive& ar, const unsigned int version);
        };
        core::Field<FtrPrm> ftrPrm;

        struct FernPrm
        {
            core::Field<Vector1d> thrr;
            core::Field<RealType> reg;
            core::Field<RealType> S;
            core::Field<RealType> M;
            core::Field<RealType> R;
            core::Field<RealType> eta;

            template <class Archive>
            void serialize(Archive& ar, const unsigned int version);
        };
        core::Field<FernPrm> fernPrm;

        core::Field<RealType> verbose;

        typedef rcpr::Recipe Recipe;

        std::vector<Recipe> cascadeRecipes;

        template <class Archive>
        void serialize(Archive& ar, const unsigned int version);
    };
    core::Field<CprPrm> cprPrm;

    struct RegModel
    {
        core::Field<Model> model;
        core::Field<Vector1d> pStar;
        core::Field<cv::Mat> pDstr;
        core::Field<RealType> T;
        core::Field<Vector1d> pStar_; // CPR Verison 2

        struct Regs
        {
            struct FtrData
            {
                core::Field<RealType> type; // feature type (1 or 2) => always 2
                core::Field<RealType> F;    // number of features to generate
                core::Field<RealType> nChn; // number of image channels
                core::Field<PointVec> xs;   // feature locations relative to unit circle (Fx2)
                core::Field<Vector1d> pids; // part ids for each x (just one part implemented)

                template <class Archive>
                void serialize(Archive& ar, const unsigned int version);
            };
            core::Field<FtrData> ftrData;

            core::Field<RealType> r;

            // std::shared_ptr<> is preferred here w/ std::vector<> to avoid need for copyable ml::XGBooster
            std::vector<std::pair<int, std::shared_ptr<ml::XGBooster>>> xgbdt;

            template <class Archive>
            void serialize(Archive& ar, const unsigned int version);
        };
        core::Field<std::vector<core::Field<Regs>>> regs;

        template <class Archive>
        void serialize(Archive& ar, const unsigned int version);
    };
    core::Field<RegModel> regModel;

    bool usesMask() const;

    struct CPROpts
    {
        core::Field<Vector1d> pInit; // initial pose
        core::Field<int> K;          // number of initial pose restarts
        core::Field<double> rad;     // radius of Gassian Parzen window for finding mode

        void merge(const CPROpts& opts, int checkExtra);
    };

    struct CPRResult
    {
        Vector1d p;
        std::vector<Vector1d> pAll;
    };

    virtual int operator()(const cv::Mat& I, const cv::Mat& M, PointVec& points, std::vector<bool>& mask) const;
    virtual int operator()(const cv::Mat& I, PointVec& points, std::vector<bool>& mask) const;

    struct FeaturesResult
    {
        Vector1d ftrs;
        std::vector<uint8_t> ftrMask;
    };

    struct FernResult
    {
        double ys;      // predicted outputs
        Vector1d ysCum; // predicted outputs after each stage
    };

    int cprTrain(const ImageMaskPairVec& images, const EllipseVec& ellipses, const HVec& H, const CprPrm& cprPrm, bool doJitter = false);
    int cprApplyTree(const cv::Mat& Is, const RegModel& regModel, const Vector1d& p, CPRResult& result, bool preview = false) const;
    int cprApplyTree(const ImageMaskPair& Is, const RegModel& regModel, const Vector1d& p, CPRResult& result, bool preview = false) const;

    virtual void setDoPreview(bool flag);

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version);

protected:
    std::string m_windowName = "debug";

    bool m_doPreview = false;
    bool m_isMat = false;
    struct StageLog
    {
        int param = 0;
        double loss = 0.0;
    };
    std::vector<StageLog> trainingLog;

#if !DRISHTI_CPR_DO_LEAN
    static int cprApply1(const cv::Mat& Is, const RegModel& regModel, const Vector1d& p, CPRResult& result);
    int cprApply(const cv::Mat& Is, const RegModel& regModel, CPRResult& result, const CPROpts& = {}) const;
#endif

#if !DRISHTI_CPR_DO_LEAN
    // CVMATIO deserialization:
    int deserialize(const std::string& filename);
    int deserialize(const char* filename);
#endif

    int stagesHint = std::numeric_limits<int>::max();

    ViewFunc m_viewer;
};

// Alias:
using FtrData = CPR::RegModel::Regs::FtrData;

int createModel(int type, CPR::Model& model);
int featuresComp(const CPR::Model& model, const Vector1d& p, const ImageMaskPair& I, const FtrData& ftrData, CPR::FeaturesResult& result, bool useNPD = false);
int ftrsGen(const CPR::Model& model, const CPR::CprPrm::FtrPrm& ftrPrmIn, FtrData& ftrData, float lambda = 0.1f);
Vector1d identity(const CPR::Model& model);
Vector1d compose(const CPR::Model& mnodel, const Vector1d& phis0, const Vector1d& phis1);
Vector1d inverse(const CPR::Model& model, const Vector1d& phis0);
Vector1d phisFrHs(const Matx33Real& Hs);
Vector1d compPhiStar(const CPR::Model& mnodel, const EllipseVec& phis);
Vector1d ellipseToPhi(const cv::RotatedRect& e);
cv::RotatedRect phiToEllipse(const Vector1d& phi);
Matx33Real phisToHs(const Vector1d& phis);
double normAng(double ang, double rng);
double dist(const CPR::Model& model, const Vector1d& phis0, const Vector1d& phis1);
void print(const Vector1d& p, bool eol = false);
void drawFeatures(cv::Mat& canvas, const PointVec& xs, const Vector1d& phi, const IntVec& features, float scale = 1.f);

template <typename T1, typename T2>
void copy(std::vector<T1>& src, std::vector<T2>& dst)
{
    dst.resize(src.size());
    std::copy(src.begin(), src.end(), dst.begin());
}

DRISHTI_RCPR_NAMESPACE_END

#include "drishti/core/drishti_stdlib_string.h" // FIRST
#include <cereal/types/polymorphic.hpp>
CEREAL_REGISTER_TYPE(drishti::rcpr::CPR);
CEREAL_REGISTER_POLYMORPHIC_RELATION(drishti::ml::ShapeEstimator, drishti::rcpr::CPR);
//CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(drishti::rcpr::CPR, cereal::specialization::member_serialize);

#endif /* DRISHTI_RCPR_CRP_H */
