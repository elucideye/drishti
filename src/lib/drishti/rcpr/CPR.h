/*!
  @file   CPR.h
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @author P. Dollár (original matlab code (random ferns))
  @brief  Declaration of Cascaded Pose Regression class (single ellipse model).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishtisdk__CPR__
#define __drishtisdk__CPR__

#define DRISHTI_CPR_DO_LEAN 1
#define DRISHTI_CPR_DO_HALF_FLOAT 1
#define DRISHTI_CPR_ANGLE_RANGE (2.0 * M_PI)
#define DRISHTI_CPR_TRANSPOSE 0

#if defined(ANDROID)
#  define HALF_ENABLE_CPP11_CMATH 0
#endif
#include "half/half.hpp"
#include "drishti/rcpr/drishti_rcpr.h"
#include "drishti/ml/ShapeEstimator.h"
#include "drishti/acf/ACFField.h"
#include "drishti/ml/XGBooster.h"
#include "drishti/core/cvmat_serialization.h"
#include "drishti/core/serialization.h"
#include "drishti/core/Logger.h"

#include <boost/serialization/export.hpp>

#include <memory>

#define PTR_TYPE std

DRISHTI_RCPR_BEGIN

#if DRISHTI_CPR_DO_HALF_FLOAT
struct PointHalf
{
    PointHalf() {}

    PointHalf( const cv::Point2f &p )
        : x( half_float::detail::float2half<std::round_to_nearest>(p.x) )
        , y( half_float::detail::float2half<std::round_to_nearest>(p.y) )
    {}

    operator cv::Point2f() const
    {
        return cv::Point2f(asFloat(x), asFloat(y));
    }

    static float asFloat(const half_float::detail::uint16 &src)
    {
        return half_float::detail::half2float(src);
    }

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & x;
        ar & y;
    }

    half_float::detail::uint16 x, y;
};
template <typename T1, typename T2> void copy(std::vector<T1> &src, std::vector<T2> &dst)
{
    dst.resize(src.size());
    std::copy(src.begin(), src.end(), dst.begin());
}
#endif


struct ImageMaskPair
{
public:

    ImageMaskPair() {}
    ImageMaskPair(const cv::Mat &image) : image(image) {}
    ImageMaskPair(const cv::Mat &image, const cv::Mat &mask) : image(image), mask(mask) {}

    const cv::Mat &getImage() const
    {
        return image;
    }
    cv::Mat &getImage()
    {
        return image;
    }
    const cv::Mat &getMask() const
    {
        return mask;
    }
    cv::Mat &getMask()
    {
        return mask;
    }

    operator cv::Mat()
    {
        return image;  // legacy ImageVec compatibility
    }

protected:

    cv::Mat image;
    cv::Mat mask;
};

#if DRISHTI_CPR_DO_LEAN
#define CV_REAL_TYPE CV_32F
typedef float RealType;
typedef std::vector<cv::Point2f> PointVec;
inline int PointVecSize(const PointVec &v)
{
    return int(v.size());
}
#else
#define CV_REAL_TYPE CV_64F
typedef double RealType;
typedef cv::Mat PointVec;
inline int PointVecSize(const PointVec &v)
{
    return v.rows;
}
#endif
typedef cv::Matx<RealType,3,3> Matx33Real;
typedef std::vector<RealType> Vector1d;

typedef std::vector<cv::Mat> ImageVec;
typedef std::vector<ImageMaskPair> ImageMaskPairVec;
typedef std::vector<Vector1d> EllipseVec;
typedef std::vector<cv::Matx33f> HVec;

class CPR : public drishti::ml::ShapeEstimator
{
public:
    
    virtual std::vector<cv::Point2f> getMeanShape() const;

    cv::RotatedRect getPStar() const ; // get mean normalized ellipse

    struct Ellipse
    {
        double xs, ys, ang, scl, asp;
    };
    
    virtual void setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
    {
        drishti::ml::ShapeEstimator::setStreamLogger(logger);
        for(auto &reg : (*regModel->regs))
        {
            for(auto &t : reg->xgbdt)
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

    virtual void setStagesRepetitionFactor(int x)
    {
        stagesRepetitionFactor = x;
    }
    virtual int getStagesRepetitionFactor() const
    {
        return stagesRepetitionFactor;
    }

    struct Model // Currently used in both CprPrm and RegModel ???
    {
        struct Parts
        {
            acf::Field<RealType> prn;
            acf::Field<Vector1d> lks;
            acf::Field<RealType> joint; // NA
            acf::Field<Vector1d> mus;
            acf::Field<Vector1d> sigs;
            acf::Field<Vector1d> wts;

            // Boost serialization:
            friend class boost::serialization::access;
            template<class Archive>  void serialize(Archive & ar, const unsigned int version);
        };
        acf::Field<Parts> parts;

        // Boost serialization:
        friend class boost::serialization::access;
        template<class Archive>  void serialize(Archive & ar, const unsigned int version);
    };

    struct CprPrm
    {
        acf::Field<Model> model;
        acf::Field<RealType> T;
        acf::Field<RealType> L;

        struct FtrPrm
        {
            acf::Field<RealType> type;
            acf::Field<RealType> F;
            acf::Field<RealType> radius;
            acf::Field<int> nChn;

            void merge(const FtrPrm &opts, int checkExtra);

            // Boost serialization:
            friend class boost::serialization::access;
            template<class Archive>  void serialize(Archive & ar, const unsigned int version);
        };
        acf::Field<FtrPrm> ftrPrm;

        struct FernPrm
        {
            acf::Field<Vector1d> thrr;
            acf::Field<RealType> reg;
            acf::Field<RealType> S;
            acf::Field<RealType> M;
            acf::Field<RealType> R;
            acf::Field<RealType> eta;

            // Boost serialization:
            friend class boost::serialization::access;
            template<class Archive> void serialize(Archive & ar, const unsigned int version);
        };
        acf::Field<FernPrm> fernPrm;

        acf::Field<RealType> verbose;

        // ##### Experimental ######
        struct Recipe
        {
            int maxLeafNodes = 5;
            int maxDepth = 4;
            int treesPerLevel = 500;
            int featurePoolSize = 400;
            int featureSampleSize = 40; // NEW
            double learningRate = 0.1; // NEW
            double dataSampleRatio = 0.5; // NEW
            bool doMask = false; // NEW

            double featureRadius = 1.66;
            double lambda = RealType(0.1); // experimental

            bool useNPD = false;          // NEW

            std::vector<int> paramIndex;

            friend class boost::serialization::access;
            template<class Archive> void serialize(Archive & ar, const unsigned int version);

            void print(std::ostream &os)
            {
                os << "maxLeafNodes: " << maxLeafNodes << std::endl;
                os << "maxDepth: " << maxDepth << std::endl;
                os << "treesPerLevel: " << treesPerLevel << std::endl;
                os << "featurePoolSize: " << featurePoolSize << std::endl;
                os << "featureSampleSize: " << featureSampleSize << std::endl;
                os << "learningRate: " << learningRate << std::endl;
                os << "dataSampleRatio: " << dataSampleRatio << std::endl;
                os << "featureRadius: " << featureRadius << std::endl;
                os << "lambda: " << lambda << std::endl;
                os << "useNPD: " << useNPD << std::endl;
                os << "doMask: " << doMask << std::endl;
                os << "paramIndex: {";
                for(const auto &i : paramIndex)
                {
                    os << i;
                }
                os << "}" << std::endl;
            }
        };
        std::vector<Recipe> cascadeRecipes;

        // Boost serialization:
        friend class boost::serialization::access;
        template<class Archive> void serialize(Archive & ar, const unsigned int version);
    };
    acf::Field<CprPrm> cprPrm;

    struct RegModel
    {
        acf::Field<Model> model;
        acf::Field<Vector1d> pStar;
        acf::Field<cv::Mat> pDstr;
        acf::Field<RealType> T;
        acf::Field<Vector1d> pStar_; // CPR Verison 2

        struct Regs
        {
#if !DRISHTI_CPR_DO_LEAN
            struct Ferns
            {
                acf::Field<std::vector<uint32_t>> fids;
                acf::Field<Vector1d> thrs;
                acf::Field<Vector1d> ysFern;

                friend class boost::serialization::access;
                template<class Archive> void serialize(Archive & ar, const unsigned int version)
                {
                    ar & fids;
                    ar & thrs;
                    ar & ysFern;
                }
            };
            acf::Field< Ferns > ferns;
#endif

            struct FtrData
            {
                acf::Field<RealType> type;      // feature type (1 or 2) => always 2
                acf::Field<RealType> F;         // number of features to generate
                acf::Field<RealType> nChn;      // number of image channels
                acf::Field<PointVec> xs; // feature locations relative to unit circle (Fx2)
                acf::Field<Vector1d > pids;   // part ids for each x (just one part implemented)

                friend class boost::serialization::access;
                template<class Archive> void serialize(Archive & ar, const unsigned int version);
            };
            acf::Field< FtrData > ftrData;

            acf::Field<RealType> r;

            std::vector<std::pair<int, std::shared_ptr<ml::XGBooster>>> xgbdt;

            // Boost serialization:
            friend class boost::serialization::access;
            template<class Archive> void serialize(Archive & ar, const unsigned int version);
        };
        acf::Field< std::vector< acf::Field<Regs> > > regs;

        // Boost serialization:
        friend class boost::serialization::access;
        template<class Archive> void serialize(Archive & ar, const unsigned int version);

    };
    acf::Field<RegModel> regModel;

    CPR();
    CPR(const CPR &src);

#if !DRISHTI_CPR_DO_LEAN
    CPR(const std::string &filename);
    CPR(const char *filename);
#endif

    bool usesMask() const;

    struct CPROpts
    {
        acf::Field<Vector1d> pInit; // initial pose
        acf::Field< int > K; // number of initial pose restarts
        acf::Field< double > rad; // radius of Gassian Parzen window for finding mode

        void merge(const CPROpts &opts, int checkExtra);
    };

    struct CPRResult
    {
        Vector1d p;
        std::vector< Vector1d > pAll;
    };

    virtual int operator()(const cv::Mat &I, const cv::Mat &M, PointVec &points, std::vector<bool> &mask) const;
    virtual int operator()(const cv::Mat &I, PointVec &points, std::vector<bool> &mask) const;

    struct FeaturesResult
    {
        Vector1d ftrs;
        std::vector<uint8_t> ftrMask;
    };

    struct FernResult
    {
        double ys; // predicted outputs
        Vector1d ysCum; // predicted outputs after each stage
    };

    int cprTrain(const ImageMaskPairVec &images, const EllipseVec &ellipses, const HVec &H, const CprPrm &cprPrm, bool doJitter=false);
    int cprApplyTree(const cv::Mat &Is, const RegModel &regModel, const Vector1d &p, CPRResult &result, bool preview=false) const;
    int cprApplyTree(const ImageMaskPair &Is, const RegModel &regModel, const Vector1d &p, CPRResult &result, bool preview=false) const;

    void log(std::ofstream &os);

    virtual void setDoPreview(bool flag);

    void setWindowName(const std::string &name)
    {
        m_windowName = name;
    }

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
    static int cprApply1(const cv::Mat &Is, const RegModel &regModel, const Vector1d &p, CPRResult &result);
    int cprApply( const cv::Mat &Is, const RegModel &regModel, CPRResult &result, const CPROpts & = {}) const;
    static int fernsRegApply(const Vector1d &ftrs, const RegModel::Regs::Ferns &ferns, FernResult &result, const std::vector<uint32_t> &indsIn);
#endif

#if !DRISHTI_CPR_DO_LEAN
    // CVMATIO deserialization:
    int deserialize(const std::string &filename);
    int deserialize(const char *filename);
#endif

    // Boost serialization:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version);

    int stagesHint = std::numeric_limits<int>::max();
    int stagesRepetitionFactor = 1;
};

// Alias:
using FtrData=CPR::RegModel::Regs::FtrData;

int createModel(int type, CPR::Model &model);
int featuresComp(const CPR::Model &model, const Vector1d &p, const ImageMaskPair &I, const FtrData& ftrData, CPR::FeaturesResult &result, bool useNPD=false);
int ftrsGen(const CPR::Model &model, const CPR::CprPrm::FtrPrm &ftrPrmIn, FtrData &ftrData, float lambda=0.1f);
Vector1d identity(const CPR::Model &model);
Vector1d compose(const CPR::Model &mnodel, const Vector1d &phis0, const Vector1d &phis1 );
Vector1d inverse(const CPR::Model &model, const Vector1d &phis0);
Vector1d phisFrHs(const Matx33Real &Hs);
Vector1d compPhiStar(const CPR::Model &mnodel, const EllipseVec &phis);
Vector1d ellipseToPhi(const cv::RotatedRect &e);
cv::RotatedRect phiToEllipse(const Vector1d &phi, bool transpose=DRISHTI_CPR_TRANSPOSE);
Matx33Real phisToHs(const Vector1d &phis);
double normAng(double ang, double rng);
double dist( const CPR::Model &model, const Vector1d &phis0, const Vector1d &phis1 );
void print(const Vector1d &p, bool eol=false);
void drawFeatures(cv::Mat &canvas, const PointVec &xs, const Vector1d &phi, const std::vector<int> &features, float scale=1.f, bool doTranspose=DRISHTI_CPR_TRANSPOSE);

DRISHTI_RCPR_END

BOOST_CLASS_EXPORT_KEY(drishti::rcpr::CPR);
BOOST_CLASS_VERSION(drishti::rcpr::CPR::RegModel, 1);

#endif /* defined(__drishtisdk__CPR__) */
