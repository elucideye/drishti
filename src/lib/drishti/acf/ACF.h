/*!
  @file   ACF.h
  @author David Hirvonen (C++ implementation)
  @author P. Dollár (original matlab code)
  @brief  Aggregated Channel Feature object detector declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __DRISHTI__ACF__
#define __DRISHTI__ACF__

#include <stdio.h>

#include "drishti/core/drishti_defs.hpp"
#include "drishti/core/drishti_string_hash.h"

#include "drishti/ml/ObjectDetector.h"
#include "drishti/acf/drishti_acf.h"
#include "drishti/acf/ACFField.h"
#include "drishti/acf/MatP.h"
#include "drishti/core/IndentingOStreamBuffer.h"
#include "drishti/core/Logger.h"

#if DRISHTI_SERIALIZE_WITH_BOOST
# include "drishti/core/boost_serialize_common.h" // optional
#endif

#include <opencv2/imgproc/imgproc.hpp>

#include <boost/multi_array.hpp>

#include <cassert>
#include <iostream>
#include <functional>

DRISHTI_ACF_NAMESPACE_BEGIN

template<class _T> struct ParserNode;

class Detector : public drishti::ml::ObjectDetector
{
public:

    typedef ParserNode<Detector> ParserNodeDetector;
    typedef std::vector<cv::Size2d> Size2dVec;
    typedef std::vector<double> RealVec;
    typedef std::vector<cv::Rect> RectVec;

    typedef std::function<int(const cv::Mat&, const std::string &tag)> MatLoggerType;

    Detector() {}
    Detector(const Detector &src);
    Detector(std::istream &is);
    Detector(const std::string &filename);
    Detector(const char *filename);

    struct Options
    {
        //   .type       - ['max'] 'max', 'maxg', 'ms', 'cover', or 'none'
        //   .thr        - [-inf] threshold below which to discard (0 for 'ms')
        //   .maxn       - [inf] if n>maxn split and run recursively (see above)
        //   .radii      - [.15 .15 1 1] supression radii ('ms' only, see above)
        //   .overlap    - [.5] area of overlap for bbs
        //   .ovrDnm     - ['union'] area of overlap denominator ('union' or 'min')
        //   .resize     - {} parameters for bbApply('resize')
        //   .separate   - [0] run nms separately on each bb type (bbType)

        struct Nms
        {
            Field<std::string> type;
            Field<double> thr;
            Field<double> maxn;
            Field<std::vector<double>> radii;
            Field<double> overlap;
            Field<std::string> ovrDnm;
            // resize ???
            Field<int> separate;

            void merge(const Nms &src, int mode);
            friend std::ostream& operator<<(std::ostream &os, const Nms &src);
            
            // Boost serialization:
            template<class Archive> void serialize(Archive & ar, const uint32_t version);
        };

        struct Pyramid
        {
            struct Chns
            {
                Chns()
                {
                    shrink.set("shrink");
                    pColor.set("pColor");
                    pGradMag.set("pGradMag");
                    pGradHist.set("pGradHist");
                    pCustom.set("pCustom");
                    complete.set("complete");
                }

                Field<int> shrink;

                struct Color
                {
                    Field<int> enabled;
                    Field<double> smooth;
                    Field<std::string> colorSpace;

                    void merge(const Color &src, int mode);
                    friend std::ostream& operator<<(std::ostream &os, const Color &src);
                    
                    // Boost serialization:
                    template<class Archive> void serialize(Archive & ar, const uint32_t version);
                };
                Field<Color> pColor;

                struct GradMag
                {
                    Field<int> enabled;
                    Field<int> colorChn;
                    Field<int> normRad;
                    Field<double> normConst;
                    Field<int> full;

                    void merge(const GradMag &src, int mode);
                    friend std::ostream& operator<<(std::ostream &os, const GradMag &src);
                    
                    // Boost serialization:
                    template<class Archive> void serialize(Archive & ar, const uint32_t version);
                };
                Field<GradMag> pGradMag;

                struct GradHist
                {
                    Field<int> enabled;
                    Field<int> binSize;
                    Field<int> nOrients;
                    Field<int> softBin;
                    Field<int> useHog;
                    Field<double> clipHog;

                    void merge(const GradHist &src, int mode);
                    friend std::ostream& operator<<(std::ostream &os, const GradHist &src);
                    
                    // Boost serialization:
                    template<class Archive> void serialize(Archive & ar, const uint32_t version);

                };
                Field<GradHist> pGradHist;

                struct Custom
                {
                    // TODO:
                    void merge(const Custom &src, int mode);
                    friend std::ostream& operator<<(std::ostream &os, const Custom &src);
                    
                    // Boost serialization:
                    template<class Archive> void serialize(Archive & ar, const uint32_t version);
                };
                Field<Custom> pCustom;
                Field<int> complete;

                void merge(const Chns &src, int mode);
                friend std::ostream& operator<<(std::ostream &os, const Chns &src);
                
                // Boost serialization:
                template<class Archive> void serialize(Archive & ar, const uint32_t version);
            };

            Field<Chns> pChns;

            Field<int> nPerOct;
            Field<int> nOctUp;
            Field<int> nApprox;
            Field<std::vector<double>> lambdas;
            Field<cv::Size> pad;
            Field<cv::Size> minDs;

            Field<double> smooth;
            Field<int> concat;
            Field<int> complete;

            void merge(const Pyramid &src, int mode);
            friend std::ostream& operator<<(std::ostream &os, const Pyramid &src);
            
            // Boost serialization:
            template<class Archive> void serialize(Archive & ar, const uint32_t version);
        };

        Field<Pyramid> pPyramid;
        Field<cv::Size> modelDs;
        Field<cv::Size> modelDsPad;

        Field<Nms> pNms;
        Field<int> stride;
        Field<double> cascThr;
        Field<double> cascCal;
        Field<std::vector<int>> nWeak;

        struct Boost
        {
            struct Tree
            {
                Field<int> nBins;
                Field<int> maxDepth;
                Field<double> minWeight;
                Field<double> fracFtrs;
                Field<int> nThreads;

                void merge(const Tree &src, int mode);
                friend std::ostream& operator<<(std::ostream &os, const Tree &src);
                
                // Boost serialization:
                template<class Archive> void serialize(Archive & ar, const uint32_t version);
            };
            Field<Tree> pTree;
            Field<int> nWeak;
            Field<int> discrete;
            Field<int> verbose;

            void merge(const Boost &src, int mode);
            friend std::ostream& operator<<(std::ostream &os, const Boost &src);
            
            // Boost serialization:
            template<class Archive> void serialize(Archive & ar, const uint32_t version);
        };

        Field<Boost> pBoost;

        Field<double> seed;
        Field<std::string> name;
        Field<std::string> posGtDir;
        Field<std::string> posImgDir;
        Field<std::string> negImgDir;
        Field<std::string> posWinDir;
        Field<std::string> negWinDir;

        Field<int> nPos;
        Field<int> nNeg;
        Field<int> nPerNeg;
        Field<int> nAccNeg;

        struct Jitter
        {
            Field<int> flip;

            void merge(const Jitter &src, int mode);
            friend std::ostream& operator<<(std::ostream &os, const Jitter &src);

            // Boost serialization:
            template<class Archive> void serialize(Archive & ar, const uint32_t version);
        };
        Field<Jitter> pJitter;
        Field<int> winsSave;

        void merge(const Options &src, int mode);
        friend std::ostream& operator<<(std::ostream &os, const Options &src);
        
        // Boost serialization:
        template<class Archive> void serialize(Archive & ar, const uint32_t version);
    };

    Options opts;

    // see adaboostTrain()
    // OUTPUTS
    //  model      - learned boosted tree classifier w the following fields
    //   .fids       - [K x nWeak] feature ids for each node
    //   .thrs       - [K x nWeak] threshold corresponding to each fid
    //   .child      - [K x nWeak] index of child for each node (1-indexed)
    //   .hs         - [K x nWeak] log ratio (.5*log(p/(1-p)) at each node
    //   .weights    - [K x nWeak] total sample weight at each node
    //   .depth      - [K x nWeak] depth of each node
    //   .errs       - [1 x nWeak] error for each tree (for debugging)
    //   .losses     - [1 x nWeak] loss after every iteration (for debugging)
    //   .treeDepth  - depth of all leaf nodes (or 0 if leaf depth varies)

    struct Classifier
    {
        cv::Mat fids;                // uint32_t
        cv::Mat thrs;                // float
        cv::Mat child;               // uint32_t
        cv::Mat hs;                  // float
        cv::Mat weights;             // float
        cv::Mat depth;               // uint32_t

        std::vector<double> errs;
        std::vector<double> losses;
        int treeDepth;

        cv::Mat thrsU8;  // prescaled threshold (x255) for uint8_t input
        const cv::Mat & getScaledThresholds(int type);

        template<class Archive> void serialize(Archive & ar, const uint32_t version);
    };

    Classifier clf;

    // see chnsCompute()
    // OUTPUTS
    // chns       - output struct
    //  .pChns      - exact input parameters used
    //  .nTypes     - number of channel types
    //  .data       - [nTypes x 1] cell [h/shrink x w/shrink x nChns] channels
    //  .info       - [nTypes x 1] struct array
    //    .name       - channel type name
    //    .pChn       - exact input parameters for given channel type
    //    .nChns      - number of channels for given channel type
    //    .padWith    - how channel should be padded (0,'replicate')

    struct Channels
    {
        Options::Pyramid::Chns pChns;
        int nTypes = 0;
        std::vector<MatP> data;
        struct Info
        {
            std::string name;
            // Detector::Options::Pyramid::Chns pChn; /* TODO: C++ requires strong type */
            int nChns = 0;
            std::string padWith;
        };
        std::vector<Info> info;
    };

    static int chnsCompute(const MatP &I, const Options::Pyramid::Chns &pChns, Channels &chns, bool isInit=false, MatLoggerType pLogger={});

    // see chnsPyramid()
    // OUTPUTS
    //  pyramid      - output struct
    //   .pPyramid     - exact input parameters used (may change from input)
    //   .nTypes       - number of channel types
    //   .nScales      - number of scales computed
    //   .data         - [nScales x nTypes] cell array of computed channels
    //   .info         - [nTypes x 1] struct array (mirrored from chnsCompute)
    //   .lambdas      - [nTypes x 1] scaling coefficients actually used
    //   .scales       - [nScales x 1] relative scales (approximate)
    //   .scaleshw     - [nScales x 2] exact scales for resampling h and w

    struct Pyramid
    {
        typedef boost::multi_array<MatP, 2> array_type;
        typedef array_type::index index;

        Detector::Options::Pyramid pPyramid; // < exact input parameters
        int nTypes = 0;
        int nScales = 0;
        array_type data;
        std::vector< Channels::Info > info;
        std::vector< double > lambdas;
        std::vector< double > scales;
        std::vector< cv::Size2d > scaleshw;

        // .rois   - [ LEVELS x CHANNELS ] array for channel access
        std::vector<std::vector<cv::Rect>> rois;
    };

    // This contains the subset of parameters that are permitted to be overriden in acfModify
    struct Modify
    {
        Field<int> nPerOct;
        Field<int> nOctUp;
        Field<int> nApprox;
        Field<std::vector<double>> lambdas;
        Field<cv::Size> pad;
        Field<cv::Size> minDs;
        Field<Options::Nms> pNms;
        Field<int> stride;
        Field<double> cascThr;
        Field<double> cascCal;
        Field<double> rescale;

        friend std::ostream& operator<<(std::ostream &os, const Modify &src);
        void merge(const Modify &src, int mode);
    };
    
    cv::Size getWindowSize() const
    {
        return opts.modelDs.get();
    }

    // (((( Compute pyramid ))))
    void computePyramid(const cv::Mat &I, Pyramid &P);
    void computePyramid(const MatP &Ip, Pyramid &P);

    static void computeChannels(const cv::Mat &I, MatP &Ip2, MatLoggerType pLogger={});
    static void computeChannels(const MatP &Ip, MatP &Ip2, MatLoggerType pLlogger={});

    // (((((((( Detection ))))))))
    int operator()(const cv::Mat &I, RectVec &objects, RealVec *scores=0);
    int operator()(const MatP &I, RectVec &objects, RealVec *scores=0);

    // Multiscale search:
    int operator()(const Pyramid &P, RectVec &objects, RealVec *scores=0);

    int chnsPyramid(const MatP &I, const Options::Pyramid *pPyramid, Pyramid &pyramid, bool isInit=false, MatLoggerType pLogger={});

    static int rgbConvert(const MatP &I, MatP &J, const std::string &cs, bool useSingle, bool isLuv=false);
    static int getScales(int nPerOct, int nOctUp, const cv::Size &minDs, int shrink, const cv::Size &sz, RealVec &scales, Size2dVec &scaleshw);
    static int convTri(const MatP &I, MatP &J, double r=1.0, int s=1);
    static int gradientMag(const cv::Mat &I, cv::Mat &M, cv::Mat &O, int channel=0, int normRad=0, double normConst=0.005, int full=0, MatLoggerType logger={});
    static int gradientHist(const cv::Mat &M, const cv::Mat &O, MatP &H, int binSize, int nOrients, int softBin, int useHog, double clipHog, int full);

    virtual void setDetectionScorePruneRatio(double ratio)
    {
        m_detectionScorePruneRatio = ratio;
    }

    struct Detection
    {
        Detection() {}
        Detection(const cv::Rect &r, double s) : roi(r), score(s) {}
        operator cv::Rect()
        {
            return roi;
        }
        cv::Rect roi;
        double score = 1.0;
    };
    using DetectionVec = std::vector<Detection>;

    void acfDetect1(const MatP &chns, const RectVec &rois, int shrink, cv::Size modelDsPad, int stride, double cascThr, DetectionVec &objects);

    int bbNms(const DetectionVec &bbsIn, const Options::Nms &pNms, DetectionVec &bbs);

    int acfModify( const Detector::Modify &params );

    // (((((((( I/O ))))))))
    int initializeOpts();
    int deserialize(const std::string &filename);
    int deserialize(const char * filename);
    int deserialize(std::istream &is);
    int deserialize(ParserNodeDetector &detector_);
    
    // Boost serialization:
    template<class Archive> void serialize(Archive & ar, const uint32_t version);

    // Additional configuration parameters:
    void setIsLuv(bool flag)
    {
        m_isLuv = flag;
    }
    bool getIsLuv() const
    {
        return m_isLuv;
    }

    void setIsTranspose(bool flag)
    {
        m_isTranspose = flag;
    }
    bool getIsTranspose() const
    {
        return m_isTranspose;
    }

    void setLogger(MatLoggerType logger)
    {
        m_logger = logger;
    }

    void setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
    {
        m_streamLogger = logger;
    }

    void setIsRowMajor(bool flag)
    {
        m_isRowMajor = flag;
    }
    bool getIsRowMajor() const
    {
        return m_isRowMajor;
    }

protected:

    MatLoggerType m_logger;

    std::shared_ptr<spdlog::logger> m_streamLogger;

    double m_detectScorePruneRatio = 0.0;

    //drishti::core::Pyramid m_pyramid;
    bool m_isLuv = false;
    bool m_isTranspose = false;
    bool m_isRowMajor = false;
};

inline cv::Vec3f rgb2luv(const cv::Vec3f &rgb)
{
    // column major format (glsl)
    cv::Matx33f RGBtoXYZ(0.430574, 0.222015, 0.020183, 0.341550, 0.706655, 0.129553, 0.178325, 0.071330, 0.939180);
    RGBtoXYZ = RGBtoXYZ.t(); // to row major

    const float y0 = 0.00885645167; //pow(6.0/29.0, 3.0);
    const float a = 903.296296296;  //pow(29.0/3.0, 3.0);
    const float un = 0.197833;
    const float vn = 0.468331;
    const float maxi = 0.0037037037;  // 1.0/270.0;
    const float minu = maxi * -88.0;
    const float minv = maxi * -134.0;
    const cv::Vec3f k(1.0, 15.0, 3.0);

    cv::Vec3f xyz = (RGBtoXYZ * rgb); // make like glsl col major
    const float c = ( xyz.dot(k) + 1e-35 );
    const float z = 1.0/c;

    cv::Vec3f luv;
    luv[0] = ((xyz[1] > y0) ? (116.0 * std::pow(xyz[1], 0.3333333333) - 16.0) : (xyz[1] * a)) * maxi;
    luv[1] = luv[0] * ((52.0 * xyz[0] * z) - (13.0*un)) - minu;
    luv[2] = luv[0] * ((117.0 * xyz[1] * z) - (13.0*vn)) - minv;

    return luv;
}

template<typename Iterator>
inline void fuseChannels(Iterator begin, Iterator end, MatP &Ip)
{
    std::vector<cv::Mat> stack;
    for(Iterator iter = begin; iter != end; iter++)
    {
        // TODO: bug report iter->begin() leads to boost error
        std::copy((*iter).begin(), (*iter).end(), std::back_inserter(stack));
    }

    cv::Mat base;
    cv::vconcat(stack, base);
    cv::Rect roi({0,0}, stack.front().size());
    for(int j = 0; roi.y < base.rows; j++, roi.y += roi.height)
    {
        stack[j] = base(roi);
    }
    Ip.base() = base;
    Ip.get() = stack;
}

DRISHTI_ACF_NAMESPACE_END

void imResample(const MatP &A, MatP &B, const cv::Size &size, double nrm);

#endif /* defined(__DRISHTI__ACF__) */
