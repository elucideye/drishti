/*!
  @file   ACF.cpp
  @author David Hirvonen (C++ implementation)
  @author P. Dollár (original matlab code)
  @brief  Aggregated Channel Feature object detector implementation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/acf/ACF.h"
#include "drishti/acf/ACFIO.h"

#include "drishti/core/IndentingOStreamBuffer.h"

#include <iomanip>

DRISHTI_ACF_NAMESPACE_BEGIN

////////////////////////////////////////////////////////

Detector::Detector(const Detector &src)
{
    clf = src.clf;
    opts = src.opts;
}

Detector::Detector(std::istream &is, const std::string &hint)
{
    deserializeAny(is, hint);
}

Detector::Detector(const std::string &filename)
{
    deserializeAny(filename);
}

int Detector::initializeOpts() // Seems to be required
{
    {
        Options dfs;
        dfs.modelDs = { "modelDs", cv::Size(41,100) };
        dfs.modelDsPad = { "modelDsPad", cv::Size(64, 128) };
        dfs.stride = { "stride", 4 };
        dfs.cascThr = { "cascThr", -1 };
        dfs.nWeak = { "nWeak", std::vector<int>{128} };
        dfs.seed = { "seed", 0 };
        dfs.nPos = { "nPos", std::numeric_limits<int>::infinity() };
        dfs.nNeg = { "nNeg", 5000 };
        dfs.nAccNeg = { "nAccNeg", 10000 };
        dfs.winsSave = { "winSave", 0 };
        opts.merge(dfs, 1);
    }

    Pyramid pyramid;

    chnsPyramid({}, &opts.pPyramid.get(), pyramid, true);
    auto p = pyramid.pPyramid;
    auto shrink = *(p.pChns->shrink);
    opts.modelDsPad = ((*(opts.modelDsPad))/shrink)*shrink;
    p.pad = ((opts.modelDsPad.get() - opts.modelDs.get())/shrink/2)*shrink; // TODO: check this
    p.minDs = opts.modelDs;

    chnsPyramid({}, &p, pyramid, true);
    p = pyramid.pPyramid;
    p.complete = 1;
    p.pChns->complete = 1;
    opts.pPyramid = p;

    // initialize pNms, pBoost, pBoost.pTree, and pLoad
    {
        Options::Nms dfs;
        dfs.type = { "type", std::string("maxg") };
        dfs.overlap = { "overlap", 0.65 };
        dfs.ovrDnm = { "ovrDnm", std::string("min") };
        opts.pNms.merge(dfs, 1);
    }

    {
        Options::Boost dfs;
        dfs.nWeak = { "nWeak", 0 };
        dfs.discrete = { "discrete", 1 };
        dfs.verbose = { "verbose", 16 };
        opts.pBoost.merge(dfs, 1);
    }

    {
        Options::Boost::Tree dfs;
        dfs.nBins = { "nBins", 256 };
        dfs.maxDepth = { "maxDepth", 2 };
        dfs.minWeight = { "minWeight", 0.01 };
        dfs.fracFtrs = { "fracFtrs", 1 };
        dfs.nThreads = { "nThreads", 1e5 };
        opts.pBoost->pTree.merge(dfs, 1);
    }

    // TODO:
    // opts.pLoad=getPrmDflt(opts.pLoad,{'squarify',{0,1}},-1);
    // opts.pLoad.squarify{2}=opts.modelDs(2)/opts.modelDs(1);

    return 0;
}

cv::Mat cvt8UC3To32FC3(const cv::Mat &I)
{
    cv::Mat If;
    I.convertTo(If, CV_32FC3);
    return If;
}

int Detector::operator()(const cv::Mat &I, std::vector<cv::Rect> &objects, std::vector<double> *scores)
{
    cv::Mat It = m_isTranspose ? I : I.t();
    cv::Mat Itf = (It.depth() == CV_32F) ? It : cvt8UC3To32FC3(It);
    MatP Ip( Itf );
    return (*this)(Ip, objects, scores);
}

/*
 * Compute pyramid from input image
 */

void Detector::computePyramid(const cv::Mat &I, Pyramid &P)
{
    cv::Mat It = m_isTranspose ? I : I.t();
    cv::Mat Itf = (It.depth() == CV_32F) ? It : cvt8UC3To32FC3(It);
    MatP Ip( Itf );
    computePyramid(Ip, P);
}

void Detector::computePyramid(const MatP &Ip, Pyramid &P)
{
    CV_Assert(Ip[0].depth() == CV_32F);
    
    auto &pPyramid = *(opts.pPyramid);
    auto pad = *(pPyramid.pad);
    auto modelDsPad = *(opts.modelDsPad);
    auto modelDs = *(opts.modelDs);

    chnsPyramid(Ip, &opts.pPyramid.get(), P, true);
}

/*
 * Compute channels from input image
 */

void Detector::computeChannels(const cv::Mat &I, MatP &Ip2, MatLoggerType pLogger)
{
    CV_Assert(I.channels() == 3);

    cv::Mat If;
    if(I.depth() != CV_32F)
    {
        I.convertTo(If, CV_32FC3, (1.0/255.0));
    }
    else
    {
        If = I;
    }

    MatP Ip(If);
    computeChannels(Ip, Ip2, pLogger);
}

void Detector::computeChannels(const MatP &Ip, MatP &Ip2, MatLoggerType pLogger)
{
    // 'pChns',{},'nPerOct',8,'nOctUp',0,'nApprox',-1,'lambdas',[],'pad',[0 0],'minDs',[16 16],'smooth',1,'concat',1,'complete',1};
    Options::Pyramid dfs;
    dfs.nPerOct = {"nPerOct", 4};
    dfs.nOctUp = {"nOctUp", 0};
    dfs.nApprox = {"nApprox", -1};
    dfs.pad = {"pad", cv::Size(0,0)};
    dfs.minDs = {"minDs", cv::Size(16,16)};
    dfs.smooth = {"smooth", 1};
    dfs.concat = {"concat", 1};
    dfs.complete = {"complete", 1};

    auto &pChns = (*dfs.pChns);

    {
        // top level
        Options::Pyramid::Chns dfs;
        dfs.shrink = {"shrink", 4};
        dfs.complete = {"complete", 1};
        pChns.merge(dfs, 1);
    }
    {
        // pColor
        Options::Pyramid::Chns::Color dfs;
        dfs.enabled = { "enabled", 1 };
        dfs.smooth = { "smooth", 1 };
        dfs.colorSpace = { "colorSpace", "luv" };
        pChns.pColor.merge(dfs, 1);
    }
    {
        // pGradMag
        Options::Pyramid::Chns::GradMag dfs;
        dfs.enabled = { "enabled", 1 };
        dfs.colorChn = { "colorChn", 0 };
        dfs.normRad = { "normRad", 5 };
        dfs.normConst = { "normConst", 0.005 };
        dfs.full = { "full", 0  };
        pChns.pGradMag.merge(dfs, 1);;
    }
    {
        // pGradHist
        Options::Pyramid::Chns::GradHist dfs;
        dfs.enabled = { "enabled", 1 };
        dfs.nOrients = { "nOrients", 6 };
        dfs.softBin = { "softBin", 0 };
        dfs.useHog = { "useHog", 0 };
        dfs.clipHog = { "clipHog", 0.2 };
        pChns.pGradHist.merge(dfs, 1);
    }

    Detector::Channels chns;
    chnsCompute(Ip, pChns, chns, false, pLogger);

    // ((( channels )))
    fuseChannels(chns.data.begin(), chns.data.end(), Ip2);
}

/*
 * Input is transposed planar format image: LUVMO
 */

int Detector::operator()(const MatP &IpTranspose, std::vector<cv::Rect> &objects, std::vector<double> *scores)
{
    // Create features:
    Pyramid P;
    chnsPyramid(IpTranspose, &opts.pPyramid.get(), P, true);

    if(m_logger)
    {
        for(int i = 0; i < P.nScales; i++)
        {
            std::stringstream ss;
            ss << std::setfill('0') << std::setw(6) << i;
            cv::Mat d = P.data[i][0].base().clone().t(), canvas;
            cv::normalize(d, canvas, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            m_logger(canvas, ss.str());
        }
    }

    return (*this)(P, objects, scores);
}

// Multiscale search:
int Detector::operator()(const Pyramid &P, std::vector<cv::Rect> &objects, std::vector<double> *scores)
{
    auto &pPyramid = *(opts.pPyramid);
    auto shrink = *(pPyramid.pChns->shrink);
    auto pad = *(pPyramid.pad);
    auto modelDsPad = *(opts.modelDsPad);
    auto modelDs = *(opts.modelDs);
    auto shift=(modelDsPad-modelDs)/2-pad;

    std::vector< Detection > bbs;
    for(int i = 0; i < P.nScales; i++)
    {
        DetectionVec ds;

        // ROI fields indicates row major storage, else column major:
        if(P.rois.size() > i)
        {
            acfDetect1(P.data[i][0], P.rois[i], shrink, modelDsPad, *(opts.stride), *(opts.cascThr), ds);
        }
        else
        {
            acfDetect1(P.data[i][0], {}, shrink, modelDsPad, *(opts.stride), *(opts.cascThr), ds);
        }

        // Scale up the detections
        for(auto &bb : ds)
        {
            //std::cout << bb.weight << std::endl;
            cv::Size size(cv::Size2d(modelDs) / P.scales[i]);
            bb.roi.x = double(bb.roi.x + shift.width) / P.scaleshw[i].width;
            bb.roi.y = double(bb.roi.y + shift.height) / P.scaleshw[i].height;
            bb.roi.width = size.width;
            bb.roi.height = size.height;

            std::swap(bb.roi.x, bb.roi.y); // TODO: review

            std::swap(bb.roi.width, bb.roi.height); // TRANSPOSE
        }
        std::copy(ds.begin(), ds.end(), std::back_inserter(bbs));
    }

    if(m_doNms)
    {
        if(bbs.size())
        {
            // Run non maximal suppression:
            DetectionVec bbOut;
            bbNms(bbs, opts.pNms, bbOut);
            std::copy(bbOut.begin(), bbOut.end(), std::back_inserter(objects));

            std::vector<double> bbScores;
            for(auto &b : bbOut)
            {
                bbScores.push_back(b.score);
            }

            prune(objects, bbScores);
            if(scores)
            {
                *scores = bbScores;
            }
        }
    }
    else
    {
        std::copy(bbs.begin(), bbs.end(), std::back_inserter(objects));
        if(scores)
        {
            for(auto &b : bbs)
            {
                scores->push_back(b.score);
            }
        }
    }

    return 0;
}

// (((((((((((((((((((( ostream ))))))))))))))))))))

std::ostream& operator<<(std::ostream &os, const Detector::Options::Pyramid::Chns::Color &src)
{
    os << src.enabled << std::endl;
    os << src.smooth << std::endl;
    os << src.colorSpace;
    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Options::Pyramid::Chns::GradMag &src)
{
    os << src.enabled << std::endl;
    os << src.colorChn << std::endl;
    os << src.normRad << std::endl;
    os << src.normConst << std::endl;
    os << src.full;
    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Options::Pyramid::Chns::GradHist &src)
{
    os << src.enabled << std::endl;
    os << src.binSize << std::endl;
    os << src.nOrients << std::endl;
    os << src.softBin << std::endl;
    os << src.useHog;
    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Options::Pyramid::Chns::Custom &src)
{
    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Options::Pyramid &src)
{
    os << src.nPerOct << std::endl;
    os << src.nOctUp << std::endl;
    os << src.nApprox << std::endl;
    os << src.lambdas << std::endl;
    os << src.pad << std::endl;
    os << src.minDs << std::endl;
    os << src.smooth << std::endl;
    os << src.concat << std::endl;
    os << src.complete << std::endl;
    os << src.pChns;
    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Options::Pyramid::Chns &src)
{
    os << src.shrink << std::endl;
    os << src.pColor << std::endl;
    os << src.pGradMag << std::endl;
    os << src.pGradHist << std::endl;
    os << src.pCustom << std::endl;
    os << src.complete;
    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Options::Nms &src)
{
    os << src.type << std::endl;
    os << src.overlap << std::endl;
    os << src.ovrDnm;
    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Options::Boost::Tree &src)
{
    os << src.nBins << std::endl;
    os << src.maxDepth << std::endl;
    os << src.minWeight << std::endl;
    os << src.fracFtrs << std::endl;
    os << src.nThreads;
    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Options::Boost &src)
{
    os << src.nWeak << std::endl;
    os << src.discrete << std::endl;
    os << src.verbose << std::endl;
    os << src.pTree;
    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Options::Jitter &src)
{
    os << src.flip;
    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Options &src)
{
    os << src.modelDs << std::endl;
    os << src.modelDsPad << std::endl;
    os << src.stride << std::endl;
    os << src.cascThr << std::endl;
    os << src.cascCal << std::endl;
    os << src.nWeak << std::endl;
    os << src.seed << std::endl;
    os << src.name << std::endl;
    os << src.posGtDir << std::endl;
    os << src.posImgDir << std::endl;
    os << src.negImgDir << std::endl;
    os << src.posWinDir << std::endl;
    os << src.negWinDir << std::endl;
    //os << src.imreadf << std::endl;
    //os << src.imreadp << std::endl;
    //os << src.pLoad << std::endl;
    os << src.nPos << std::endl;
    os << src.nNeg << std::endl;
    os << src.nPerNeg << std::endl;
    os << src.nAccNeg << std::endl;
    os << src.pJitter << std::endl;
    os << src.winsSave << std::endl;
    os << src.pBoost << std::endl;
    os << src.pNms << std::endl;
    os << src.pPyramid << std::endl;

    return os;
}

std::ostream& operator<<(std::ostream &os, const Detector::Modify &src)
{
    os << src.nPerOct << std::endl;
    os << src.nOctUp << std::endl;
    os << src.nApprox << std::endl;
    os << src.lambdas << std::endl;
    os << src.pad << std::endl;
    os << src.minDs << std::endl;
    os << src.pNms << std::endl;
    os << src.stride << std::endl;
    os << src.cascThr << std::endl;
    os << src.cascCal << std::endl;
    os << src.rescale << std::endl;

    return os;
}

// (((((((((((((((((((((((( merge ))))))))))))))))))))))))
void Detector::Options::Pyramid::Chns::Color::merge(const Color &src, int checkExtra)
{
    enabled.merge(src.enabled, checkExtra);
    smooth.merge(src.smooth, checkExtra);
    colorSpace.merge(src.colorSpace, checkExtra);
}

void Detector::Options::Pyramid::Chns::GradMag::merge(const GradMag &src, int checkExtra)
{
    enabled.merge(src.enabled, checkExtra);
    colorChn.merge(src.colorChn , checkExtra);
    normRad.merge(src.normRad, checkExtra);
    normConst.merge(src.normConst, checkExtra);
    full.merge(src.full, checkExtra);
}

void Detector::Options::Pyramid::Chns::GradHist::merge(const GradHist &src, int checkExtra)
{
    enabled.merge(src.enabled, checkExtra);
    binSize.merge(src.binSize, checkExtra);
    nOrients.merge(src.nOrients, checkExtra);
    softBin.merge(src.softBin, checkExtra);
    useHog.merge(src.useHog, checkExtra);
}

void Detector::Options::Pyramid::Chns::Custom::merge(const Custom &src, int checkExtra)
{

}

void Detector::Options::Pyramid::merge(const Pyramid &src, int checkExtra)
{
    nPerOct.merge(src.nPerOct, checkExtra);
    nOctUp.merge(src.nOctUp, checkExtra);
    nApprox.merge(src.nApprox, checkExtra);
    lambdas.merge(src.lambdas, checkExtra);
    pad.merge(src.pad, checkExtra);
    minDs.merge(src.minDs, checkExtra);
    smooth.merge(src.smooth, checkExtra);
    concat.merge(src.concat, checkExtra);
    complete.merge(src.complete, checkExtra);
    pChns.merge(src.pChns, checkExtra);
}


void Detector::Options::Pyramid::Chns::merge(const Chns &src, int checkExtra)
{
    shrink.merge(src.shrink, checkExtra);
    pColor.merge(src.pColor, checkExtra);
    pGradMag.merge(src.pGradMag, checkExtra);
    pGradHist.merge(src.pGradHist, checkExtra);
    pCustom.merge(src.pCustom, checkExtra);
    complete.merge(src.complete, checkExtra);
}

void Detector::Options::Nms::merge(const Nms &src, int checkExtra)
{
    type.merge(src.type, checkExtra);
    overlap.merge(src.overlap, checkExtra);
    ovrDnm.merge(src.ovrDnm, checkExtra);
    thr.merge(src.thr, checkExtra);
    maxn.merge(src.maxn, checkExtra);
    radii.merge(src.radii, checkExtra);
    separate.merge(src.separate, checkExtra);
}

void Detector::Options::Boost::Tree::merge(const Tree &src, int checkExtra)
{
    nBins.merge(src.nBins, checkExtra);
    maxDepth.merge(src.maxDepth, checkExtra);
    minWeight.merge(src.minWeight, checkExtra);
    fracFtrs.merge(src.fracFtrs, checkExtra);
    nThreads.merge(src.nThreads, checkExtra);
}

void Detector::Options::Boost::merge(const Boost &src, int checkExtra)
{
    nWeak.merge(src.nWeak, checkExtra);
    discrete.merge(src.discrete, checkExtra);
    verbose.merge(src.verbose, checkExtra);
    pTree.merge(src.pTree, checkExtra);
}

void Detector::Options::Jitter::merge(const Jitter &src, int checkExtra)
{
    //flip;
}

void Detector::Options::merge(const Options &src, int checkExtra)
{
    modelDs.merge(src.modelDs, checkExtra);
    modelDs.merge(src.modelDsPad, checkExtra);
    stride.merge(src.stride, checkExtra);
    cascThr.merge(src.cascThr, checkExtra);
    cascCal.merge(src.cascCal, checkExtra);
    nWeak.merge(src.nWeak, checkExtra);
    seed.merge(src.seed, checkExtra);
    name.merge(src.name, checkExtra);
    posGtDir.merge(src.posGtDir, checkExtra);
    posImgDir.merge(src.posImgDir, checkExtra);
    negImgDir.merge(src.negImgDir, checkExtra);
    posWinDir.merge(src.posWinDir, checkExtra);
    negWinDir.merge(src.negWinDir, checkExtra);

    // imreadf
    // imreadp
    // pLoad

    nPos.merge(src.nPos, checkExtra);
    nNeg.merge(src.nNeg, checkExtra);
    nPerNeg.merge(src.nPerNeg, checkExtra);
    nAccNeg.merge(src.nAccNeg, checkExtra);
    pJitter.merge(src.pJitter, checkExtra);
    winsSave.merge(src.winsSave, checkExtra);
    pBoost.merge(src.pBoost, checkExtra);
    pNms.merge(src.pNms, checkExtra);
    pPyramid.merge(src.pPyramid, checkExtra);
}

DRISHTI_ACF_NAMESPACE_END
