/*******************************************************************************
* Piotr's Image&Video Toolbox      Version 3.21
* Copyright 2013 Piotr Dollar.  [pdollar-at-caltech.edu]
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include "drishti/acf/ACF.h"
#include "drishti/core/Parallel.h"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <boost/multi_array.hpp>

using namespace std;

typedef unsigned int uint32;

#define DRISHTI_ACF_DEBUG_CID 0
#define DRISHTI_ACF_DEBUG_SCANNING 0

DRISHTI_ACF_NAMESPACE_BEGIN

/*
 * These are computed in row major order:
 */

#define GPU_ACF_TRANSPOSE 1 // 1 = compatibility with matlab column major training

using RectVec = std::vector<cv::Rect>;
using UInt32Vec = std::vector<uint32_t>;
static UInt32Vec computeChannelIndex(const RectVec &rois, uint32 rowStride, int modelWd, int modelHt, int width, int height);
static UInt32Vec computeChannelIndexColMajor(int nChns, int modelWd, int modelHt, int width, int height);

class DetectionSink
{
public:
    virtual void add(const cv::Point & p, float value)
    {
        hits.emplace_back(p, value);
    }
    std::vector<std::pair<cv::Point, float>> hits;
};

class DetectionSinkLock : public DetectionSink
{
public:
    virtual void add(const cv::Point & p, float value)
    {
        std::lock_guard<std::mutex> lock(mutex);
        DetectionSink::add(p, value);
    }
    std::mutex mutex;
};

class DetectionParams : public cv::ParallelLoopBody
{
public:
    cv::Size winSize; // possibly transposed
    cv::Size size1;
    cv::Point step1;
    int stride;
    int shrink;
    int rowStride;
    std::vector<uint32_t> cids;
    const uint32 *fids;
    const float *hs = nullptr;
    const float *thrs = nullptr;
    int nTrees;
    int nTreeNodes;
    float cascThr;
    const uint32_t *child = nullptr;

    MatP I;
    cv::Mat canvas;
    
    virtual float evaluate(uint32_t row, uint32_t col) const = 0;
};

template <class T, int kDepth>
class ParallelDetectionBody: public DetectionParams
{
public:

    ParallelDetectionBody(const T *chns, DetectionSink *sink)
        : chns(chns)
        , sink(sink)
    {}
    
    virtual void operator()(const cv::Range &range) const
    {
#if DEBUG_SCANNING
        cv::imshow("I", I.base());
#endif
        for(int c = 0; c < size1.width; c += step1.x)
        {
            for( int r=0; r< size1.height; r += step1.y)
            {
                int offset=(r*stride/shrink) + (c*stride/shrink) * rowStride;
                float h = evaluate(chns, offset);
#if DEBUG_SCANNING
                drawScan(r, c, offset);
#endif
                if(h > cascThr)
                {
                    sink->add({c,r}, h);
                }
            }
        }
    }

    void drawScan(int r, int c, int offset) const
    {
        if(r == c && !(r % 4))
        {
            for(int i = 0; i < cids.size(); i++)
            {
                const_cast<T&>(chns[offset + cids[i]]) = 255*float(i%(12*12))/float(12*12);
            }
        }
    }

    void getChild(const T *chns1, uint32 offset, uint32 &k0, uint32 &k) const
    {
        int index = cids[fids[k]];
        float ftr = chns1[index];
        k = (ftr<thrs[k]) ? 1 : 2;
        k0 = k+= k0*2;
        k += offset;
    }
    
    float evaluate(uint32_t row, uint32_t col) const
    {
        int offset=(row*stride/shrink) + (col*stride/shrink) * rowStride;
        return evaluate(chns, offset);
    }

    float evaluate(const T *chns1, uint32_t index) const
    {
        float h = 0.f;
        for( int t = 0; t < nTrees; t++ )
        {
            uint32 offset=t*nTreeNodes, k=offset, k0=0;
            for(int i = 0; i < kDepth; i++)
            {
                getChild(chns1+index,offset,k0,k);
            }
            h += hs[k];
            if( h<=cascThr )
            {
                break;
            }
        }
        return h;
    }

    // Input params:
    const T *chns;
    DetectionSink *sink;
};

const cv::Mat& Detector::Classifier::getScaledThresholds(int type) const
{
    switch(type)
    {
        case CV_32FC1:
            return thrs;
        case CV_8UC1:
            return thrsU8;
        default:
            assert(false);
    }
}

static std::shared_ptr<DetectionParams> allocDetector(const MatP &I, DetectionSink *sink)
{
    switch(I.depth())
    {
        case CV_8UC1:
            return std::make_shared<ParallelDetectionBody<uint8_t, 2>>(I[0].ptr<uint8_t>(), sink);
        case CV_32FC1:
            return std::make_shared<ParallelDetectionBody<float, 2>>(I[0].ptr<float>(), sink);
        default:
            assert(false);
    }
}

auto Detector::createDetector(const MatP &I, const RectVec &rois, int shrink, cv::Size modelDsPad, int stride, DetectionSink *sink) const -> DetectionParamPtr
{
    int modelHt = modelDsPad.height;
    int modelWd = modelDsPad.width;
    
    cv::Size chnsSize = I.size();
    int height = chnsSize.height;
    int width = chnsSize.width;
    int nChns = I.channels();
    int rowStride = static_cast<int>(I[0].step1());
    
    if(!m_isRowMajor)
    {
        std::swap(height, width);
        std::swap(modelHt, modelWd);
    }
    
    const int height1 = (int) ceil(float(height*shrink-modelHt+1)/stride);
    const int width1 = (int) ceil(float(width*shrink-modelWd+1)/stride);
    
    // Precompute channel offsets:
    std::vector<uint32_t> cids;
    if(rois.size())
    {
        cids = computeChannelIndex(rois, rowStride, modelWd/shrink, modelHt/shrink, width, height);
    }
    else
    {
        cids = computeChannelIndexColMajor(nChns, modelWd/shrink, modelHt/shrink, width, height);
    }

    // Extract relevant fields from trees
    // Note: Need tranpose for column-major storage
    auto & trees = clf;
    int nTreeNodes = trees.fids.rows; // TODO: check?
    int nTrees = trees.fids.cols;
    std::swap(nTrees, nTreeNodes);
    assert( trees.treeDepth == 2); // TODO: switch
    cv::Mat thresholds = trees.getScaledThresholds(I.depth());
    
    std::shared_ptr<DetectionParams> detector = allocDetector(I, sink);
    
    // Scanning parameters
    detector->winSize = { modelWd, modelHt };
    detector->size1 = { width1, height1 };
    detector->step1 = { 1, 1 };
    detector->stride = stride;
    detector->shrink = shrink;
    detector->rowStride = rowStride;
    detector->cids = cids;
    
    // Tree parameters:
    detector->thrs = thresholds.ptr<float>();
    detector->fids = trees.fids.ptr<uint32_t>();
    detector->nTrees = nTrees;
    detector->nTreeNodes = nTreeNodes;
    detector->hs = trees.hs.ptr<float>();
    detector->child = trees.child.ptr<uint32_t>();
    detector->I = I;
    
    return detector;
}

// Changelog:
//
// 3/21/2015: Rework arithmetic for row-major storage order

void Detector::acfDetect1(const MatP &I, const RectVec &rois, int shrink, cv::Size modelDsPad, int stride, double cascThr, std::vector<Detection> &objects)
{
    //DetectionSinkLock detections;
    //auto detector = createDetector(I, rois, shrink, modelDsPad, stride, &detections);
    //detector->cascThr = cascThr;
    //cv::parallel_for_({0, detector->size1.width}, *detector);

    DetectionSink detections;
    auto detector = createDetector(I, rois, shrink, modelDsPad, stride, &detections);
    detector->cascThr = cascThr;
    (*detector)({0, detector->size1.width});
    
    for(const auto &hit : detections.hits)
    {
        cv::Rect roi({hit.first.x*stride, hit.first.y*stride}, detector->winSize);
#if GPU_ACF_TRANSPOSE
        std::swap(roi.x, roi.y);
        std::swap(roi.width, roi.height);
#endif
        objects.push_back(Detection(roi, hit.second));
    }
}

float Detector::evaluate(const MatP &I, int shrink, cv::Size modelDsPad, int stride) const
{
    auto detector = createDetector(I, {}, shrink, modelDsPad, stride, nullptr);
    detector->cascThr = 0.f;
    return detector->evaluate(0,0);
}

// local static utility routines:

static UInt32Vec computeChannelIndex(const RectVec &rois, uint32 rowStride, int modelWd, int modelHt, int width, int height)
{
#if GPU_ACF_TRANSPOSE
    assert(rois.size() > 1);
    int nChns = static_cast<int>(rois.size());
    int chnStride = rois[1].x - rois[0].x;
    
    UInt32Vec cids(nChns * modelWd * modelHt);
    
    int m = 0;
    for( int z=0; z<nChns; z++ )
    {
        for( int c=0; c<modelWd; c++ )
        {
            for( int r=0; r<modelHt; r++ )
            {
                cids[m++] = z*chnStride + c*rowStride + r;
            }
        }
    }
    return cids;
#else
    
    assert(rois.size() > 1);
    int nChns = static_cast<int>(rois.size());
    int chnStride = rowStride * (rois[1].y - rois[0].y);
    
    UInt32Vec cids(nChns * modelWd * modelHt);
    
    int m = 0;
    for( int z=0; z<nChns; z++ )
    {
        for( int c=0; c<modelWd; c++ )
        {
            for( int r=0; r<modelHt; r++ )
            {
                cids[m++] = z*chnStride + r*rowStride + c;
            }
        }
    }
    return cids;
#endif
}

static UInt32Vec computeChannelIndexColMajor(int nChns, int modelWd, int modelHt, int width, int height)
{
    UInt32Vec cids(nChns * modelWd * modelHt);
    
    int m = 0, area = (width * height);
    for( int z=0; z<nChns; z++ )
    {
        for( int c=0; c<modelWd; c++ )
        {
            for( int r=0; r<modelHt; r++ )
            {
                cids[m++] = z*area + c*height + r;
            }
        }
    }
    return cids;
}

DRISHTI_ACF_NAMESPACE_END
