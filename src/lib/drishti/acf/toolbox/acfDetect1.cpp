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

DRISHTI_ACF_BEGIN

/*
 * These are computed in row major order:
 */

#define GPU_ACF_TRANSPOSE 1

using RectVec = std::vector<cv::Rect>;
using UInt32Vec = std::vector<uint32_t>;

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

struct DetectionResults
{
    virtual void add(const cv::Point & p, float value)
    {
        hits.emplace_back(p, value);
    }
    std::vector<std::pair<cv::Point, float>> hits;
};

struct DetectionResultsMT : public DetectionResults
{
    virtual void add(const cv::Point & p, float value)
    {
        std::lock_guard<std::mutex> lock(mutex);
        DetectionResults::add(p, value);
    }
    std::mutex mutex;
};

struct DetectionParams : public cv::ParallelLoopBody
{
    cv::Size size1;
    cv::Point step1;
    int stride;
    int shrink;
    int rowStride;
    std::vector<uint32_t> cids;
    uint32 *fids;
    float *hs = nullptr;
    float *thrs = nullptr;
    int nTrees;
    int nTreeNodes;
    float cascThr;
    uint32_t *child = nullptr;

    MatP I;
    cv::Mat canvas;
};

#define DEBUG_SCANNING 0

template <class T, int kDepth>
class ParallelDetectionBody: public DetectionParams
{
public:

    ParallelDetectionBody(const T *chns, DetectionResults &results)
        : chns(chns)
        , results(results)
    {}

    virtual void operator()( const cv::Range &range) const
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
                if(h>cascThr)
                {
                    results.add({c,r}, h);
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
            }// cids.size()); }
            {
                cv::imshow("canvas", I.base());
                cv::waitKey(0);
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

    DetectionResults &results;
};

static std::shared_ptr<DetectionParams> createDetector(const MatP &I, DetectionResults &results)
{
    switch(I.depth())
    {
        case CV_8UC1:
            return std::make_shared<ParallelDetectionBody<uint8_t, 2>>(I[0].ptr<uint8_t>(), results);
        case CV_32FC1:
            return std::make_shared<ParallelDetectionBody<float, 2>>(I[0].ptr<float>(), results);
        default:
            assert(false);
    }
}

const cv::Mat& Detector::Classifier::getScaledThresholds(int type)
{
    switch(type)
    {
        case CV_32FC1:
            return thrs;
        case CV_8UC1:
        {
            if(thrsU8.empty())
            {
                thrsU8 = thrs * 255.0;
            }
            return thrsU8;
        }
        default:
            assert(false);
    }
}

// Changelog:
//
// 3/21/2015: Rework arithmetic for row-major storage order

void Detector::acfDetect1(const MatP &I, const RectVec &rois, int shrink, cv::Size modelDsPad, int stride, double cascThr, std::vector<Detection> &objects)
{
    // get dimensions and constants
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

#define DEBUG_CID 0
#if DEBUG_CID
        uint8_t *ptr = (uint8_t *) I[0].ptr<uint8_t>();
        for(int i = 0; i < cids.size(); i++)
        {
            ptr[cids[i]] = i % 255; // 255;
        }
        cv::imshow("base", I.base());
#endif
    }
    else
    {
        cids = computeChannelIndexColMajor(nChns, modelWd/shrink, modelHt/shrink, width, height);
    }

    // apply classifier to each patch

    DetectionResultsMT detections;

    // Need tranpose for column-major storage
    // extract relevant fields from trees
    auto & trees = clf;
    int nTreeNodes = trees.fids.rows; // TODO: check?
    int nTrees = trees.fids.cols;
    std::swap(nTrees, nTreeNodes);
    assert( trees.treeDepth == 2); // TODO: switch
    cv::Mat thresholds = trees.getScaledThresholds(I.depth());

    std::shared_ptr<DetectionParams> detector = createDetector(I, detections);

    // Scanning parameters
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
    detector->cascThr = cascThr;
    detector->child = trees.child.ptr<uint32_t>();
    detector->I = I;

    //cv::parallel_for_({0, width1}, *detector, 4);
    (*detector)({0, width1});

    for(const auto &hit : detections.hits)
    {
        cv::Rect roi(hit.first.x*stride, hit.first.y*stride, modelWd, modelHt);
#if GPU_ACF_TRANSPOSE
        std::swap(roi.x, roi.y);
#endif
        objects.push_back(Detection(roi, hit.second));
    }
}

DRISHTI_ACF_END
