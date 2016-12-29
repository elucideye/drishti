/*!
  @file   face/gpu/EyeFilter.cpp
  @author David Hirvonen
  @brief  Perform temporal filtering on stabilized eye crops.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/gpu/EyeFilter.h"
#include "drishti/geometry/motion.h"
#include "drishti/eye/IrisNormalizer.h"
#include "drishti/core/make_unique.h"

#include "ogles_gpgpu/common/proc/transform.h"
#include "ogles_gpgpu/common/proc/lowpass.h"
#include "ogles_gpgpu/common/proc/highpass.h"
#include "ogles_gpgpu/common/proc/diff.h"
#include "ogles_gpgpu/common/proc/gauss_opt.h"
#include "ogles_gpgpu/common/proc/fifo.h"
#include "ogles_gpgpu/common/common_includes.h"

#include <memory>

BEGIN_OGLES_GPGPU

static void convert(const EyeWarp &src, ogles_gpgpu::MappedTextureRegion &dst);

EyeFilter::EyeFilter(const Size2d &sizeOut, Mode mode, float upper, float lower, float gain, float offset)
    : m_sizeOut(sizeOut)
    , m_upper(upper)
    , m_lower(lower)
    , m_gain(gain)
    , m_offset(offset)
    , m_doSmoothing(mode == kBandPass)
{
    transformProc.setInterpolation(TransformProc::BICUBIC);

    if(m_doSmoothing)
    {
        smoothProc = drishti::core::make_unique<ogles_gpgpu::GaussOptProc>(5);
        procPasses.push_back(smoothProc.get());
        firstProc = smoothProc.get();

        procPasses.push_back(&transformProc);
        smoothProc->add(&transformProc);
    }
    else
    {
        procPasses.push_back(&transformProc);
        firstProc = &transformProc;
    }

    // Add a fifo proc for GPU sliding window:
    fifoProc = drishti::core::make_unique<ogles_gpgpu::FifoProc>(3);
    transformProc.add(fifoProc.get());    

    switch(mode)
    {
        case kLowPass:
        {
            lowPassProc = std::unique_ptr<LowPassFilterProc>(new LowPassFilterProc(m_upper));

            procPasses.push_back( lowPassProc.get() );
            transformProc.add(lowPassProc.get());
            lastProc = lowPassProc.get();
        }
        break;

        case kBandPass:
        {
            lowPassProc = std::unique_ptr<LowPassFilterProc>(new LowPassFilterProc(m_lower));
            procPasses.push_back(lowPassProc.get());

            transformProc.add(lowPassProc.get());

            lowPassProc2 = std::unique_ptr<LowPassFilterProc>(new LowPassFilterProc(m_upper));
            procPasses.push_back(lowPassProc2.get());
            transformProc.add(lowPassProc2.get());

            diffProc = std::unique_ptr<DiffProc>(new DiffProc(m_gain)); // TODO: , m_offset));
            procPasses.push_back(diffProc.get());
            lowPassProc->add(diffProc.get(), 0);
            lowPassProc2->add(diffProc.get(), 1);

            lastProc = diffProc.get();
        }
        break;

        case kNone:
        default:
        {
            lastProc = &transformProc;
        }
        break;
    }
}

EyeFilter::~EyeFilter()
{
    procPasses.clear();
}

void EyeFilter::dump(std::vector<cv::Mat4b> &frames)
{
    // FifoProc::operator[] will preserve temporal ordering:
    frames.resize(fifoProc->getBufferCount());
    for(int i = 0; i < frames.size(); i++)
    {
        frames[i].create((*fifoProc)[i]->getOutFrameH(), (*fifoProc)[i]->getOutFrameW());
        (*fifoProc)[i]->getResultData(frames[i].ptr<uint8_t>());
    }
}

// Make sure to apply resize to transform, not gaussian smoother (if enabled)
void EyeFilter::setOutputSize(float scaleFactor)
{
    transformProc.setOutputSize(scaleFactor);
}
void EyeFilter::setOutputSize(int outW, int outH)
{
    transformProc.setOutputSize(outW, outH);
}

ProcInterface * EyeFilter::getInputFilter() const
{
    return firstProc;
}

ProcInterface * EyeFilter::getOutputFilter() const
{
    return lastProc;
}

void EyeFilter::renderIris()
{

}

int EyeFilter::render(int position)
{
    // If we have faces, then configure appropriate transformations:
    drishti::face::FaceStabilizer stabilizer({m_sizeOut.width, m_sizeOut.height});
    stabilizer.setDoAutoScaling(m_doAutoScaling);

    if(m_faces.size())
    {
        m_eyes = stabilizer.renderEyes(m_faces[0], {getInFrameW(),getInFrameH()});
        for(int i = 0; i < 2; i++)
        {
            MappedTextureRegion region;
            convert(m_eyes[i], region);
            transformProc.addCrop(region);
        }
    }

    getInputFilter()->process(position);

    renderIris();

    m_faces.clear();

    return 0;
}

int EyeFilter::init(int inW, int inH, unsigned int order, bool prepareForExternalInput)
{
    getInputFilter()->prepare(inW, inH, 0, std::numeric_limits<int>::max(), 0);
    return 0;
}

int EyeFilter::reinit(int inW, int inH, bool prepareForExternalInput)
{
    getInputFilter()->prepare(inW, inH, 0, std::numeric_limits<int>::max(), 0);
    return 0;
}

// ############ UTILTIY ###############

static void convert(const EyeWarp &src, ogles_gpgpu::MappedTextureRegion &dst)
{
    cv::Matx44f MVPt;
    transformation::R3x3To4x4(src.H.t(), MVPt);
    dst.roi = Rect2d(src.roi.x, src.roi.y, src.roi.width, src.roi.height);
    for(int y = 0; y < 4; y++)
    {
        for(int x = 0; x < 4; x++)
        {
            dst.H.data[y][x] = MVPt(y,x);
        }
    }
}

END_OGLES_GPGPU
