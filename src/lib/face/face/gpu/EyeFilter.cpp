/*!
  @file   face/gpu/EyeFilter.cpp
  @author David Hirvonen
  @brief  Perform temporal filtering on stabilized eye crops.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "face/gpu/EyeFilter.h"
#include "geometry/motion.h"
#include "eye/IrisNormalizer.h"

#include <memory>

BEGIN_OGLES_GPGPU

static void convert(const EyeWarp &src, ogles_gpgpu::MappedTextureRegion &dst);

#define DO_LOW_PASS 1

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
        smoothProc = std::unique_ptr<ogles_gpgpu::GaussOptProc>(new ogles_gpgpu::GaussOptProc(5));
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
//const ProcInterface * EyeFilter::getInputFilter() const { return firstProc; }

ProcInterface * EyeFilter::getOutputFilter() const
{
    return lastProc;
}
//const ProcInterface * EyeFilter::getOutputFilter() const { return lastProc; }

void EyeFilter::renderIris()
{

}

int EyeFilter::render(int position)
{
    // If we have faces, then configure appropriate transformations:
    FaceStabilizer stabilizer({m_sizeOut.width, m_sizeOut.height});
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
    dst.roi = { src.roi.x, src.roi.y, src.roi.width, src.roi.height };
    for(int y = 0; y < 4; y++)
    {
        for(int x = 0; x < 4; x++)
        {
            dst.H.data[y][x] = MVPt(y,x);
        }
    }
}

END_OGLES_GPGPU
