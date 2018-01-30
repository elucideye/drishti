/*! -*-c++-*-
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
#include "drishti/core/scope_guard.h"

#include "ogles_gpgpu/common/proc/transform.h"
#include "ogles_gpgpu/common/proc/lowpass.h"
#include "ogles_gpgpu/common/proc/highpass.h"
#include "ogles_gpgpu/common/proc/diff.h"
#include "ogles_gpgpu/common/proc/gauss_opt.h"
#include "ogles_gpgpu/common/proc/fifo.h"
#include "ogles_gpgpu/common/proc/fir3.h"
#include "ogles_gpgpu/common/common_includes.h"

#include <memory>

BEGIN_OGLES_GPGPU

static void convert(const drishti::eye::EyeWarp& src, ogles_gpgpu::MappedTextureRegion& dst);

EyeFilter::EyeFilter(const Size2d& sizeOut, Mode mode, float cutoff, int history)
    : m_history(history)
    , m_sizeOut(sizeOut)
{
    firstProc = &transformProc;

    transformProc.setInterpolation(TransformProc::BICUBIC);

    // Add a fifo proc for GPU sliding window:
    fifoProc = drishti::core::make_unique<ogles_gpgpu::FifoProc>(history);
    transformProc.add(fifoProc.get());

    procPasses = { &transformProc, fifoProc.get() };

    switch (mode)
    {
        case kIirLowPass:
        {
            lowPassProc = drishti::core::make_unique<LowPassFilterProc>(cutoff);
            procPasses.push_back(lowPassProc.get());

            transformProc.add(lowPassProc.get());
            lastProc = lowPassProc.get();
        }
        break;

        case kMean3:
        {
            mean3Proc = drishti::core::make_unique<ogles_gpgpu::Fir3Proc>();
            mean3Proc->setWeights({ 0.33f, 0.33f, 0.33f });
            procPasses.push_back(mean3Proc.get());

            fifoProc->addWithDelay(mean3Proc.get(), 0, 0);
            fifoProc->addWithDelay(mean3Proc.get(), 1, 1);
            fifoProc->addWithDelay(mean3Proc.get(), 2, 2);

            lastProc = mean3Proc.get();
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

void EyeFilter::dump(std::vector<cv::Mat4b>& frames, std::vector<EyePair>& eyes, int n, bool getImage)
{
    // FifoProc::operator[] will preserve temporal ordering
    auto length = fifoProc->getBufferCount();
    n = std::min(n, static_cast<int>(length));
    frames.resize(n);
    eyes.resize(n);
    for (int i = 0; i < n; i++)
    {
        if (getImage)
        {
            // Pull frames in reverse order such that frames[0] is newest
            auto* filter = (*fifoProc)[i];
            frames[i].create(filter->getOutFrameH(), filter->getOutFrameW());
            filter->getResultData(frames[i].ptr<uint8_t>());
        }

        cv::Matx33f N = transformation::denormalize(frames[i].size());
        for (int j = 0; j < m_eyeHistory[i].size(); j++)
        {
            const auto& e = m_eyeHistory[i][j];
            eyes[i][j] = (N * e.H) * e.eye;
        }
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

ProcInterface* EyeFilter::getInputFilter() const
{
    return firstProc;
}

ProcInterface* EyeFilter::getOutputFilter() const
{
    return lastProc;
}

void EyeFilter::renderIris()
{
}

int EyeFilter::render(int position)
{
    // If we have faces, then configure appropriate transformations:
    drishti::face::FaceStabilizer stabilizer({ m_sizeOut.width, m_sizeOut.height });
    stabilizer.setDoAutoScaling(m_doAutoScaling);

    if (m_faces.size())
    {
        // For now we display eyes from the first face:
        m_eyes = stabilizer.renderEyes(m_faces[0], { getInFrameW(), getInFrameH() });
        for (int i = 0; i < 2; i++)
        {
            MappedTextureRegion region;
            convert(m_eyes[i], region);
            transformProc.addCrop(region);
        }
    }

    // Maintain eye history queue of size == 3
    m_eyeHistory.push_front(m_eyes);
    if (m_eyeHistory.size() > m_history)
    {
        m_eyeHistory.pop_back();
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

static void convert(const drishti::eye::EyeWarp& src, ogles_gpgpu::MappedTextureRegion& dst)
{
    cv::Matx44f MVPt;
    transformation::R3x3To4x4(src.H.t(), MVPt);
    dst.roi = Rect2d(src.roi.x, src.roi.y, src.roi.width, src.roi.height);
    for (int y = 0; y < 4; y++)
    {
        for (int x = 0; x < 4; x++)
        {
            dst.H.data[y][x] = MVPt(y, x);
        }
    }
}

END_OGLES_GPGPU
