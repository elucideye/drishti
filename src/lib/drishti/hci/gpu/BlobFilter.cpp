/*! -*-c++-*-
  @file   gpu/BlobFilter.cpp
  @author David Hirvonen
  @brief  Illuminate the scene with pulses of LCD light.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/gpu/BlobFilter.h"
#include "drishti/graphics/binomial.h"
#include "drishti/graphics/saturation.h"

#include "ogles_gpgpu/common/proc/hessian.h"
#include "ogles_gpgpu/common/proc/gauss_opt.h"
#include "ogles_gpgpu/common/proc/fifo.h"
#include "ogles_gpgpu/common/proc/fir3.h"
#include "ogles_gpgpu/common/proc/nms.h"

#include <opencv2/core.hpp>

#include <limits>

BEGIN_OGLES_GPGPU

class BlobFilter::Impl
{
public:
    Impl()
        : smoothProc1(1)
        , hessianProc1(2000.0f, false)
        , saturationProc(1.0)
    {
        nmsProc1.swizzle(1, 3); // in(2), out(3)

        smoothProc1.add(&saturationProc);
        saturationProc.add(&hessianProc1);
        hessianProc1.add(&nmsProc1);
    }

    ogles_gpgpu::GaussOptProc smoothProc1;
    ogles_gpgpu::HessianProc hessianProc1;
    ogles_gpgpu::NmsProc nmsProc1;
    ogles_gpgpu::SaturationProc saturationProc;
};

BlobFilter::BlobFilter()
{
    m_impl = std::make_shared<Impl>();

    // Add filters to procPasses for state management
    procPasses.push_back(&m_impl->smoothProc1);
    procPasses.push_back(&m_impl->hessianProc1);
    procPasses.push_back(&m_impl->nmsProc1);
    procPasses.push_back(&m_impl->saturationProc);
}

BlobFilter::~BlobFilter()
{
    procPasses.clear();
}

ProcInterface* BlobFilter::getInputFilter() const
{
    return &m_impl->smoothProc1;
}

ProcInterface* BlobFilter::getHessian() const
{
    return &m_impl->hessianProc1;
}

ProcInterface* BlobFilter::getHessianPeaks() const
{
    return &m_impl->nmsProc1;
}

ProcInterface* BlobFilter::getOutputFilter() const
{
    return &m_impl->nmsProc1;
}

int BlobFilter::render(int position)
{
    // Execute internal filter chain
    getInputFilter()->process(position);
    return 0;
}

int BlobFilter::init(int inW, int inH, unsigned int order, bool prepareForExternalInput)
{
    getInputFilter()->prepare(inW, inH, 0, std::numeric_limits<int>::max(), 0);
    return 0;
}

int BlobFilter::reinit(int inW, int inH, bool prepareForExternalInput)
{
    getInputFilter()->prepare(inW, inH, 0, std::numeric_limits<int>::max(), 0);
    return 0;
}

static cv::Size ogles_gpgpu_size(const ogles_gpgpu::Size2d& src)
{
    return cv::Size(src.width, src.height);
}

cv::Mat BlobFilter::paint()
{
    cv::Mat canvas;

    {
        cv::Mat4b smoothProc1(ogles_gpgpu_size(m_impl->smoothProc1.getOutFrameSize()));
        cv::Mat4b hessianProc1(ogles_gpgpu_size(m_impl->hessianProc1.getOutFrameSize()));

        m_impl->smoothProc1.getResultData(smoothProc1.ptr());
        m_impl->hessianProc1.getResultData(hessianProc1.ptr());

        std::vector<cv::Mat> all{
            smoothProc1,
            hessianProc1
        };

        cv::vconcat(all, canvas);
    }

    return canvas;
}

END_OGLES_GPGPU
