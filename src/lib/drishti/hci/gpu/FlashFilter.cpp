/*!
  @file   gpu/FlashFilter.cpp
  @author David Hirvonen
  @brief  Illuminate the scene with pulses of LCD light.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/gpu/FlashFilter.h"

#include <limits>

BEGIN_OGLES_GPGPU

class FlashFilter::Impl
{
public:
    Impl(FlashFilter::FilterKind kind)
    : smoothProc(3)
    , medianProc()
    , fifoProc(3)
    , fir3Proc(false)
    {
        switch(kind)
        {
            case FlashFilter::kLaplacian :
                fir3Proc.setWeights({-0.25, 0.5, -0.25});
                break;
            case FlashFilter::kCenteredDifference :
                fir3Proc.setWeights({-0.5, 0.0, +0.5});
                break;
        }
        
        smoothProc.add(&medianProc);
        medianProc.add(&fifoProc);
        
        //  (newest) RGB....RGB....RGB (oldest)
        fifoProc.addWithDelay(&fir3Proc, 0, 0);
        fifoProc.addWithDelay(&fir3Proc, 1, 1);
        fifoProc.addWithDelay(&fir3Proc, 2, 2);
    }
    
    ogles_gpgpu::GaussOptProc smoothProc;
    ogles_gpgpu::MedianProc medianProc;
    ogles_gpgpu::FIFOPRoc fifoProc;
    ogles_gpgpu::Fir3Proc fir3Proc;
};

FlashFilter::FlashFilter(FilterKind kind)
{
    m_impl = std::make_shared<Impl>(kind);
    
    // Add filters to procPasses for state management
    procPasses.push_back(&m_impl->smoothProc);
    procPasses.push_back(&m_impl->medianProc);
    procPasses.push_back(&m_impl->fifoProc);
    procPasses.push_back(&m_impl->fir3Proc);
}

FlashFilter::~FlashFilter()
{
    procPasses.clear();
}

ProcInterface* FlashFilter::getInputFilter() const
{
    return &m_impl->smoothProc;
}

ProcInterface* FlashFilter::getOutputFilter() const
{
    return &m_impl->fir3Proc;
}

int FlashFilter::render(int position)
{
    // Execute internal filter chain
    getInputFilter()->process(position);
    return 0;
}

int FlashFilter::init(int inW, int inH, unsigned int order, bool prepareForExternalInput)
{
    getInputFilter()->prepare(inW, inH, 0, std::numeric_limits<int>::max(), 0);
    return 0;
}

int FlashFilter::reinit(int inW, int inH, bool prepareForExternalInput)
{
    getInputFilter()->prepare(inW, inH, 0, std::numeric_limits<int>::max(), 0);
    return 0;
}

END_OGLES_GPGPU
