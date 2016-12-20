/*!
  @file   gpu/FlashFilter.cpp
  @author David Hirvonen
  @brief  Illuminate the scene with pulses of LCD light.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_gpu_FlashFilter_h__
#define __drishti_hci_gpu_FlashFilter_h__

#include "ogles_gpgpu/common/proc/gauss_opt.h"
#include "ogles_gpgpu/common/proc/fifo.h"
#include "ogles_gpgpu/common/proc/fir3.h"
#include "ogles_gpgpu/common/proc/median.h"
#include "ogles_gpgpu/common/proc/lowpass.h"

// #### Simple flash filter ####
BEGIN_OGLES_GPGPU

class FlashFilter : public MultiPassProc
{
public:
    
    enum FilterKind
    {
        kCenteredDifference,
        kLaplacian
    };
    
    class Impl;
    
    FlashFilter(FilterKind kind=kLaplacian);
    ~FlashFilter();
    
    virtual ProcInterface* getInputFilter() const;
    virtual ProcInterface* getOutputFilter() const;
    
    /**
     * Return the processor's name.
     */
    virtual const char *getProcName() { return "FlashFilter"; }
    
    virtual int init(int inW, int inH, unsigned int order, bool prepareForExternalInput = false);
    virtual int reinit(int inW, int inH, bool prepareForExternalInput = false);
    virtual int render(int position);
    
    std::shared_ptr<Impl> m_impl;
};

END_OGLES_GPGPU

#endif // __drishti_hci_gpu_FlashFilter_h__
