/*!
  @file   gpu/FlashFilter.cpp
  @author David Hirvonen
  @brief  Illuminate the scene with pulses of LCD light.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef FLASH_FILTER_H
#define FLASH_FILTER_H

#include "ogles_gpgpu/common/proc/gauss_opt.h"
#include "ogles_gpgpu/common/proc/fifo.h"
#include "ogles_gpgpu/common/proc/fir3.h"
#include "ogles_gpgpu/common/proc/median.h"
#include "ogles_gpgpu/common/proc/lowpass.h"

// #### Simple flash filter ####
BEGIN_OGLES_GPGPU

class FlashFilter
{
public:
    FlashFilter();
    ProcInterface *last()
    {
        return &fir3Proc;
    }

    ogles_gpgpu::GaussOptProc smoothProc;
    ogles_gpgpu::LowPassFilterProc lowPassProc;;
    ogles_gpgpu::MedianProc medianProc;
    ogles_gpgpu::FIFOPRoc fifoProc;
    ogles_gpgpu::Fir3Proc fir3Proc;
};

END_OGLES_GPGPU

#endif // FLASH_FILTER_H
