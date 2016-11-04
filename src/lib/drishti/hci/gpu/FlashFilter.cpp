/*!
  @file   gpu/FlashFilter.cpp
  @author David Hirvonen
  @brief  Illuminate the scene with pulses of LCD light.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/gpu/FlashFilter.h"

BEGIN_OGLES_GPGPU

FlashFilter::FlashFilter()
    : smoothProc(3)
    , lowPassProc(0.6)
    , medianProc()
    , fifoProc(3)
    , fir3Proc(false)
{
    fir3Proc.setWeights({-0.25, 0.5, -0.25}); // laplacian:

    smoothProc.add(&lowPassProc);
    lowPassProc.add(&medianProc);
    medianProc.add(&fifoProc);

    //  (newest) RGB....RGB....RGB (oldest)
    fifoProc.addWithDelay(&fir3Proc, 0, 0);
    fifoProc.addWithDelay(&fir3Proc, 1, 1);
    fifoProc.addWithDelay(&fir3Proc, 2, 2);
}

END_OGLES_GPGPU
