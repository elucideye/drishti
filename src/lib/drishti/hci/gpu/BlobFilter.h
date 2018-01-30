/*! -*-c++-*-
  @file   gpu/BlobFilter.cpp
  @author David Hirvonen
  @brief  Illuminate the scene with pulses of LCD light.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_gpu_BlobFilter_h__
#define __drishti_hci_gpu_BlobFilter_h__

#include "ogles_gpgpu/common/proc/base/multipassproc.h"

#include <opencv2/core.hpp> // for paint()

#include <memory>

// #### Simple blob filter ####
BEGIN_OGLES_GPGPU

class BlobFilter : public MultiPassProc
{
public:
    class Impl;

    BlobFilter();
    ~BlobFilter();

    virtual ProcInterface* getInputFilter() const;
    virtual ProcInterface* getOutputFilter() const;

    ProcInterface* getHessian() const;
    ProcInterface* getHessianPeaks() const;

    /**
     * Return the processor's name.
     */
    virtual const char* getProcName() { return "BlobFilter"; }

    virtual int init(int inW, int inH, unsigned int order, bool prepareForExternalInput = false);
    virtual int reinit(int inW, int inH, bool prepareForExternalInput = false);
    virtual int render(int position = 0);

    cv::Mat paint();

    std::shared_ptr<Impl> m_impl;
};

END_OGLES_GPGPU

#endif // __drishti_hci_gpu_BlobFilter_h__
