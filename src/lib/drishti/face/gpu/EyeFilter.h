/*!
  @file   face/gpu/EyeFilter.h
  @author David Hirvonen
  @brief  Perform temporal filtering on stabilized eye crops.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_gpu_EyeFilter_h__
#define __drishti_face_gpu_EyeFilter_h__

#include "ogles_gpgpu/common/common_includes.h"

BEGIN_OGLES_GPGPU
class GaussOptProc;
class LowPassFilterProc;
class LowPassFilterProc;
class DiffProc;
class FifoProc;
END_OGLES_GPGPU

#include "drishti/face/gpu/FaceStabilizer.h"
#include "drishti/face/gpu/MultiTransformProc.h"
#include "drishti/face/Face.h"
#include "drishti/eye/gpu/EyeWarp.h"

#include "ogles_gpgpu/common/proc/base/multipassproc.h"

#include <opencv2/core.hpp>

#include <memory>

BEGIN_OGLES_GPGPU

class EyeFilter : public ogles_gpgpu::MultiPassProc
{
public:

    class Impl;

    enum Mode
    {
        kNone,
        kLowPass,
        kBandPass
    };

    EyeFilter(const Size2d &sizeOut, Mode mode, float upper=0.5, float lower=0.25, float gain=10.0, float offset=0.f);

    virtual ~EyeFilter();

    void setAutoScaling(bool flag)
    {
        m_doAutoScaling = flag;
    }

    std::array<EyeWarp, 2> &getEyeWarps()
    {
        return m_eyes;
    }
    const std::array<EyeWarp, 2> &getEyeWarps() const
    {
        return m_eyes;
    }

    /**
     * Return the processors name.
     */
    virtual const char *getProcName()
    {
        return "EyeFilter";
    }

    virtual ProcInterface* getInputFilter() const;
    virtual ProcInterface* getOutputFilter() const;
    
    virtual void setOutputSize(float scaleFactor);
    virtual void setOutputSize(int outW, int outH);

    virtual int init(int inW, int inH, unsigned int order, bool prepareForExternalInput = false);
    virtual int reinit(int inW, int inH, bool prepareForExternalInput = false);
    virtual int render(int position);

    void addFace(const drishti::face::FaceModel &face)
    {
        m_faces.push_back(face);
    }

    void dump(std::vector<cv::Mat4b> &images);

    void renderIris();

protected:

    std::vector<drishti::face::FaceModel> m_faces;

    std::array<EyeWarp, 2> m_eyes;

    bool m_doAutoScaling = false;

    Size2d m_sizeOut;

    MultiTransformProc transformProc;

    std::unique_ptr<GaussOptProc> smoothProc; // optional
    std::unique_ptr<LowPassFilterProc> lowPassProc;
    std::unique_ptr<LowPassFilterProc> lowPassProc2;
    std::unique_ptr<DiffProc> diffProc;
    std::unique_ptr<FifoProc> fifoProc; // maintain buffer

    ProcInterface *lastProc = nullptr;
    ProcInterface *firstProc = nullptr;

    float m_upper = 0.5;
    float m_lower = 0.25;
    float m_gain = 10.0;
    float m_offset = 0.5;
    bool m_doSmoothing = false;

    std::shared_ptr<Impl> m_impl;
};

END_OGLES_GPGPU

#endif // __drishti_face_gpu_EyeFilter_h__
