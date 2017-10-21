/*! -*-c++-*-
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
class Fir3Proc;
END_OGLES_GPGPU

#include "drishti/face/gpu/FaceStabilizer.h"
#include "drishti/face/gpu/MultiTransformProc.h"
#include "drishti/face/Face.h"
#include "drishti/eye/gpu/EyeWarp.h"

#include "ogles_gpgpu/common/proc/base/multipassproc.h"

#include <opencv2/core.hpp>

#include <memory>
#include <deque>

BEGIN_OGLES_GPGPU

class EyeFilter : public ogles_gpgpu::MultiPassProc
{
public:
    using EyewWarpPair = std::array<drishti::eye::EyeWarp, 2>;
    using EyePair = std::array<drishti::eye::EyeModel, 2>;

    class Impl;

    enum Mode
    {
        kNone,
        kIirLowPass,
        kMean3,
    };

    EyeFilter(const Size2d& sizeOut, Mode mode, float cutoff, int history);

    virtual ~EyeFilter();

    void setAutoScaling(bool flag)
    {
        m_doAutoScaling = flag;
    }

    EyewWarpPair& getEyeWarps()
    {
        return m_eyes;
    }

    const EyewWarpPair& getEyeWarps() const
    {
        return m_eyes;
    }

    /**
     * Return the processors name.
     */
    virtual const char* getProcName()
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

    void addFace(const drishti::face::FaceModel& face)
    {
        m_faces.push_back(face);
    }

    void dump(std::vector<cv::Mat4b>& images, std::vector<EyePair>& eyes, int n, bool getImage);

    void renderIris();

protected:
    int m_history = 3;

    std::vector<drishti::face::FaceModel> m_faces;

    EyewWarpPair m_eyes;
    std::deque<EyewWarpPair> m_eyeHistory;

    bool m_doAutoScaling = false;

    Size2d m_sizeOut;

    MultiTransformProc transformProc;

    std::unique_ptr<FifoProc> fifoProc; // maintain buffer
    std::unique_ptr<LowPassFilterProc> lowPassProc;
    std::unique_ptr<Fir3Proc> mean3Proc;

    ProcInterface* lastProc = nullptr;
    ProcInterface* firstProc = nullptr;

    std::shared_ptr<Impl> m_impl;
};

END_OGLES_GPGPU

#endif // __drishti_face_gpu_EyeFilter_h__
