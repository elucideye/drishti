/*!
  @file   GPUACF.h
  @author David Hirvonen
  @brief  Implementation of OpenGL shader optimized Aggregated Channel Feature computation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_acf_GPUACF_h__
#define __drishti_acf_GPUACF_h__

#include "drishti/acf/drishti_acf.h"
#include "drishti/acf/ACF.h"

#include "ogles_gpgpu/common/proc/video.h"
#include "ogles_gpgpu/common/proc/grayscale.h"
#include "ogles_gpgpu/common/proc/pyramid.h"
#include "ogles_gpgpu/common/proc/grad.h"
#include "ogles_gpgpu/common/proc/gauss.h"
#include "ogles_gpgpu/common/proc/gauss_opt.h"
#include "ogles_gpgpu/common/proc/transform.h"

#include "ogles_gpgpu/common/proc/tensor.h"
#include "ogles_gpgpu/common/proc/nms.h"
#include "ogles_gpgpu/common/proc/shitomasi.h"
#include "ogles_gpgpu/common/proc/flow.h"

#include "drishti/acf/gpu/gain.h"
#include "drishti/acf/gpu/swizzle2.h"
#include "drishti/acf/gpu/gradhist.h"
#include "drishti/acf/gpu/rgb2luv.h"
#include "drishti/acf/gpu/binomial.h"
#include "drishti/acf/gpu/triangle.h"

// *INDENT-OFF*
namespace spdlog { class logger; }
// *INDENT-ON*

BEGIN_OGLES_GPGPU

// #### GPU #####

// 1) RGB -> LUV
// 2) conv [1 2 1] ; [1 2 1]'
// 3) gradient: mag, orientation, dx, dy

#define GPU_ACF_TRANSPOSE 1

struct ACF : public ogles_gpgpu::VideoSource
{
public:

    using array_type = drishti::acf::Detector::Pyramid::array_type;

    ACF(void *glContext, const Size2d &size, const std::vector<Size2d> &scales, int grayWidth=0, int flowWidth = 0, bool debug=false);
    virtual void preConfig();
    virtual void postConfig();
    static cv::Mat getImage(ProcInterface &proc);
    static cv::Mat getImage(ProcInterface &proc, cv::Mat &frame);

    bool getChannelStatus()
    {
        return m_runChannels;
    }
    bool getFlowStatus()
    {
        return m_doFlow && m_runFlow;
    }

    virtual void operator()(const FrameInput &frame);
    virtual void operator()(const Size2d &size, void* pixelBuffer, bool useRawPixels, GLuint inputTexture=0, GLenum inputPixFormat=DFLT_PIX_FORMAT);

    void connect(std::shared_ptr<spdlog::logger> &logger);

    void setRotation(int degrees);
    
    void setDoLuvTransfer(bool flag)
    {
        m_doLuvTransfer = flag;
    }

    static bool processImage(ProcInterface &proc, MemTransfer::FrameDelegate &delegate);
    int getChannelCount() const; // LUVM0123456
    std::vector<std::vector<Rect2d>> getCropRegions() const;
    std::vector<Rect2d> getChannelCropRegions(int level = 0) const;
    void prepare();
    void fill(drishti::acf::Detector::Pyramid &pyramid);
    void fill(drishti::acf::Detector::Pyramid &Pout, const drishti::acf::Detector::Pyramid &Pin);

    // GPU => CPU for ACF:
    cv::Mat getChannels();

    // GPU => CPU for grayscale:
    const cv::Mat &getGrayscale();

    // ACF base resolution to Grayscale image
    float getGrayscaleScale() const
    {
        return m_grayscaleScale;
    }

    // GPU => CPU for optical flow:
    const cv::Mat &getFlow();

    std::vector<cv::Mat> getFlowPyramid();

    // Scale of flow wrt inputimage
    float getFlowScale() const
    {
        return m_flowScale;
    }

    ProcInterface * first();

//protected:

    // Retrieve Luv image as planar 3 channel CV_32F
    const MatP & getLuvPlanar();
    
    // Retrieve Luv image in packed CV_8UC4 (RGBA) format
    const cv::Mat& getLuv();
    
    cv::Mat getChannelsImpl();

    Size2d m_size;

    bool m_debug = false;
    
    // Retriev input image:

    cv::Mat m_luv;
    MatP m_luvPlanar;
    bool m_doLuvTransfer = false;
    bool m_hasLuvOutput = false;

    // Grayscale stuff:
    bool m_doGray = false;
    float m_grayscaleScale = 1.0f;
    bool m_hasGrayscaleOutput = false;
    cv::Mat m_grayscale;
    
    ogles_gpgpu::NoopProc rotationProc; // make sure we have an unmodified upright image
    ogles_gpgpu::GaussOptProc rgbSmoothProc;
    ogles_gpgpu::NoopProc reduceRgbSmoothProc;  // reduce
    ogles_gpgpu::Rgb2LuvProc rgb2luvProc;
    ogles_gpgpu::PyramidProc pyramidProc;
    ogles_gpgpu::GaussOptProc smoothProc; // (1);
    ogles_gpgpu::NoopProc reduceLuvProc;
    ogles_gpgpu::GradProc gradProc; // (1.0);
    ogles_gpgpu::GaussOptProc gradSmoothProc;
    ogles_gpgpu::NoopProc reduceGradProc;
    ogles_gpgpu::GaussOptProc normProc; // (5, true, 0.015);
    ogles_gpgpu::GradHistProc gradHistProcA; // (6, 0, 1.f);
    ogles_gpgpu::GradHistProc gradHistProcB; // (6, 4, 1.f);
    ogles_gpgpu::GaussOptProc gradHistProcASmooth; // (1);
    ogles_gpgpu::GaussOptProc gradHistProcBSmooth; // (1);
    ogles_gpgpu::NoopProc reduceGradHistProcASmooth; // (1);
    ogles_gpgpu::NoopProc reduceGradHistProcBSmooth; // (1);

    // #### OUTPUT ###
    ogles_gpgpu::NoopProc normProcOut; //(0.33);
    ogles_gpgpu::NoopProc gradProcOut; //(16.0);
    ogles_gpgpu::NoopProc gradHistProcAOut; //(1.0f);
    ogles_gpgpu::NoopProc gradHistProcBOut; //(1.0f);
    
    ogles_gpgpu::NoopProc luvTransposeOut; //  transposed LUV output

    // Multi-texture swizzle
    ogles_gpgpu::MergeProc mergeProcLUVG;

    uint64_t frameIndex = 0;

    // These are temporary overrides
    bool m_runChannels = true;
    bool m_runFlow = true;

    // Experimental (flow)
    FlowOptPipeline flow;

    std::vector<Rect2d> m_crops;

    // Flwo stuff:
    bool m_hasFlowOutput = false;
    bool m_doFlow = false;
    float m_flowScale = 1.0f;
    cv::Mat m_flow;

    // Channel stuff:
    cv::Mat m_channels;
    bool m_hasChannelOutput = false;

    std::shared_ptr<spdlog::logger> m_logger;
};

END_OGLES_GPGPU


#endif // __drishti_acf_GPUACF_h__
