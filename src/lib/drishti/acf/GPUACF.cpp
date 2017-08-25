/*!
  @file   GPUACF.cpp
  @author David Hirvonen
  @brief  Declaration of OpenGL shader optimized Aggregated Channel Feature computation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/acf/GPUACF.h"

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

// generic shaders
#include "drishti/graphics/gain.h"
#include "drishti/graphics/swizzle.h"
#include "drishti/graphics/rgb2luv.h"
#include "drishti/graphics/binomial.h"

// acf specific shader
#include "drishti/acf/gpu/swizzle2.h"
#include "drishti/acf/gpu/gradhist.h"
#include "drishti/acf/gpu/triangle.h"

#include "drishti/core/convert.h"
#include "drishti/core/timing.h"
#include "drishti/core/Logger.h"
#include "drishti/core/Parallel.h"
#include "drishti/core/make_unique.h"
#include "ogles_gpgpu/common/gl/memtransfer_optimized.h"

#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <chrono>
#include <array>

#include <sys/types.h> // for umask()
#include <sys/stat.h>  // -/-

// clang-format off
#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#  define TEXTURE_FORMAT_IS_RGBA 1
#else
#  define TEXTURE_FORMAT GL_BGRA
#  define TEXTURE_FORMAT_IS_RGBA 0
#endif
// clang-format on

#define GPU_ACF_DEBUG_CHANNELS 0

#define DO_INLINE_MERGE 0

BEGIN_OGLES_GPGPU

struct ACF::Impl
{
    Impl(void* glContext, const Size2d& size, const SizeVec& scales, FeatureKind kind, int grayWidth, int flowWidth, bool debug)
        : m_featureKind(kind)
        , m_size(size)
        , m_debug(debug)
        , m_doGray(grayWidth > 0)
        , m_grayscaleScale(float(grayWidth) / float(size.width))
        , m_doFlow(flowWidth > 0)
        , m_flowScale(float(flowWidth) / float(size.width)) // TODO: not when using pyramid
    {
        initACF(scales, kind, debug);

        if (m_doGray)
        {
            Size2d graySize(grayWidth, int(m_grayscaleScale * size.height + 0.5f));
            reduceRgbSmoothProc = drishti::core::make_unique<ogles_gpgpu::NoopProc>();
            reduceRgbSmoothProc->setOutputSize(graySize.width, graySize.height);
            rgbSmoothProc->add(reduceRgbSmoothProc.get()); // ### OUTPUT ###
        }

        if (m_doFlow) // use the same scale as corners:
        {
            // Compute scale relative to bottom pyramid level in pyramidProc()
            float pyramidToFlow = float(flowWidth) / scales[0].width;

            // ((( Optical flow )))
            flow = drishti::core::make_unique<ogles_gpgpu::FlowOptPipeline>(0.004, 1.0, false);
            pyramidProc->add(flow.get());
            flow->setOutputSize(pyramidToFlow);

#if TEXTURE_FORMAT_IS_RGBA
            flowBgra = drishti::core::make_unique<ogles_gpgpu::SwizzleProc>();
            flow->add(flowBgra.get());
            flowBgraInterface = flowBgra.get();
#else
            flowBgraInterface = flow.get();
#endif
        }

        if (m_doLuvTransfer)
        {
            // Add transposed Luv output for CPU processing (optional)
            luvTransposeOut->setOutputRenderOrientation(RenderOrientationDiagonal);
            rgb2luvProc->add(luvTransposeOut.get());
        }
    }

    void initACF(const SizeVec& scales, FeatureKind kind, bool debug)
    {
        rotationProc = drishti::core::make_unique<ogles_gpgpu::NoopProc>();
        rgbSmoothProc = drishti::core::make_unique<ogles_gpgpu::GaussOptProc>(2.0f);
        rgb2luvProc = drishti::core::make_unique<ogles_gpgpu::Rgb2LuvProc>();
        pyramidProc = drishti::core::make_unique<ogles_gpgpu::PyramidProc>(scales);
        smoothProc = drishti::core::make_unique<ogles_gpgpu::GaussOptProc>(1);
        reduceLuvProc = drishti::core::make_unique<ogles_gpgpu::NoopProc>();
        gradProc = drishti::core::make_unique<ogles_gpgpu::GradProc>(1.0f);
        reduceGradProc = drishti::core::make_unique<ogles_gpgpu::NoopProc>();
        normProc = drishti::core::make_unique<ogles_gpgpu::GaussOptProc>(7, true, 0.005f);
        gradHistProcA = drishti::core::make_unique<ogles_gpgpu::GradHistProc>(6, 0, 1.f);
        gradHistProcB = drishti::core::make_unique<ogles_gpgpu::GradHistProc>(6, 4, 1.f);
        gradHistProcASmooth = drishti::core::make_unique<ogles_gpgpu::GaussOptProc>(3.0f);
        gradHistProcBSmooth = drishti::core::make_unique<ogles_gpgpu::GaussOptProc>(3.0f);
        reduceGradHistProcASmooth = drishti::core::make_unique<ogles_gpgpu::NoopProc>(1.0f);
        reduceGradHistProcBSmooth = drishti::core::make_unique<ogles_gpgpu::NoopProc>(1.0f);

        // Reduce base LUV image to highest resolution used in pyramid:
        rgb2luvProc->setOutputSize(scales[0].width, scales[0].height);

        reduceGradProc->setOutputSize(0.25);
        reduceLuvProc->setOutputSize(0.25);
        reduceGradHistProcASmooth->setOutputSize(0.25);
        reduceGradHistProcBSmooth->setOutputSize(0.25);

#if GPU_ACF_TRANSPOSE
        reduceGradProc->setOutputRenderOrientation(RenderOrientationDiagonal);
        reduceLuvProc->setOutputRenderOrientation(RenderOrientationDiagonal);
        reduceGradHistProcASmooth->setOutputRenderOrientation(RenderOrientationDiagonal);
        reduceGradHistProcBSmooth->setOutputRenderOrientation(RenderOrientationDiagonal);
#endif

        pyramidProc->setInterpolation(ogles_gpgpu::TransformProc::BICUBIC);

        rotationProc->add(rgbSmoothProc.get());
        rgbSmoothProc->add(rgb2luvProc.get());

        // ((( luv -> pyramid(luv) )))
        rgb2luvProc->add(pyramidProc.get());

        // ((( pyramid(luv) -> smooth(pyramid(luv)) )))
        pyramidProc->add(smoothProc.get());

        // ((( smooth(pyramid(luv)) -> {luv_out, MOXY} )))
        smoothProc->add(reduceLuvProc.get()); // output 1/4 LUV
        smoothProc->add(gradProc.get());      // MOXY

        // ((( MOXY -> norm(M) ))
        gradProc->add(normProc.get()); // norm(M)OX.

        // ((( norm(M) -> {histA, histB} )))
        normProc->add(reduceGradProc.get());
        normProc->add(gradHistProcA.get());
        normProc->add(gradHistProcB.get());

        // ((( histA -> smooth(histA) )))
        gradHistProcA->add(gradHistProcASmooth.get());
        gradHistProcASmooth->add(reduceGradHistProcASmooth.get());

        // ((( histB -> smooth(histB) )))
        gradHistProcB->add(gradHistProcBSmooth.get());
        gradHistProcBSmooth->add(reduceGradHistProcBSmooth.get());

        switch (kind)
        {
            case kM012345:

                // This uses two swizzle steps to creaet LG56 output
                // Adding a 3 input texture swizzler might be slightly more efficient.

                // ((( MERGE(luv, grad) )))
                mergeProcLUVG = drishti::core::make_unique<MergeProc>(MergeProc::kSwizzleABC1);
                reduceLuvProc->add(mergeProcLUVG.get(), 0);
                reduceGradProc->add(mergeProcLUVG.get(), 1);

                // ((( MERGE(lg, 56) )))
                mergeProcLG56 = drishti::core::make_unique<MergeProc>(MergeProc::kSwizzleAD12);
                mergeProcLUVG->add(mergeProcLG56.get(), 0);
                reduceGradHistProcBSmooth->add(mergeProcLG56.get(), 1);
                break;

            case kLUVM012345:
                // ((( MERGE(luv, grad) )))
                mergeProcLUVG = drishti::core::make_unique<MergeProc>(MergeProc::kSwizzleABC1);
                reduceLuvProc->add(mergeProcLUVG.get(), 0);
                reduceGradProc->add(mergeProcLUVG.get(), 1);
                break;
            default:
                CV_Assert(false);
        }

        if (debug)
        {
            // #### OUTPUT ###
            normProcOut = drishti::core::make_unique<ogles_gpgpu::NoopProc>(0.33f);
            gradProcOut = drishti::core::make_unique<ogles_gpgpu::NoopProc>(1.0f);
            gradHistProcAOut = drishti::core::make_unique<ogles_gpgpu::NoopProc>(1.0f);
            gradHistProcBOut = drishti::core::make_unique<ogles_gpgpu::NoopProc>(1.0f);

            gradProc->add(gradProcOut.get());                       // ### OUTPUT ###
            normProc->add(normProcOut.get());                       // ### OUTPUT ###
            reduceGradHistProcBSmooth->add(gradHistProcBOut.get()); // ### OUTPUT ###
            reduceGradHistProcASmooth->add(gradHistProcAOut.get()); // ### OUTPUT ###
        }
    }

    // This provides a map for unpacking/swizzling OpenGL textures (i.e., RGBA or BGRA) to user
    // memory using NEON optimized instructions.
    ACF::ChannelSpecification getACFChannelSpecification(MatP& acf) const
    {
        // clang-format on
        const auto& rgba = m_rgba;
        switch (m_featureKind)
        {
            case kLUVM012345:
                // 10 : { LUVMp; H0123p; H4567p } requires 3 textures
                return ACF::ChannelSpecification
            {
                {{{acf[0],rgba[0]},{acf[1],rgba[1]},{acf[2],rgba[2]},{acf[3],rgba[3]}},mergeProcLUVG.get()},
                {{{acf[4],rgba[0]},{acf[5],rgba[1]},{acf[6],rgba[2]},{acf[7],rgba[3]}},reduceGradHistProcASmooth.get()},
                {{{acf[8],rgba[0]},{acf[9],rgba[1]}}, reduceGradHistProcBSmooth.get()}
            };
                
            case kM012345:
                // 7: { Mp; H0123p; H4567p } requires only 2 textures
                return ACF::ChannelSpecification
            {
                {{{acf[0],rgba[1]},{acf[5],rgba[2]},{acf[6],rgba[3]}}, mergeProcLG56.get()},
                {{{acf[1],rgba[0]},{acf[2],rgba[1]},{acf[3],rgba[2]},{acf[4],rgba[3]}}, reduceGradHistProcASmooth.get()}
            };
            default: CV_Assert(false);        
        }
        return ACF::ChannelSpecification();
        // clang-format on
    }

    // ::: MEMBER VARIABLES :::
    FeatureKind m_featureKind = kLUVM012345;
    
    std::array<int, 4> m_rgba = { { 0, 1, 2, 3 } };
    Size2d m_size;
    
    bool m_debug = false;
    
    // Retriev input image:
    bool m_doAcfTransfer = true;
    
    cv::Mat m_luv;
    MatP m_luvPlanar;
    bool m_doLuvTransfer = false;
    bool m_hasLuvOutput = false;
    
    // Grayscale stuff:
    bool m_doGray = false;
    float m_grayscaleScale = 1.0f;
    bool m_hasGrayscaleOutput = false;
    cv::Mat m_grayscale;
    
    std::unique_ptr<ogles_gpgpu::NoopProc> rotationProc; // make sure we have an unmodified upright image
    std::unique_ptr<ogles_gpgpu::GaussOptProc> rgbSmoothProc;
    std::unique_ptr<ogles_gpgpu::NoopProc> reduceRgbSmoothProc; // reduce
    std::unique_ptr<ogles_gpgpu::Rgb2LuvProc> rgb2luvProc;
    std::unique_ptr<ogles_gpgpu::PyramidProc> pyramidProc;
    std::unique_ptr<ogles_gpgpu::GaussOptProc> smoothProc; // (1);
    std::unique_ptr<ogles_gpgpu::NoopProc> reduceLuvProc;
    std::unique_ptr<ogles_gpgpu::GradProc> gradProc; // (1.0);
    std::unique_ptr<ogles_gpgpu::NoopProc> reduceGradProc;
    std::unique_ptr<ogles_gpgpu::GaussOptProc> normProc;              // (5, true, 0.015);
    std::unique_ptr<ogles_gpgpu::GradHistProc> gradHistProcA;         // (6, 0, 1.f);
    std::unique_ptr<ogles_gpgpu::GradHistProc> gradHistProcB;         // (6, 4, 1.f);
    std::unique_ptr<ogles_gpgpu::GaussOptProc> gradHistProcASmooth;   // (1);
    std::unique_ptr<ogles_gpgpu::GaussOptProc> gradHistProcBSmooth;   // (1);
    std::unique_ptr<ogles_gpgpu::NoopProc> reduceGradHistProcASmooth; // (1);
    std::unique_ptr<ogles_gpgpu::NoopProc> reduceGradHistProcBSmooth; // (1);

    // #### OUTPUT ###
    std::unique_ptr<ogles_gpgpu::NoopProc> normProcOut;      //(0.33);
    std::unique_ptr<ogles_gpgpu::NoopProc> gradProcOut;      //(16.0);
    std::unique_ptr<ogles_gpgpu::NoopProc> gradHistProcAOut; //(1.0f);
    std::unique_ptr<ogles_gpgpu::NoopProc> gradHistProcBOut; //(1.0f);
    std::unique_ptr<ogles_gpgpu::NoopProc> luvTransposeOut;  //  transposed LUV output

    // Multi-texture swizzle (one or the other for 8 vs 10 channels)
    std::unique_ptr<ogles_gpgpu::MergeProc> mergeProcLUVG;
    std::unique_ptr<ogles_gpgpu::MergeProc> mergeProcLG56;

    uint64_t frameIndex = 0;

    // Experimental (flow)
    std::unique_ptr<Flow2Pipeline> flow;
    std::unique_ptr<ogles_gpgpu::SwizzleProc> flowBgra; // (optional)
    ogles_gpgpu::ProcInterface* flowBgraInterface = nullptr;

    std::vector<Rect2d> m_crops;

    // Flwo stuff:
    bool m_hasFlowOutput = false;
    bool m_doFlow = false;
    float m_flowScale = 1.0f;
    cv::Mat m_flow;

    // Channel stuff:
    cv::Mat m_channels;
    bool m_hasChannelOutput = false;

    bool m_usePBO = false;

    bool needsTextures() const
    {
        bool status = false;
        status |= m_doAcfTransfer && !m_hasChannelOutput;
        status |= m_doGray && !m_hasGrayscaleOutput;
        status |= m_doLuvTransfer && !m_hasLuvOutput;
        status |= m_doFlow && ~m_hasFlowOutput;
        return status;
    }
    
    std::shared_ptr<spdlog::logger> m_logger;
};

// ::::::::::: PUBLIC ::::::::::::::

// { 1280 x 960 } x 0.25 => 320x240
ACF::ACF(void* glContext, const Size2d& size, const SizeVec& scales, FeatureKind kind, int grayWidth, int flowWidth, bool debug)
    : VideoSource(glContext)
{
    impl = drishti::core::make_unique<Impl>(glContext, size, scales, kind, grayWidth, flowWidth, debug);

    // ((( video -> smooth(luv) )))
    set(impl->rotationProc.get());
}

ACF::~ACF()
{
    // Required in Xcode 8.1 to supoprt instantiation of std::shared_ptr<ogles_gppgu::ACF>
    // with forward declares std::unique_ptr<> member variables.
}

void ACF::tryEnablePlatformOptimizations()
{
    Core::tryEnablePlatformOptimizations();
}

void ACF::setLogger(std::shared_ptr<spdlog::logger>& logger)
{
    impl->m_logger = logger;
}

bool ACF::getChannelStatus()
{
    return impl->m_hasChannelOutput;
}

bool ACF::getFlowStatus()
{
    return impl->m_doFlow;
}

void ACF::setDoLuvTransfer(bool flag)
{
    impl->m_doLuvTransfer = flag;
}

void ACF::setDoAcfTrasfer(bool flag)
{
    impl->m_doAcfTransfer = flag;
}

void ACF::setUsePBO(bool flag)
{
    impl->m_usePBO = flag;
}

bool ACF::getUsePBO() const
{
    return impl->m_usePBO;
}

// ACF base resolution to Grayscale image
float ACF::getGrayscaleScale() const
{
    return impl->m_grayscaleScale;
}

// Scale of flow wrt inputimage
float ACF::getFlowScale() const
{
    return impl->m_flowScale;
}

Flow2Pipeline* ACF::getFlowProc()
{
    return impl->flow.get();
}
const std::array<int, 4>& ACF::getChannelOrder()
{
    return impl->m_rgba;
}

ProcInterface* ACF::first()
{
    return impl->rotationProc.get();
}

ProcInterface* ACF::getRgbSmoothProc()
{
    return dynamic_cast<ProcInterface*>(impl->rgbSmoothProc.get());
}

void ACF::connect(std::shared_ptr<spdlog::logger>& logger)
{
    impl->m_logger = logger;
}

void ACF::setRotation(int degrees)
{
    first()->setOutputRenderOrientation(ogles_gpgpu::degreesToOrientation(degrees));
}

const cv::Mat& ACF::getGrayscale()
{
    assert(impl->m_doGray);
    return impl->m_grayscale;
}

std::vector<cv::Mat> ACF::getFlowPyramid()
{
    // Build flow pyramid:
    const float scale = float(impl->m_flow.rows) / float(impl->m_crops[0].height);
    const cv::Rect bounds({ 0, 0 }, impl->m_flow.size());

    std::vector<cv::Mat> flow(impl->m_crops.size());
    for (int i = 0; i < impl->m_crops.size(); i++)
    {
        const auto& c = impl->m_crops[i];
        cv::Rect2f crop(c.x, c.y, c.width, c.height);
        cv::Rect roi(crop.tl() * scale, crop.br() * scale);
        flow[i] = impl->m_flow(roi & bounds);
    }

    return flow;
}

/*
 * Texture swizzling will be used to ensure BGRA format on texture read.
 */

const cv::Mat& ACF::getFlow()
{
    return impl->m_flow;
}

void ACF::initLuvTransposeOutput()
{
    // Add transposed Luv output for CPU processing (optional)
    impl->luvTransposeOut = drishti::core::make_unique<ogles_gpgpu::NoopProc>();
    impl->luvTransposeOut->setOutputRenderOrientation(RenderOrientationDiagonal);
    impl->rgb2luvProc->add(impl->luvTransposeOut.get());
}

void ACF::operator()(const Size2d& size, void* pixelBuffer, bool useRawPixels, GLuint inputTexture, GLenum inputPixFormat)
{
    FrameInput frame(size, pixelBuffer, useRawPixels, inputTexture, inputPixFormat);
    return (*this)(frame);
}

// Implement virtual API to toggle detection + tracking:
void ACF::operator()(const FrameInput& frame)
{
    // Inial pipeline filters:
    // this -> rotationProc -> rgbSmoothProc -> rgb2luvProc -> pyramidProc
    bool needsPyramid = (impl->m_doFlow || impl->m_doAcfTransfer);
    bool needsLuv = (needsPyramid | impl->m_doLuvTransfer);

    // Initial LUV transpose operation (upright image):
    if (impl->m_doLuvTransfer & !impl->luvTransposeOut.get())
    {
        initLuvTransposeOutput();
    }

    if (impl->rgb2luvProc.get())
    {
        impl->rgb2luvProc->setActive(needsLuv);
    }

    if (impl->pyramidProc.get())
    {
        impl->pyramidProc->setActive(needsPyramid);
    }

    if (impl->flow.get())
    {
        impl->flow->setActive(impl->m_doFlow);
    }

    // smoothProc is the highest level unique ACF processing filter:
    if (impl->smoothProc.get())
    {
        impl->smoothProc->setActive(impl->m_doAcfTransfer);
    }

    impl->frameIndex++;

    VideoSource::operator()(frame); // call main method

    // Start the transfer asynchronously here:
    if(impl->m_usePBO)
    {
        beginTransfer();
    }
}

void ACF::beginTransfer()
{
    if (impl->m_doAcfTransfer)
    {
        switch (impl->m_featureKind)
        {
            case kLUVM012345:
            {
                impl->mergeProcLUVG->getResultData(nullptr);
                impl->reduceGradHistProcASmooth->getResultData(nullptr);
                impl->reduceGradHistProcBSmooth->getResultData(nullptr);
            }
            break;
                
            case kM012345:
            {
                impl->mergeProcLG56->getResultData(nullptr);
                impl->reduceGradHistProcASmooth->getResultData(nullptr);
            }
            break;
        }
    }

    if (impl->m_doGray)
    {
        impl->reduceRgbSmoothProc->getResultData(nullptr);
    }

    if (impl->m_doLuvTransfer)
    {
        impl->luvTransposeOut->getResultData(nullptr);
    }
        
    if (impl->m_doFlow)
    {
        impl->flowBgraInterface->getResultData(nullptr);
    }
}

void ACF::preConfig()
{
    impl->m_hasLuvOutput = false;
    impl->m_hasFlowOutput = false;
    impl->m_hasChannelOutput = false;
    impl->m_hasGrayscaleOutput = false;
}

void ACF::postConfig()
{
    // Obtain the scaled image rois:
    impl->m_crops.clear();
    const auto& rois = impl->pyramidProc->getLevelCrops();
    for (auto& r : rois)
    {
        // TODO: check rounding error (add clipping)?
        impl->m_crops.emplace_back(r.x >> 2, r.y >> 2, r.width >> 2, r.height >> 2);
    }
}

cv::Mat ACF::getImage(ProcInterface& proc, cv::Mat& frame)
{
    if (dynamic_cast<MemTransferOptimized*>(proc.getMemTransferObj()))
    {
        MemTransfer::FrameDelegate delegate = [&](const Size2d& size, const void* pixels, size_t bytesPerRow) {
            frame = cv::Mat(size.height, size.width, CV_8UC4, (void*)pixels, bytesPerRow).clone();
        };
        proc.getResultData(delegate);
    }
    else
    {
        frame.create(proc.getOutFrameH(), proc.getOutFrameW(), CV_8UC4); // noop if preallocated
        proc.getResultData(frame.ptr());
    }
    return frame;
}

cv::Mat ACF::getImage(ProcInterface& proc)
{
    cv::Mat frame;
    return getImage(proc, frame);
}

bool ACF::processImage(ProcInterface& proc, MemTransfer::FrameDelegate& delegate)
{
    bool status = false;
    MemTransfer* pTransfer = proc.getMemTransferObj();
    if (dynamic_cast<MemTransferOptimized*>(pTransfer))
    {
        proc.getResultData(delegate);
        status = true;
    }
    return status;
}

// size_t bytesPerRow = proc.getMemTransferObj()->bytesPerRow();
// cv::Mat result(proc.getOutFrameH(), proc.getOutFrameW(), CV_8UC4);
// proc.getResultData(result.ptr());
// return result;

int ACF::getChannelCount() const
{
    switch (impl->m_featureKind)
    {
        case kM012345:
            return 7;
        case kLUVM012345:
            return 10;
        default:
            return 0;
    }
}

std::vector<std::vector<Rect2d>> ACF::getCropRegions() const
{
    // CReate array of channel rois for each pyramid level
    size_t levelCount = impl->m_crops.size();
    std::vector<std::vector<Rect2d>> crops(levelCount);
    for (size_t i = 0; i < levelCount; i++)
    {
        crops[i] = getChannelCropRegions(int(i));
    }
    return crops;
}

// Copy the parameters from a reference pyramid
void ACF::fill(drishti::acf::Detector::Pyramid& Pout, const drishti::acf::Detector::Pyramid& Pin)
{
    Pout.pPyramid = Pin.pPyramid;
    Pout.nTypes = Pin.nTypes;
    Pout.nScales = Pin.nScales;
    Pout.info = Pin.info;
    Pout.lambdas = Pin.lambdas;
    Pout.scales = Pin.scales;
    Pout.scaleshw = Pin.scaleshw;

    auto crops = getCropRegions();
    assert(crops.size() > 1);

    Pout.rois.resize(crops.size());
    for (int i = 0; i < crops.size(); i++)
    {
        Pout.rois[i].resize(crops[i].size());
        for (int j = 0; j < crops[i].size(); j++)
        {
            const auto& r = crops[i][j];
            Pout.rois[i][j] = cv::Rect(r.x, r.y, r.width, r.height);
        }
    }
    fill(Pout);
}

// Channels crops are vertically concatenated in the master image:
std::vector<Rect2d> ACF::getChannelCropRegions(int level) const
{
    assert(level < impl->m_crops.size());

#if GPU_ACF_TRANSPOSE
    int step = impl->m_crops[0].height;
    Rect2d roi = impl->m_crops[level];
    std::swap(roi.x, roi.y);
    std::swap(roi.width, roi.height);
    std::vector<Rect2d> crops(getChannelCount(), roi);
    for (int i = 1; i < getChannelCount(); i++)
    {
        crops[i].x += (step * i);
    }
#else
    std::vector<Rect2d> crops(getChannelCount(), impl->m_crops[level]);
    for (int i = 1; i < getChannelCount(); i++)
    {
        crops[i].y += (impl->m_crops[0].height * i);
    }
#endif
    return crops;
}

void ACF::prepare()
{
}

using array_type = drishti::acf::Detector::Pyramid::array_type;

// Fill
void ACF::fill(drishti::acf::Detector::Pyramid& pyramid)
{
    auto acf = getChannels();

    // Build ACF input:
    const auto regions = getCropRegions();
    const int levelCount = static_cast<int>(regions.size());
    const int channelCount = static_cast<int>(regions.front().size());

    pyramid.nScales = int(levelCount);

    // Create multiresolution representation:
    auto& data = pyramid.data;
    data.resize(levelCount);

    for (int i = 0; i < levelCount; i++)
    {
        data[i].resize(1);
        auto& channels = data[i][0];
        channels.base() = acf;
        channels.resize(int(channelCount));
        for (int j = 0; j < channelCount; j++)
        {
            const auto& roi = regions[i][j];
            channels[j] = acf({ roi.x, roi.y, roi.width, roi.height });
        }
    }
}

static void unpackImage(const cv::Mat4b& frame, std::vector<drishti::core::PlaneInfo>& dst)
{
    switch (dst.front().plane.type())
    {
        case CV_8UC4:
            dst.front().plane = frame.clone(); // deep copy
            break;
        case CV_8UC1:
            drishti::core::unpack(frame, dst);
            break;
        case CV_32FC1:
            drishti::core::convertU8ToF32(frame, dst);
            break;
        default:
            break;
    }
}

static void unpackImage(ProcInterface& proc, std::vector<drishti::core::PlaneInfo>& dst)
{
    MemTransfer::FrameDelegate handler = [&](const Size2d& size, const void* pixels, size_t rowStride) {
        cv::Mat4b frame(size.height, size.width, (cv::Vec4b*)pixels, rowStride);
        switch (dst.front().plane.type())
        {
            case CV_8UC4:
                dst.front().plane = frame.clone(); // deep copy
                break;
            case CV_8UC1:
                drishti::core::unpack(frame, dst);
                break;
            case CV_32FC1:
                drishti::core::convertU8ToF32(frame, dst);
                break;
            default:
                break;
        }
    };
    proc.getResultData(handler);
}

// NOTE: GPUACF::getLuvPlanar(), provides a direct/optimized alternative to
// the following CV_8UC4 access and conversion:

// const auto &LUVA = impl->m_acf->getLuv(); // BGRA
// cv::Mat LUV, LUVf;
// cv::cvtColor(LUVA, LUV, cv::COLOR_BGRA2RGB);
// LUV.convertTo(LUVf, CV_32FC3, 1.0/255.0);
// MatP LUVp(LUVf.t());

const MatP& ACF::getLuvPlanar()
{
    CV_Assert(impl->m_hasLuvOutput);
    return impl->m_luvPlanar;
}

const cv::Mat& ACF::getLuv()
{
    impl->m_luv = getImage(*impl->rgb2luvProc);
    return impl->m_luv;
}

cv::Mat ACF::getChannels()
{
    // This needs to be done after full pipeline execution, but before
    // the channels are retrieved.
    impl->m_rgba = initChannelOrder();

    cv::Mat result = getChannelsImpl();

    return result;
}

// This provides a map for unpacking/swizzling OpenGL textures (i.e., RGBA or BGRA) to user
// memory using NEON optimized instructions.
ACF::ChannelSpecification ACF::getACFChannelSpecification(MatP& acf) const
{
    return impl->getACFChannelSpecification(acf);
}

void ACF::release()
{
    impl->m_grayscale.release();
    impl->m_channels.release();
    impl->m_flow.release();
}

std::array<int, 4> ACF::initChannelOrder()
{
    // Default GL_BGRA so we can use GL_RGBA for comparisons
    // since GL_BGRA is undefined on Android
    std::array<int, 4> rgba = { { 2, 1, 0, 3 } };
    if (pipeline->getMemTransferObj()->getOutputPixelFormat() == GL_RGBA) // assume BGRA
    {
        std::swap(rgba[0], rgba[2]);
    }

    return rgba;
}

cv::Mat ACF::getChannelsImpl()
{
    using drishti::core::unpack;

    // clang-format off
    std::stringstream ss;
    const auto tag = DRISHTI_LOCATION_SIMPLE;        
    drishti::core::ScopeTimeLogger scopeTimeLogger = [&](double elapsed)
    {
        if (impl->m_logger)
        {
            impl->m_logger->info("TIMING:{}:{};total={}", tag, ss.str(), elapsed);
        }
    }; // clang-format on
    
    if (impl->needsTextures())
    {
        { // Create a scope for glFlush() timing
            drishti::core::ScopeTimeLogger glFinishTimer = [&](double t) { ss << "glFlush=" << t << ";"; };
            if (auto pTransfer = dynamic_cast<MemTransferOptimized*>(impl->rgb2luvProc->getMemTransferObj()))
            {
                pTransfer->flush();
            }
            else
            {
                glFlush();
            }
        }

        prepare();

        if (m_timer)
        {
            m_timer("read begin");
        }

        ACF::ChannelSpecification planeIndex;
        const auto& rgba = impl->m_rgba; // alias
        
        cv::Mat flow;
        MatP acf, gray, luv;

        if (impl->m_doAcfTransfer)
        {
            const auto acfSize = impl->reduceGradHistProcASmooth->getOutFrameSize();
            acf.create({ acfSize.width, acfSize.height }, CV_8UC1, getChannelCount(), GPU_ACF_TRANSPOSE);
            planeIndex = getACFChannelSpecification(acf);
        }

        if (impl->m_doGray)
        {
            // Here we use the green channel:
            const auto graySize = impl->reduceRgbSmoothProc->getOutFrameSize();
            gray.create({ graySize.width, graySize.height }, CV_8UC1, 1);
            PlaneInfoVec grayInfo{ { gray[0], rgba[0] } };
            planeIndex.emplace_back(grayInfo, impl->reduceRgbSmoothProc.get());
        }

        if (impl->m_doLuvTransfer)
        {
            const float alpha = 1.0f / 255.0f;
            const auto luvSize = impl->luvTransposeOut->getOutFrameSize();
            luv.create({ luvSize.width, luvSize.height }, CV_32FC1, 3);
            PlaneInfoVec luvInfo{ { luv[0], rgba[0], alpha }, { luv[1], rgba[1], alpha }, { luv[2], rgba[2], alpha } };
            planeIndex.emplace_back(luvInfo, impl->luvTransposeOut.get());
        }
        
        if (impl->m_doFlow)
        {
            const auto flowSize = impl->flowBgraInterface->getOutFrameSize();
            flow.create(flowSize.height, flowSize.width, CV_8UC4);
            PlaneInfoVec flowInfo{ { flow } };
            planeIndex.emplace_back(flowInfo, impl->flowBgraInterface);
        }

        // We can use either the direct MemTransferOptimized access, or glReadPixels()
        if (dynamic_cast<MemTransferOptimized*>(impl->rgb2luvProc->getMemTransferObj()))
        {
            drishti::core::ScopeTimeLogger unpackTimer = [&](double t) { ss << "unpack=" << t << ";"; };

            // TODO: confirm in documentation that ios texture caches can be queried in parallel
            // Experimentally this seems to be the case.
            // clang-format off
            drishti::core::ParallelHomogeneousLambda harness = [&](int i)
            {
                planeIndex[i].second->getMemTransferObj()->setOutputPixelFormat(TEXTURE_FORMAT);
                unpackImage(*planeIndex[i].second, planeIndex[i].first);
            }; // clang-format on

#if OGLES_GPGPU_IOS
            // iOS texture cache can be queried in parallel:
            cv::parallel_for_({ 0, int(planeIndex.size()) }, harness);
#else
            harness({ 0, int(planeIndex.size()) });
#endif
        }
        else
        {
            drishti::core::ScopeTimeLogger unpackTimer = [&](double t) { ss << "unpack=" << t << ";"; };
            // clang-format off
            drishti::core::ParallelHomogeneousLambda harness = [&](int i)
            {
                planeIndex[i].second->getMemTransferObj()->setOutputPixelFormat(TEXTURE_FORMAT);
                unpackImage(getImage(*planeIndex[i].second), planeIndex[i].first);
            }; // clang-format on
            harness({ 0, int(planeIndex.size()) });
        }

        if (impl->m_doAcfTransfer)
        {
            impl->m_channels = acf.base();
            impl->m_hasChannelOutput = true;
        }

        if (impl->m_doGray)
        {
            impl->m_grayscale = gray[0];
            impl->m_hasGrayscaleOutput = true;
        }

        if (impl->m_doLuvTransfer)
        {
            impl->m_luvPlanar = luv;
            impl->m_hasLuvOutput = true;
        }
        
        if (impl->m_doFlow)
        {
            impl->m_flow = flow;
            impl->m_hasFlowOutput = true;
        }

        if (m_timer)
        {
            m_timer("read end");
        }
    }

    return impl->m_channels;
}

ACF::FeatureKind getFeatureKind(const drishti::acf::Detector::Options::Pyramid::Chns& chns)
{
    const auto& pColor = chns.pColor.get();
    const auto& pGradMag = chns.pGradMag.get();
    const auto& pGradHist = chns.pGradHist.get();

    // Currently all supported GPU ACF channel types have trailing M012345
    if (!(pGradMag.enabled && pGradHist.enabled && (pGradHist.nOrients == 6)))
    {
        return ACF::kUnknown;
    }

    if (pColor.enabled)
    {
        if (pColor.colorSpace.get() == "luv")
        {
            return ACF::kLUVM012345;
        }
        else
        {
            return ACF::kUnknown;
        }
    }
    else
    {
        return ACF::kM012345;
    }

    return ACF::kUnknown; // compiler warning
}

END_OGLES_GPGPU
