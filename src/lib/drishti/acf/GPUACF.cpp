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

#include "drishti/acf/gpu/gain.h"
#include "drishti/acf/gpu/swizzle2.h"
#include "drishti/acf/gpu/gradhist.h"
#include "drishti/acf/gpu/rgb2luv.h"
#include "drishti/acf/gpu/binomial.h"
#include "drishti/acf/gpu/triangle.h"

#include "drishti/core/convert.h"
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

#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#else
#  define TEXTURE_FORMAT GL_BGRA
#endif

#define GPU_ACF_DEBUG_CHANNELS 0

#define DO_INLINE_MERGE 0

BEGIN_OGLES_GPGPU

// { 1280 x 960 } x 0.25 => 320x240
ACF::ACF(void *glContext, const Size2d &size, const SizeVec &scales, FeatureKind kind, int grayWidth, int flowWidth, bool debug)
    : VideoSource(glContext)
    , m_featureKind(kind)
    , m_size(size)
    , m_debug(debug)
    , m_doGray(grayWidth > 0)
    , m_grayscaleScale(float(grayWidth)/float(size.width))
    , m_doFlow(flowWidth > 0)
    , m_flowScale(float(flowWidth)/float(size.width)) // TODO: not when using pyramid

{
    initACF(scales, kind, debug);
  
    if(m_doGray)
    {
        Size2d graySize(grayWidth, int(m_grayscaleScale * size.height + 0.5f));
        reduceRgbSmoothProc = drishti::core::make_unique<ogles_gpgpu::NoopProc>();
        reduceRgbSmoothProc->setOutputSize(graySize.width, graySize.height);
        rgbSmoothProc->add(reduceRgbSmoothProc.get()); // ### OUTPUT ###
    }
    
    if(m_doFlow) // use the same scale as corners:
    {
        // Compute scale relative to bottom pyramid level in pyramidProc()
        float pyramidToFlow = float(flowWidth) / scales[0].width;
        
        // ((( Optical flow )))
        flow = drishti::core::make_unique<ogles_gpgpu::FlowOptPipeline>(0.004, 1.0, false);
        pyramidProc->add(flow.get());
        flow->setOutputSize(pyramidToFlow);
    }
    if(m_doLuvTransfer)
    {
        // Add transposed Luv output for CPU processing (optional)
        luvTransposeOut->setOutputRenderOrientation(RenderOrientationDiagonal);
        rgb2luvProc->add(luvTransposeOut.get());
    }
}

ACF::~ACF()
{
    // Required in Xcode 8.1 to supoprt instantiation of std::shared_ptr<ogles_gppgu::ACF>
    // with forward declares std::unique_ptr<> member variables.
}

void ACF::initACF(const SizeVec &scales, FeatureKind kind, bool debug)
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
    
    // ((( video -> smooth(luv) )))
    set(rotationProc.get());
    
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
    gradProc->add(normProc.get());        // norm(M)OX.
    
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
    
    switch(kind)
    {
        case kLM012345:
            
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
    }
    
    if(debug)
    {
        // #### OUTPUT ###
        normProcOut = drishti::core::make_unique<ogles_gpgpu::NoopProc>(0.33f);
        gradProcOut = drishti::core::make_unique<ogles_gpgpu::NoopProc>(1.0f);
        gradHistProcAOut = drishti::core::make_unique<ogles_gpgpu::NoopProc>(1.0f);
        gradHistProcBOut = drishti::core::make_unique<ogles_gpgpu::NoopProc>(1.0f);
        
        gradProc->add(gradProcOut.get());  // ### OUTPUT ###
        normProc->add(normProcOut.get());  // ### OUTPUT ###
        reduceGradHistProcBSmooth->add(gradHistProcBOut.get()); // ### OUTPUT ###
        reduceGradHistProcASmooth->add(gradHistProcAOut.get()); // ### OUTPUT ###
    }
}

ProcInterface * ACF::first()
{
    return rotationProc.get();
}

void ACF::connect(std::shared_ptr<spdlog::logger> &logger)
{
    m_logger = logger;
}

void ACF::setRotation(int degrees)
{
    first()->setOutputRenderOrientation(ogles_gpgpu::degreesToOrientation(degrees));
}

const cv::Mat & ACF::getGrayscale()
{
    assert(m_doGray);
    assert(m_hasChannelOutput && m_crops.size());
    return m_grayscale;
}

std::vector<cv::Mat> ACF::getFlowPyramid()
{
    // Build flow pyramid:
    const float scale = float(m_flow.rows) / float(m_crops[0].height);
    const cv::Rect bounds({0,0}, m_flow.size());

    std::vector<cv::Mat> flow(m_crops.size());
    for(int i = 0; i < m_crops.size(); i++)
    {
        const auto &c = m_crops[i];
        cv::Rect2f crop(c.x, c.y, c.width, c.height);
        cv::Rect roi(crop.tl() * scale, crop.br() * scale);
        flow[i] = m_flow(roi & bounds);
    }

    return flow;
}

const cv::Mat & ACF::getFlow()
{
    return m_flow;
}

void ACF::operator()(const Size2d &size, void* pixelBuffer, bool useRawPixels, GLuint inputTexture, GLenum inputPixFormat)
{
    FrameInput frame(size, pixelBuffer, useRawPixels, inputTexture, inputPixFormat);
    return (*this)(frame);
}

void ACF::initLuvTransposeOutput()
{
    // Add transposed Luv output for CPU processing (optional)
    luvTransposeOut = drishti::core::make_unique<ogles_gpgpu::NoopProc>();
    luvTransposeOut->setOutputRenderOrientation(RenderOrientationDiagonal);
    
    rgb2luvProc->add(luvTransposeOut.get());
}

// Implement virtual API to toggle detection + tracking:
void ACF::operator()(const FrameInput &frame)
{
    if(m_doLuvTransfer & !luvTransposeOut.get())
    {
        initLuvTransposeOutput();
    }
    
    frameIndex++;

    auto tic = std::chrono::system_clock::now();
    VideoSource::operator()(frame); // call main method
    std::chrono::duration<double> elapsedSeconds =  std::chrono::system_clock::now() - tic;
    if(m_logger)
    {
        m_logger->info() << "ACF COMPUTE SECONDS: " << elapsedSeconds.count();
    }
}

void ACF::preConfig()
{
    m_hasLuvOutput = false;
    m_hasFlowOutput = false;
    m_hasChannelOutput = false;
    m_hasGrayscaleOutput = false;
}

void ACF::postConfig()
{
    // Obtain the scaled image rois:
    m_crops.clear();
    const auto &rois = pyramidProc->getLevelCrops();
    for(auto &r : rois)
    {
        m_crops.emplace_back(r.x>>2, r.y>>2, r.width>>2, r.height>>2); // TODO: check rounding error (add clipping)?
    }
}

cv::Mat ACF::getImage(ProcInterface &proc, cv::Mat &frame)
{
    if(dynamic_cast<MemTransferOptimized*>(proc.getMemTransferObj()))
    {
        MemTransfer::FrameDelegate delegate = [&](const Size2d &size, const void *pixels, size_t bytesPerRow)
        {
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

cv::Mat ACF::getImage(ProcInterface &proc)
{
    cv::Mat frame;
    return getImage(proc, frame);
}

bool ACF::processImage(ProcInterface &proc, MemTransfer::FrameDelegate &delegate)
{
    bool status = false;
    MemTransfer *pTransfer = proc.getMemTransferObj();
    if(dynamic_cast<MemTransferOptimized*>(pTransfer))
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
    switch(m_featureKind)
    {
        case kLM012345: return 8;        
        case kLUVM012345: return 10;
    }
}

std::vector<std::vector<Rect2d>> ACF::getCropRegions() const
{
    // CReate array of channel rois for each pyramid level
    size_t levelCount = m_crops.size();
    std::vector<std::vector<Rect2d>> crops(levelCount);
    for(size_t i = 0; i < levelCount; i++)
    {
        crops[i] = getChannelCropRegions(int(i));
    }
    return crops;
}

// Copy the parameters from a reference pyramid
void ACF::fill(drishti::acf::Detector::Pyramid &Pout, const drishti::acf::Detector::Pyramid &Pin)
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
    for(int i = 0; i < crops.size(); i++)
    {
        Pout.rois[i].resize(crops[i].size());
        for(int j = 0; j < crops[i].size(); j++)
        {
            const auto &r = crops[i][j];
            Pout.rois[i][j] = cv::Rect(r.x, r.y, r.width, r.height);
        }
    }
    fill(Pout);
}

// Channels crops are vertically concatenated in the master image:
std::vector<Rect2d> ACF::getChannelCropRegions(int level) const
{
    assert(level < m_crops.size());

#if GPU_ACF_TRANSPOSE
    int step = m_crops[0].height;
    Rect2d roi = m_crops[level];
    std::swap(roi.x, roi.y);
    std::swap(roi.width, roi.height);
    std::vector<Rect2d> crops(getChannelCount(), roi);
    for(int i = 1; i < getChannelCount(); i++)
    {
        crops[i].x += (step * i);
    }
#else
    std::vector<Rect2d> crops(getChannelCount(), m_crops[level]);
    for(int i = 1; i < getChannelCount(); i++)
    {
        crops[i].y += (m_crops[0].height * i);
    }
#endif
    return crops;
}

void ACF::prepare()
{

}

using array_type = drishti::acf::Detector::Pyramid::array_type;

// Fill
void ACF::fill(drishti::acf::Detector::Pyramid &pyramid)
{
    auto acf = getChannels();

    // Build ACF input:
    auto regions = getCropRegions();
    int levelCount = static_cast<int>(regions.size());
    int channelCount = static_cast<int>(regions.front().size());

    pyramid.nScales = int(levelCount);

    // Create multiresolution representation:
    auto &data = pyramid.data;
    data.resize( boost::extents[levelCount][1]);

    for(int i = 0; i < levelCount; i++)
    {
        auto &channels = data[i][0];
        channels.base() = acf;
        channels.resize(int(channelCount));
        for(int j = 0; j < channelCount; j++)
        {
            const auto &roi = regions[i][j];
            channels[j] = acf({roi.x, roi.y, roi.width, roi.height});
        }
    }
}

static void unpackImage(const cv::Mat4b &frame, std::vector<drishti::core::PlaneInfo> &dst)
{
    switch(dst.front().plane.depth())
    {
        case CV_8UC1: drishti::core::unpack(frame, dst); break;
        case CV_32FC1: drishti::core::convertU8ToF32(frame, dst); break;
        default: break;
    }
}

static void unpackImage(ProcInterface &proc, std::vector<drishti::core::PlaneInfo> &dst)
{
    MemTransfer::FrameDelegate handler = [&](const Size2d &size, const void *pixels, size_t rowStride)
    {
        cv::Mat4b frame(size.height, size.width, (cv::Vec4b*)pixels, rowStride);
        switch(dst.front().plane.depth())
        {
            case CV_8UC1: drishti::core::unpack(frame, dst); break;
            case CV_32FC1: drishti::core::convertU8ToF32(frame, dst); break;
            default: break;
        }
    };
    proc.getResultData(handler);
}

// NOTE: GPUACF::getLuvPlanar(), provides a direct/optimized alternative to
// the following CV_8UC4 access and conversion:

// const auto &LUVA = m_acf->getLuv(); // BGRA
// cv::Mat LUV, LUVf;
// cv::cvtColor(LUVA, LUV, cv::COLOR_BGRA2RGB);
// LUV.convertTo(LUVf, CV_32FC3, 1.0/255.0);
// MatP LUVp(LUVf.t());

const MatP& ACF::getLuvPlanar()
{
    CV_Assert(m_hasLuvOutput);
    return m_luvPlanar;
}

const cv::Mat& ACF::getLuv()
{
    m_luv = getImage(*rgb2luvProc);
    return m_luv;
}

cv::Mat ACF::getChannels()
{
    auto tic = std::chrono::system_clock::now();
    
    cv::Mat result = getChannelsImpl();
    std::chrono::duration<double> elapsedSeconds =  std::chrono::system_clock::now() - tic;
    if(m_logger)
    {
        m_logger->info() << "ACF ACCESS SECONDS:" << elapsedSeconds.count();
    }

    return result;
}

// This provides a map for unpacking/swizzling OpenGL textures (i.e., RGBA or BGRA) to user
// memory using NEON optimized instructions.
ACF::ChannelSpecification ACF::getACFChannelSpecification(MatP &acf, const std::array<int,4> &rgba) const
{
    switch(m_featureKind)
    {
        // 10 : { LUVMp; H0123p; H4567p } requires 3 textures
        case kLUVM012345: return ACF::ChannelSpecification
        {
            {{{acf[0],rgba[0]}, {acf[1],rgba[1]}, {acf[2],rgba[2]}, {acf[3],rgba[3]}}, mergeProcLUVG.get()},
            {{{acf[4],rgba[0]}, {acf[5],rgba[1]}, {acf[6],rgba[2]}, {acf[7],rgba[3]}}, reduceGradHistProcASmooth.get()},
            {{{acf[8],rgba[0]}, {acf[9],rgba[1]}}, reduceGradHistProcBSmooth.get()}
        };
            
        // 8: { LMp; H0123p; H4567p } requires only 2 textures
        case kLM012345: return ACF::ChannelSpecification
        {
            {{{acf[0],rgba[0]}, {acf[1],rgba[1]}, {acf[6],rgba[2]}, {acf[7],rgba[3]}}, mergeProcLG56.get()},
            {{{acf[2],rgba[0]}, {acf[3],rgba[1]}, {acf[4],rgba[2]}, {acf[5],rgba[3]}}, reduceGradHistProcASmooth.get()}
        };
    }
}

void ACF::release()
{
    m_grayscale.release();
    m_channels.release();
    m_flow.release();
}

cv::Mat ACF::getChannelsImpl()
{
    using drishti::core::unpack;
    
    glFlush();

    if(m_doFlow && !m_hasFlowOutput)
    {
        getImage(*flow, m_flow);
        m_hasFlowOutput = true;
    }

    if(!m_hasChannelOutput)
    {
        prepare();

        if(m_timer)
        {
            m_timer("read begin");
        }

        // Default GL_BGRA so we can use GL_RGBA for comparisons
        // since GL_BGRA is undefined on Android
        std::array<int,4> rgba = {{ 2, 1, 0, 3 }};
        if(pipeline->getMemTransferObj()->getOutputPixelFormat() == GL_RGBA) // assume BGRA
        {
            std::swap(rgba[0], rgba[2]);
        }
        
        MatP acf, gray, luv;
        const auto acfSize = reduceGradHistProcASmooth->getOutFrameSize();
        acf.create({acfSize.width, acfSize.height}, CV_8UC1, getChannelCount(), GPU_ACF_TRANSPOSE);
        
        auto planeIndex = getACFChannelSpecification(acf, rgba);
    
        if(m_doGray)
        {
            // Here we use the green channel:
            const auto graySize = reduceRgbSmoothProc->getOutFrameSize();
            gray.create({graySize.width, graySize.height}, CV_8UC1, 1);
            PlaneInfoVec grayInfo {{gray[0], rgba[1]}};
            planeIndex.emplace_back(grayInfo, reduceRgbSmoothProc.get());
        }
        
        if(m_doLuvTransfer)
        {
            const float alpha = 1.0f/255.0f;
            const auto luvSize = luvTransposeOut->getOutFrameSize();
            luv.create({luvSize.width, luvSize.height}, CV_32FC1, 3);
            PlaneInfoVec luvInfo {{luv[0],rgba[0],alpha},{luv[1],rgba[1],alpha},{luv[2],rgba[2],alpha}};
            planeIndex.emplace_back(luvInfo, luvTransposeOut.get());
        }

        // We can use either the direct MemTransferOptimized acces, or glReadPixels()
        if(dynamic_cast<MemTransferOptimized *>(rgb2luvProc->getMemTransferObj()))
        {
            // TODO: confirm in documentation that ios texture caches can be queried in parallel
            // Experimentally this seems to be the case.
            drishti::core::ParallelHomogeneousLambda harness = [&](int i)
            {
                planeIndex[i].second->getMemTransferObj()->setOutputPixelFormat(TEXTURE_FORMAT);
                unpackImage(*planeIndex[i].second, planeIndex[i].first);
            };

#if OGLES_GPGPU_IOS
            // iOS texture cache can be queried in parallel:
            cv::parallel_for_({0, int(planeIndex.size())}, harness);
#else
            harness({0, int(planeIndex.size())});
#endif
        }
        else
        {
            drishti::core::ParallelHomogeneousLambda harness = [&](int i)
            {
                planeIndex[i].second->getMemTransferObj()->setOutputPixelFormat(TEXTURE_FORMAT);
                unpackImage(getImage(*planeIndex[i].second), planeIndex[i].first);
            };

            harness({0, int(planeIndex.size())});
        }

        m_channels = acf.base();
        if(m_doGray)
        {
            m_grayscale = gray[0];
            m_hasGrayscaleOutput = true;
        }

        if(m_doLuvTransfer)
        {
            m_luvPlanar = luv;
            m_hasLuvOutput = true;
        }
        
#if GPU_ACF_DEBUG_CHANNELS
        {
            std::string home = getenv("HOME");
#if ANDROID
            m_logger->info() << "ANDROID: " << home;
            umask(777);
            cv::imwrite(home + "/gray.png", gray[0]);
            cv::imwrite(home + "/acf.png", m_channels);
#else
            cv::imwrite(home + "/Documents/gray.png", gray[0]);
            cv::imwrite(home + "/Documents/acf.png", m_channels);
#endif
        }
        
#endif // GPU_ACF_DEBUG_CHANNELS

        if(m_timer)
        {
            m_timer("read end");
        }

        m_hasChannelOutput = true;
    }

    return m_channels;
}

END_OGLES_GPGPU
