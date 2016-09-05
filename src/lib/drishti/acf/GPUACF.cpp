/*!
  @file   GPUACF.cpp
  @author David Hirvonen
  @brief  Declaration of OpenGL shader optimized Aggregated Channel Feature computation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <iostream>
#include <chrono>

#include "drishti/acf/GPUACF.h"
#include "drishti/core/convert.h"
#include "drishti/core/Logger.h"
#include "drishti/core/Parallel.h"
#include "ogles_gpgpu/common/gl/memtransfer_optimized.h"

#include <sys/types.h> // for umask()
#include <sys/stat.h>  // -/-

#include <opencv2/highgui/highgui.hpp>

#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#else
#  define TEXTURE_FORMAT GL_BGRA
#endif

#define DO_INLINE_MERGE 0

BEGIN_OGLES_GPGPU

// { 1280 x 960 } x 0.25 => 320x240

ACF::ACF(void *glContext, const Size2d &size, const std::vector<Size2d> &scales, int grayWidth, int flowWidth, bool debug)
    : VideoSource(glContext)
    , m_size(size)
    , m_debug(debug)
    , m_doGray(grayWidth > 0)
    , m_grayscaleScale(float(grayWidth)/float(size.width))
    , rgbSmoothProc(2.0)
    , reduceRgbSmoothProc()
    , rgb2luvProc()
    , pyramidProc(scales)
    , smoothProc(1)
    , reduceLuvProc()
    , gradProc(1.0)
    , reduceGradProc()
    , normProc(7, true, 0.005)
    , gradHistProcA(6, 0, 1.f)
    , gradHistProcB(6, 4, 1.f)
    , gradHistProcASmooth(3.0)
    , gradHistProcBSmooth(3.0)
    , reduceGradHistProcASmooth(1.0)
    , reduceGradHistProcBSmooth(1.0)

      // Debugging:
    , normProcOut(0.33)
    , gradProcOut(1.0)
    , gradHistProcAOut(1.0f)
    , gradHistProcBOut(1.0f)

      // Optical flow:
    , flow(0.004, 1.0, false)
    , m_doFlow(flowWidth > 0)
    , m_flowScale(float(flowWidth)/float(size.width)) // TODO: not when using pyramid

{
    // TODO: add rescale to width preserving aspect ratio

    // Note: The pyramid step itself performs reductions:

    // Reduce base LUV image to highest resolution used in pyramid:
    rgb2luvProc.setOutputSize(scales[0].width, scales[0].height);

    if(m_doGray)
    {
        Size2d graySize(grayWidth, int(m_grayscaleScale * size.height + 0.5f));
        reduceRgbSmoothProc.setOutputSize(graySize.width, graySize.height);
        rgbSmoothProc.add(&reduceRgbSmoothProc); // ### OUTPUT ###
    }

    if(m_doFlow) // use the same scale as corners:
    {
        // Compute scale relative to bottom pyramid level in pyramidProc()
        float pyramidToFlow = float(flowWidth) / scales[0].width;

        // ((( Optical flow )))
        pyramidProc.add(&flow);
        flow.setOutputSize(pyramidToFlow);
    }

    reduceGradProc.setOutputSize(0.25);
    reduceLuvProc.setOutputSize(0.25);
    reduceGradHistProcASmooth.setOutputSize(0.25);
    reduceGradHistProcBSmooth.setOutputSize(0.25);

#if GPU_ACF_TRANSPOSE
    reduceGradProc.setOutputRenderOrientation(RenderOrientationDiagonal);
    reduceLuvProc.setOutputRenderOrientation(RenderOrientationDiagonal);
    reduceGradHistProcASmooth.setOutputRenderOrientation(RenderOrientationDiagonal);
    reduceGradHistProcBSmooth.setOutputRenderOrientation(RenderOrientationDiagonal);
#endif

    pyramidProc.setInterpolation(ogles_gpgpu::TransformProc::BICUBIC);

    // ((( video -> smooth(luv) )))
    set(&rotationProc);

    rotationProc.add(&rgbSmoothProc);
    rgbSmoothProc.add(&rgb2luvProc);

    // ((( luv -> pyramid(luv) )))
    rgb2luvProc.add(&pyramidProc);

    // ((( pyramid(luv) -> smooth(pyramid(luv)) )))
    pyramidProc.add(&smoothProc);

    // ((( smooth(pyramid(luv)) -> {luv_out, MOXY} )))
    smoothProc.add(&reduceLuvProc); // output 1/4 LUV
    smoothProc.add(&gradProc);      // MOXY

    // ((( MOXY -> norm(M) ))
    gradProc.add(&normProc);        // norm(M)OX.

    // ((( norm(M) -> {histA, histB} )))
    normProc.add(&reduceGradProc);
    normProc.add(&gradHistProcA);
    normProc.add(&gradHistProcB);

    // ((( histA -> smooth(histA) )))
    gradHistProcA.add(&gradHistProcASmooth);
    gradHistProcASmooth.add(&reduceGradHistProcASmooth);

    // ((( histB -> smooth(histB) )))
    gradHistProcB.add(&gradHistProcBSmooth);
    gradHistProcBSmooth.add(&reduceGradHistProcBSmooth);

    // ((( MERGE(luv, grad) )))
    reduceLuvProc.add(&mergeProcLUVG, 0);
    reduceGradProc.add(&mergeProcLUVG, 1);

    if(m_debug)
    {
        gradProc.add(&gradProcOut);  // ### OUTPUT ###
        normProc.add(&normProcOut);  // ### OUTPUT ###
        reduceGradHistProcBSmooth.add(&gradHistProcBOut); // ### OUTPUT ###
        reduceGradHistProcASmooth.add(&gradHistProcAOut); // ### OUTPUT ###
    }
}

ProcInterface * ACF::first()
{
    return &rotationProc;
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

// TODO: more exact mechanism for resized rois from ogles_gpgpu filters:
std::vector<cv::Mat> ACF::getFlowPyramid()
{
    // Build flow pyramid:
    const float scale = float(m_flow.rows) / float(m_crops[0].height);
    const cv::Rect bounds({0,0}, m_flow.size());

    std::vector<cv::Mat> flow(m_crops.size());
    for(int i = 0; i < m_crops.size(); i++)
    {
        const auto &c = m_crops[i]; // 72x894  vs crop[0] = {160x90}
        cv::Rect2f crop(c.x, c.y, c.width, c.height);
        cv::Rect roi(crop.tl() * scale, crop.br() * scale);
        flow[i] = m_flow(roi & bounds);
    }
    //cv::imshow("flow0", flow[0]);
    //cv::imshow("flow1", flow[1]);
    //cv::waitKey(10);

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

// Implement virtual API to toggle detection + tracking:
void ACF::operator()(const FrameInput &frame)
{
    //m_runFlow = (frameIndex % 2);
    //m_runChannels = !runFlow;

    m_runFlow = true;
    m_runFlow = true;

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
    m_hasFlowOutput = false;
    m_hasChannelOutput = false;
    m_hasGrayscaleOutput = false;
}

void ACF::postConfig()
{
    // Obtain the scaled image rois:
    m_crops.clear();
    const auto &rois = pyramidProc.getLevelCrops();
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
    return 10;    // LUVM0123456
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

static void unpackImage(ProcInterface &proc, std::vector<drishti::core::PlaneInfo> &dst)
{
    MemTransfer::FrameDelegate handler = [&](const Size2d &size, const void *pixels, size_t rowStride)
    {
        cv::Mat4b frame(size.height, size.width, (cv::Vec4b*)pixels, rowStride);
        unpack(frame, dst);
    };
    proc.getResultData(handler);
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

cv::Mat ACF::getChannelsImpl()
{

    using drishti::core::unpack;

    if(m_doFlow && !m_hasFlowOutput)
    {
        getImage(flow, m_flow);
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
        int rgba[] = { 2, 1, 0, 3 };
        if(pipeline->getMemTransferObj()->getOutputPixelFormat() == GL_RGBA) // assume BGRA
        {
            std::swap(rgba[0], rgba[2]);
        }

        MatP acf, gray, xy;

        using PlaneInfoVec = std::vector<drishti::core::PlaneInfo>;
        std::vector<std::pair<PlaneInfoVec, ProcInterface *>> planeIndex;

        if(1)
        {
            cv::Size acfSize(mergeProcLUVG.getOutFrameW(), mergeProcLUVG.getOutFrameH());
            acf.create(acfSize, CV_8UC1, getChannelCount(), GPU_ACF_TRANSPOSE);

            // { LUVMp; H0123p; H4567p }
            planeIndex =
            {
                {{{acf[0],rgba[0]}, {acf[1],rgba[1]}, {acf[2],rgba[2]}, {acf[3],rgba[3]}}, &mergeProcLUVG},
                {{{acf[4],rgba[0]}, {acf[5],rgba[1]}, {acf[6],rgba[2]}, {acf[7],rgba[3]}}, &reduceGradHistProcASmooth},
                {{{acf[8],rgba[0]}, {acf[9],rgba[1]}}, &reduceGradHistProcBSmooth}
            };
        }

        if(m_doGray)
        {
            // Here we use the green channel:
            cv::Size graySize(reduceRgbSmoothProc.getOutFrameW(), reduceRgbSmoothProc.getOutFrameH());
            gray.create(graySize, CV_8UC1, 1);
            planeIndex.emplace_back(PlaneInfoVec {{gray[0], rgba[1]}}, &reduceRgbSmoothProc );
        }

        // We can use either the direct MemTransferOptimized acces, or glReadPixels()
        if(dynamic_cast<MemTransferOptimized *>(rgb2luvProc.getMemTransferObj()))
        {
            // TODO: confirm ios texture caches can be queried in parallel
            std::function<void(int)> unpacker = [&](int i)
            {
                planeIndex[i].second->getMemTransferObj()->setOutputPixelFormat(TEXTURE_FORMAT);
                unpackImage(*planeIndex[i].second, planeIndex[i].first);
            };
            drishti::core::ParallelHomogeneousLambda harness(unpacker);

#if OGLES_GPGPU_IOS
            // iOS texture cache can be queried in parallel:
            cv::parallel_for_({0, int(planeIndex.size())}, harness);
#else
            harness({0, int(planeIndex.size())});
#endif
        }
        else
        {
            std::function<void(int)> unpacker = [&](int i)
            {
                planeIndex[i].second->getMemTransferObj()->setOutputPixelFormat(TEXTURE_FORMAT);
                unpack(getImage(*planeIndex[i].second), planeIndex[i].first);
            };
            drishti::core::ParallelHomogeneousLambda harness(unpacker);
            harness({0, int(planeIndex.size())});
        }

        m_channels = acf.base();
        if(m_doGray)
        {
            m_grayscale = gray[0];
        }

        if(0)
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

        if(m_timer)
        {
            m_timer("read end");
        }

        m_hasChannelOutput = true;
    }

    return m_channels;
}

END_OGLES_GPGPU
