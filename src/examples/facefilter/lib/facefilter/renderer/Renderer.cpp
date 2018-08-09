/*! -*-c++-*-
  @file Renderer.cpp
  @brief Implementation of the main native facefilter tracking and rendering manager class.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
*/

#include <facefilter/renderer/Renderer.h>
#include <facefilter/renderer/FaceTrackerTest.h>
#include <facefilter/renderer/FaceTrackerFactoryJson.h>
#include <facefilter/renderer/Context.h> // Application
#include <facefilter/renderer/fill.h> // fill mode logic
#include <facefilter/make_unique.h>

#include <drishti/FaceTracker.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <ogles_gpgpu/common/proc/video.h>
#include <ogles_gpgpu/common/proc/grayscale.h>
#include <ogles_gpgpu/common/proc/disp.h>

#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#else
#  define TEXTURE_FORMAT GL_BGRA
#endif

BEGIN_FACEFILTER_NAMESPACE

struct Renderer::Impl
{
    Impl(
        void *glContext,
        const cv::Size &frameSize,
        GLenum pixelFormat,
        float focalLength,
        int rotation,
        const cv::Size &displaySize,
        Application &application
    )
        : glContext(glContext)
        , frameSize(frameSize)
        , pixelFormat(pixelFormat)
        , rotation(rotation)
        , displaySize(displaySize)
        , m_Application(application)
    {
        cv::Size frameSizeUp = frameSize;
        const bool hasTranspose = ((rotation / 90) % 2);
        if (hasTranspose)
        {
            std::swap(frameSizeUp.width, frameSizeUp.height);
        }
        
        const auto &&transform = recalculateViewGeometry(frameSizeUp, displaySize, kFillModePreserveAspectRatio);
        disp.setOffset(transform.x, transform.y);
        disp.setDisplayResolution(transform.scaleX, transform.scaleY);
        disp.setOutputSize(displaySize.width, displaySize.height);

        gray.setOutputRenderOrientation(ogles_gpgpu::degreesToOrientation(rotation));

        initDrishti(frameSizeUp, focalLength, glContext);
        
        if(m_tracker)
        {
            // Manually prepare display if drishti will be used
            gray.setGrayscaleConvType(ogles_gpgpu::GRAYSCALE_INPUT_CONVERSION_NONE);

            disp.setOutputRenderOrientation(ogles_gpgpu::RenderOrientationFlippedMirrored);
            disp.prepare(frameSizeUp.width, frameSizeUp.height, 0, false);
        }
        else
        {
            gray.add(&disp);
        }
        
        // Always prepare gray regardless of whether drishti is configured
        gray.prepare(frameSize.width, frameSize.height, 0, false);
    }
    
    std::shared_ptr<spdlog::logger>& getLogger()
    {
        return m_Application.getLogger();
    }

    void initDrishti(const cv::Size &frameSize, const float fx, void *glContext)
    {
        m_factory = FaceTrackerFactoryJson::create(
            m_Application.getAssetManager(),
            m_Application.getAsset("drishti_assets"),
            getLogger()
        );

        m_callbacks = facefilter::make_unique<FaceTrackTest>(getLogger(), "/tmp"); // path?

        // Configure the face tracker parameters:
        drishti::sdk::Vec2f p(frameSize.width / 2, frameSize.height / 2);
        drishti::sdk::SensorModel::Intrinsic intrinsic(p, fx, { frameSize.width, frameSize.height });
        drishti::sdk::SensorModel::Extrinsic extrinsic(drishti::sdk::Matrix33f::eye());
        drishti::sdk::SensorModel sensor(intrinsic, extrinsic);

        drishti::sdk::Context context(sensor);
        context.setDoSingleFace(false);                      // only detect 1 face per frame
        context.setMinDetectionDistance(0.00f);              // min distance
        context.setMaxDetectionDistance(0.5f);              // max distance
        context.setFaceFinderInterval(0.0333f);                // detect on every frame ...
        context.setAcfCalibration(0.001f);                   // adjust detection sensitivity
        context.setRegressorCropScale(1.1f);                 // regressor crop scale
        context.setMinTrackHits(3);                          // # of hits before a new track is started
        context.setMaxTrackMisses(3);                        // # of misses before the track is abandoned
        context.setMinFaceSeparation(0.5f);                  // min face separation
        context.setDoAnnotation(true);                       // add default annotations for quick preview
        context.setDoCpuACF(false);                          // available only if using the simple pipeline
        context.setDoOptimizedPipeline(true);                // configure optimized pipeline
        context.setGLContext(glContext);
    
        m_tracker = facefilter::make_unique<drishti::sdk::FaceTracker>(&context, m_factory->factory);
        if (!m_tracker)
        {
            m_Application.getLogger()->error("Failed to create face tracker");
            throw std::runtime_error("Failed to insantiate tracker");
        }
        
        m_tracker->add(m_callbacks->table);
    }

    void render()
    {
        gray.process(texId, 1, inputTextureTarget);
        
        if(m_tracker)
        {
            drishti::sdk::VideoFrame frame({frameSize.width,frameSize.height}, nullptr, false, gray.getOutputTexId(), TEXTURE_FORMAT);
            auto outputTex = (*m_tracker)(frame);
            disp.process(outputTex, 1, GL_TEXTURE_2D);
        }
    }
    
    void setDisplayBufferCallback(const std::function<void()> &callback)
    {
        setDisplayBuffer = callback;
        disp.setPreRenderCallback([=](ogles_gpgpu::ProcInterface *me) {
            setDisplayBuffer();
        });
    }

    void * glContext = nullptr;
    cv::Size frameSize;
    GLenum pixelFormat;
    int rotation = 0;
    cv::Size displaySize;
    std::uint32_t texId = 999;
    GLenum inputTextureTarget = GL_TEXTURE_2D;

    std::function<void()> setDisplayBuffer;

    ogles_gpgpu::GrayscaleProc gray;
    ogles_gpgpu::Disp disp;

    Application &m_Application;

    std::unique_ptr<FaceTrackerFactoryJson> m_factory;
    std::unique_ptr<FaceTrackTest> m_callbacks;
    std::unique_ptr<drishti::sdk::FaceTracker> m_tracker;
};

Renderer::~Renderer() = default;

Renderer::Renderer(
    void *glContext,
    const cv::Size &frameSize,
    GLenum pixelFormat,
    float focalLength,
    int rotation,
    const cv::Size &displaySize,
    Application &application
)
{
    m_impl = std::make_shared<Impl>(
        glContext, frameSize, pixelFormat, focalLength, rotation, displaySize, application
    );
}

void Renderer::resize(int w, int h) 
{
    // TODO
}

void Renderer::render() 
{
    if(m_impl)
    {
        m_impl->render();
    }
}

void Renderer::setTexture(std::uint32_t texId)
{
    if(m_impl)
    {
        m_impl->texId = texId;
    }
}

void Renderer::setInputTextureTarget(GLenum textureTaret)
{
    if(m_impl)
    {
        m_impl->inputTextureTarget = textureTaret;
    }
}

void Renderer::registerSetDisplayBufferCallback(const std::function<void()> &callback)
{
    if(m_impl)
    {
        m_impl->setDisplayBufferCallback(callback);
    }
}

END_FACEFILTER_NAMESPACE
