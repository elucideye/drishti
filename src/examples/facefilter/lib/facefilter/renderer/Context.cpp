/*! -*-c++-*-
  @file  Context.cpp
  @brief Context implementation for C++ facefilter application.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This contains required state for the facefilter application to instantiate and
  run the native c++ and OpenGL facefilter processing code.
*/

#include <facefilter/renderer/Context.h>
#include <facefilter/make_unique.h>

#include <ogles_gpgpu/common/core.h>
#if defined(__ANDROID__) || defined(ANDROID)
#  include <ogles_gpgpu/platform/opengl/gl_includes.h> // GL_TEXTURE_EXTERNAL_OES
#endif

#include <ogles_gpgpu/common/proc/video.h>
#include <ogles_gpgpu/common/proc/grayscale.h>

#include <iostream>

BEGIN_FACEFILTER_NAMESPACE

static std::shared_ptr<spdlog::logger> createLogger(const char* name);

Application::Application()
    : m_hasCamera(false)
    , m_hasDisplay(false)
#if defined(__ANDROID__)
    , m_JNIEnv(nullptr)
#endif
{
    m_logger = createLogger("facefilter");
}

Application::~Application()
{
    spdlog::drop("facefilter");
}

std::shared_ptr<spdlog::logger>& Application::getLogger()
{
    return m_logger;
}

const std::string& Application::getAsset(const char* key)
{
    return m_assets;
}

void Application::setAssetManager(void* assetManager)
{
    m_assetManager = assetManager;
}

void* Application::getAssetManager()
{
    return m_assetManager;
}

void Application::loadAsset(const char* key, const char* filename)
{
    if (std::string(key) == "drishti_assets")
    {
        m_assets = filename;
    }
}

void Application::initContext(void* context)
{
    m_context = context;
}

void Application::drawFrame(std::uint32_t texId)
{
    if (m_renderer)
    {
        m_renderer->setTexture(texId);
        m_renderer->render();
    }
}

void Application::drawFrame(void* ptr, bool useRawPixels, std::uint32_t type)
{
    if (!m_video)
    {
        m_video = facefilter::make_unique<ogles_gpgpu::VideoSource>();
        m_gray = facefilter::make_unique<ogles_gpgpu::GrayscaleProc>();
        m_gray->setGrayscaleConvType(ogles_gpgpu::GRAYSCALE_INPUT_CONVERSION_NONE);
        m_video->set(m_gray.get());
    }
    (*m_video)({ { m_cameraWidth, m_cameraHeight }, ptr, false, 0, type });

    drawFrame(m_gray->getOutputTexId());
}

void Application::initCamera(int width, int height, int rotation, float focalLength)
{
    m_hasCamera = true;
    m_cameraWidth = width;
    m_cameraHeight = height;
    m_cameraRotation = rotation;
    m_cameraFocalLength = focalLength;

    initPipeline();
}

void Application::initDisplay(int width, int height)
{
    m_hasDisplay = true;
    m_displayWidth = width;
    m_displayHeight = height;

    initPipeline();
}

void Application::setPreviewGeometry(float tx, float ty, float sx, float sy)
{
    if (m_renderer)
    {
        m_renderer->setPreviewGeometry(tx, ty, sx, sy);
    }
}

void Application::destroy()
{
    m_hasCamera = false;
    m_hasDisplay = false;

    m_renderer = nullptr;
}

void Application::initPipeline()
{
#if OGLES_GPGPU_IOS && !defined(DRISHTI_OPENGL_ES3)
    // Only avaialble for ios specific builds (ios texture cache)
    m_context = ogles_gpgpu::Core::getCurrentEAGLContext();
    ogles_gpgpu::Core::tryEnablePlatformOptimizations();
#endif

    if (m_hasCamera && m_hasDisplay && !m_assets.empty() && !m_renderer)
    {
        glActiveTexture(GL_TEXTURE0);
        m_renderer = std::make_shared<Renderer>(
            m_context,
            cv::Size(m_cameraWidth, m_cameraHeight),
            GL_RGBA,
            m_cameraFocalLength,
            m_cameraRotation,
            cv::Size(m_displayWidth, m_displayHeight),
            *this
        );

#if defined(__ANDROID__) || defined(ANDROID)
        // For fast texture processing on Android devices, the camera is paired
        // with a SurfaceTexture in teh Android application layer:
        //
        //   https://developer.android.com/ndk/reference/group/surface-texture
        //
        // This configuration will provide direct and efficient texture output,
        // however, the textures are provided in GL_TEXTURE_EXTERNAL_OES format,
        // and we must inform our native ogles_gpgpu OpenGL layer about this
        // input type so it can handle them properly.
        m_renderer->setInputTextureTarget(GL_TEXTURE_EXTERNAL_OES);
#endif

        if (setDisplayBufferCallback)
        {
            m_renderer->registerSetDisplayBufferCallback(setDisplayBufferCallback);
        }
    }
}

void Application::setGLContext(void* context)
{
    m_context = context;
}

// Note: This must come after initPipeline (obviously)
void Application::registerSetDisplayBufferCallback(const std::function<void()>& callback)
{
    setDisplayBufferCallback = callback;
}

static std::shared_ptr<spdlog::logger> createLogger(const char* name)
{
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_mt>());
#if defined(__ANDROID__)
    sinks.push_back(std::make_shared<spdlog::sinks::android_sink>());
#endif
    auto logger = std::make_shared<spdlog::logger>(name, begin(sinks), end(sinks));
    spdlog::register_logger(logger);
    spdlog::set_pattern("[%H:%M:%S.%e | thread:%t | %n | %l]: %v");
    return logger;
}

END_FACEFILTER_NAMESPACE
