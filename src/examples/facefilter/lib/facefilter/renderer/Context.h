/*! -*-c++-*-
  @file  Context.h
  @brief Context declaration for C++ facefilter application.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
*/

#ifndef __facefilter_renderer_Context_h__
#define __facefilter_renderer_Context_h__

#include <facefilter/facefilter.h>
#include <facefilter/renderer/Renderer.h>

#include <memory>
#include <functional>

#include <spdlog/spdlog.h> // for portable logging
#include <spdlog/fmt/ostr.h>

// clang-format off
#if defined(__ANDROID__)
#  include <jni.h> // JNIEnv
#endif
// clang-format on

BEGIN_FACEFILTER_NAMESPACE

struct Application
{
public:
    Application();
    ~Application();
    void setAssetManager(void* assetManager);
    void loadAsset(const char* key, const char* filename);
    void drawFrame(std::uint32_t texId);
    void initContext(void* context);
    void initCamera(int width, int height, int rotation, float focalLength);
    void initDisplay(int width, int height);
    void destroy();
    void setGLContext(void* context);
    void registerSetDisplayBufferCallback(const std::function<void()>& callback);

    std::shared_ptr<spdlog::logger>& getLogger();

    const std::string& getAsset(const char* key = "drishti_assets");
    void* getAssetManager();

protected:
    void initPipeline();

    void* m_assetManager = nullptr;

    void* m_context = nullptr;
    std::function<void()> setDisplayBufferCallback;

    bool m_hasCamera;
    int m_cameraWidth;
    int m_cameraHeight;
    int m_cameraRotation;
    float m_cameraFocalLength;

    bool m_hasDisplay;
    int m_displayWidth;
    int m_displayHeight;

    std::string m_assets;

#if defined(__ANDROID__)
    JNIEnv* m_JNIEnv;
    jobject m_jobject;
#endif

    std::shared_ptr<Renderer> m_renderer;

    std::shared_ptr<spdlog::logger> m_logger;
};

END_FACEFILTER_NAMESPACE

#endif // __facefilter_renderer_Context_h__
