/*! -*-c++-*-
  @file  FaceFilterRender.cpp
  @brief iOS swift bindings for the facefilter application.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
*/

#include <facefilter/renderer/Context.h>
#include <facefilter/renderer/Renderer.h>
#include <facefilter/renderer/facefilter_renderer.h>

#include <iostream>
#include <memory>

facefilter::Application g_app;

#ifdef __cplusplus
FACEFILTER_EXTERN_C_BEGIN
#endif

void FaceFilterRender_loadAsset(const char* key, const char* url)
{
    g_app.loadAsset(key, url);
}

void FaceFilterRender_drawFrame(uint32_t texture)
{
    g_app.drawFrame(texture);
}

void FaceFilterRender_surfaceChanged(int width, int height)
{
    g_app.initDisplay(width, height);
}

void FaceFilterRender_surfaceCreated(int width, int height)
{
    g_app.initDisplay(width, height);
}

void FaceFilterRender_cameraCreated(int width, int height, int rotation, float focalLength)
{
    g_app.initCamera(width, height, rotation, focalLength);
}

int FaceFilterRender_allocTexture(int width, int height)
{
    return 0;
}

void FaceFilterRender_registerCallback(void* classPtr, void (*callback)(void*))
{
    g_app.registerSetDisplayBufferCallback([=]() { callback(classPtr); });
}

void FaceFilterRender_setGLContext(void* context)
{
    g_app.setGLContext(context);
}

#ifdef __cplusplus
FACEFILTER_EXTERN_C_END
#endif
