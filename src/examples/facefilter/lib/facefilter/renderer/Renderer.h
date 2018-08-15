/*! -*-c++-*-
  @file Renderer.h
  @brief Declaration of the main native facefilter tracking and rendering manager class.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
*/

#ifndef __facefilter_renderer_Renderer_h__
#define __facefilter_renderer_Renderer_h__

#include <facefilter/facefilter.h>

#include <opencv2/core.hpp>

#include <ogles_gpgpu/platform/opengl/gl_includes.h>

#include <memory>

BEGIN_FACEFILTER_NAMESPACE

struct Application;

class Renderer
{
public:
    Renderer(
        void* glContext,
        const cv::Size& frameSize,
        GLenum pixelFormat,
        float focalLength,
        int rotation,
        const cv::Size& displaySize,
        Application& application);
    virtual ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer(Renderer&&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer& operator=(Renderer&&) = delete;

    void resize(int w, int h);
    void setPreviewGeometry(float tx, float ty, float sx, float sy);
    void render();
    void setTexture(std::uint32_t texId);
    void setInputTextureTarget(GLenum textureTaret);
    void registerSetDisplayBufferCallback(const std::function<void()>& callback);

protected:
    struct Impl;
    std::shared_ptr<Impl> m_impl;
};

END_FACEFILTER_NAMESPACE

#endif // __facefilter_renderer_Renderer_h__
