/*! -*-c++-*-
  @file   MeshProc.h
  @author David Hirvonen
  @brief Declaration of ogles_gpgpu shader for drawing lines.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_graphics_MeshProc_h__
#define __drishti_graphics_MeshProc_h__

#include "ogles_gpgpu/common/proc/base/filterprocbase.h"

#include "drishti/graphics/meshtex.h"
#include "drishti/graphics/LineShader.h"
#include "drishti/graphics/MeshShader.h"

#include <opencv2/core.hpp>

#include <functional>

BEGIN_OGLES_GPGPU

class MeshProc : public ogles_gpgpu::FilterProcBase
{
public:
    using VertexBuffer = std::vector<glm::vec4>;
    using CoordBuffer = std::vector<glm::vec2>;

    MeshProc(const drishti::graphics::MeshTex& mesh, const cv::Mat& iso, bool doWire);

    const char* getProcName()
    {
        return "MeshProc";
    }

    void setModelViewProjection(const glm::mat4& mvp);

    void setBackground(const glm::vec4& rgba)
    {
        background = rgba;
    }

private:
    virtual void filterRenderCleanup();

    virtual const char* getFragmentShaderSource()
    {
        return fshaderPixelSrc;
    }

    std::shared_ptr<LineShader> lineShader;
    std::shared_ptr<MeshShader> meshShader;

    glm::vec4 background = { 0.f, 0.f, 0.f, 0.f };

    static const char* fshaderPixelSrc;
};

END_OGLES_GPGPU

#endif // __drishti_graphics_MeshProc_h__
