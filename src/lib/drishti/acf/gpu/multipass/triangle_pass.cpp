/*! -*-c++-*-
  @file   triangle_pass.cpp
  @author David Hirvonen (C++ implementation)
  @brief Declaration of a single pass ogles_gpgpu triangle filter shader.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "ogles_gpgpu/common/common_includes.h"

#include "triangle_pass.h"

#include <cmath>

using namespace ogles_gpgpu;

void getOptimizedTriangle(int blurRadius, std::vector<GLfloat>& weights, std::vector<GLfloat>& offsets)
{
    int width = 2 * blurRadius + 1;
    weights.resize(width);
    offsets.resize(width);

    float norm((blurRadius + 1) * (blurRadius + 1));

    // 5: 1 2 3 4 5 6 5 4 3 2 1
    for (int i = 0, j = width - 1; i <= blurRadius; i++, j--)
    {
        weights[i] = weights[j] = float(i + 1) / norm;
        offsets[i] = -(blurRadius - i);
        offsets[j] = +(blurRadius - i);
    }
}

std::string vertexShaderForTriangle(int blurRadius)
{
    std::vector<GLfloat> standardTriangleWeights;
    std::vector<GLfloat> optimizedTriangleOffsets;
    getOptimizedTriangle(blurRadius, standardTriangleWeights, optimizedTriangleOffsets);

    int numberOfOptimizedOffsets = optimizedTriangleOffsets.size();

    std::stringstream ss;
    ss << "attribute vec4 position;\n";
    ss << "attribute vec4 inputTextureCoordinate;\n";
    ss << "uniform float texelWidthOffset;\n";
    ss << "uniform float texelHeightOffset;\n\n";
    ss << "varying vec2 blurCoordinates[" << (unsigned long)(numberOfOptimizedOffsets) << "];\n\n";
    ss << "void main()\n";
    ss << "{\n";
    ss << "   gl_Position = position;\n";
    ss << "   vec2 singleStepOffset = vec2(texelWidthOffset, texelHeightOffset);\n";
    ss << "   blurCoordinates[0] = inputTextureCoordinate.xy;\n";
    for (int currentOptimizedOffset = 0; currentOptimizedOffset < numberOfOptimizedOffsets; currentOptimizedOffset++)
    {
        const auto& optOffset = optimizedTriangleOffsets[currentOptimizedOffset];
        ss << "   blurCoordinates[" << currentOptimizedOffset << "] = inputTextureCoordinate.xy + singleStepOffset * " << std::fixed << optOffset << ";\n";
    }
    ss << "}\n";

    return ss.str();
}

std::string fragmentShaderForTriangle(int blurRadius, bool doNorm = false, int pass = 1, float normConst = 0.005f)
{
    std::vector<GLfloat> standardTriangleWeights;
    std::vector<GLfloat> optimizedTriangleOffsets;
    getOptimizedTriangle(blurRadius, standardTriangleWeights, optimizedTriangleOffsets);

    int numberOfOffsets = optimizedTriangleOffsets.size();

    std::stringstream ss;
    ss << "uniform sampler2D inputImageTexture;\n";
    ss << "uniform float texelWidthOffset;\n";
    ss << "uniform float texelHeightOffset;\n\n";
    ss << "varying vec2 blurCoordinates[" << (numberOfOffsets) << "];\n\n";
    ss << "void main()\n";
    ss << "{\n";
    ss << "   vec4 sum = vec4(0.0);\n";
    ss << "   vec4 center = texture2D(inputImageTexture, blurCoordinates[0]);\n";

    for (int currentBlurCoordinateIndex = 0; currentBlurCoordinateIndex < numberOfOffsets; currentBlurCoordinateIndex++)
    {
        GLfloat weight = standardTriangleWeights[currentBlurCoordinateIndex];
        int index = (unsigned long)((currentBlurCoordinateIndex));
        ss << "   sum += texture2D(inputImageTexture, blurCoordinates[" << index << "]) * " << std::fixed << weight << ";\n";
    }

    if (doNorm)
    {
        if (pass == 1)
        {
            ss << "   gl_FragColor = vec4(center.rgb, sum.r);\n";
        }
        else
        {
            ss << "   gl_FragColor = vec4( center.r/(sum.a + " << std::fixed << normConst << "), center.gb, 1.0);\n";
        }
    }
    else
    {
        ss << "   gl_FragColor = sum;\n";
    }

    ss << "}\n";

    return ss.str();
}

void TriangleProcPass::setRadius(int radius)
{
    if (radius != _blurRadiusInPixels)
    {
        _blurRadiusInPixels = radius;

        //std::cout << "Blur radius " << _blurRadiusInPixels << " calculated sample radius " << calculatedSampleRadius << std::endl;
        //std::cout << "===" << std::endl;

        vshaderTriangleSrc = vertexShaderForTriangle(radius);
        fshaderTriangleSrc = fragmentShaderForTriangle(radius, doNorm, renderPass, normConst);

        std::cout << vshaderTriangleSrc << std::endl;
        std::cout << fshaderTriangleSrc << std::endl;
    }
}

// TODO: We need to override this if we are using the GPUImage shaders
void TriangleProcPass::filterShaderSetup(const char* vShaderSrc, const char* fShaderSrc, GLenum target)
{
    // create shader object
    ProcBase::createShader(vShaderSrc, fShaderSrc, target);

    // get shader params
    shParamAPos = shader->getParam(ATTR, "position");
    shParamATexCoord = shader->getParam(ATTR, "inputTextureCoordinate");
    Tools::checkGLErr(getProcName(), "filterShaderSetup");
}

void TriangleProcPass::setUniforms()
{
    FilterProcBase::setUniforms();

    glUniform1f(shParamUTexelWidthOffset, (renderPass == 1) * pxDx);
    glUniform1f(shParamUTexelHeightOffset, (renderPass == 2) * pxDy);
}

void TriangleProcPass::getUniforms()
{
    FilterProcBase::getUniforms();

    // calculate pixel delta values
    pxDx = 1.0f / (float)outFrameW; // input or output?
    pxDy = 1.0f / (float)outFrameH;

    shParamUInputTex = shader->getParam(UNIF, "inputImageTexture");
    shParamUTexelWidthOffset = shader->getParam(UNIF, "texelWidthOffset");
    shParamUTexelHeightOffset = shader->getParam(UNIF, "texelHeightOffset");
}

const char* TriangleProcPass::getFragmentShaderSource()
{
    return fshaderTriangleSrc.c_str();
}

const char* TriangleProcPass::getVertexShaderSource()
{
    return vshaderTriangleSrc.c_str();
}
