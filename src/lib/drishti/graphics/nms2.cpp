//
// ogles_gpgpu project - GPGPU for mobile devices and embedded systems using OpenGL ES 2.0
//
// Author: Markus Konrad <post@mkonrad.net>, Winter 2014/2015  http://www.mkonrad.net
//         David Hirvonen
// See LICENSE file in project repository root for the license.
//

#include "drishti/graphics/nms2.h"

#include <regex>

using namespace std;
using namespace ogles_gpgpu;

// clang-format off
const char *Nms2Proc::fshaderNmsSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
precision OGLES_GPGPU_HIGHP float;
#endif

 uniform sampler2D inputImageTexture;
 
 varying vec2 textureCoordinate;
 varying vec2 leftTextureCoordinate;
 varying vec2 rightTextureCoordinate;
 
 varying vec2 topTextureCoordinate;
 varying vec2 topLeftTextureCoordinate;
 varying vec2 topRightTextureCoordinate;
 
 varying vec2 bottomTextureCoordinate;
 varying vec2 bottomLeftTextureCoordinate;
 varying vec2 bottomRightTextureCoordinate;
 
 uniform float threshold;
 
 void main()
 {
     float bottomColor = texture2D(inputImageTexture, bottomTextureCoordinate).r;
     float bottomLeftColor = texture2D(inputImageTexture, bottomLeftTextureCoordinate).r;
     float bottomRightColor = texture2D(inputImageTexture, bottomRightTextureCoordinate).r;
     vec4 centerColor = texture2D(inputImageTexture, textureCoordinate);
     float leftColor = texture2D(inputImageTexture, leftTextureCoordinate).r;
     float rightColor = texture2D(inputImageTexture, rightTextureCoordinate).r;
     float topColor = texture2D(inputImageTexture, topTextureCoordinate).r;
     float topRightColor = texture2D(inputImageTexture, topRightTextureCoordinate).r;
     float topLeftColor = texture2D(inputImageTexture, topLeftTextureCoordinate).r;
     
     // Use a tiebreaker for pixels to the left and immediately above this one
     float multiplier = 1.0 - step(centerColor.r, topColor);
     multiplier = multiplier * (1.0 - step(centerColor.r, topLeftColor));
     multiplier = multiplier * (1.0 - step(centerColor.r, leftColor));
     multiplier = multiplier * (1.0 - step(centerColor.r, bottomLeftColor));
     
     float maxValue = max(centerColor.r, bottomColor);
     maxValue = max(maxValue, bottomRightColor);
     maxValue = max(maxValue, rightColor);
     maxValue = max(maxValue, topRightColor);
     
     float finalValue = centerColor.r * step(maxValue, centerColor.r) * multiplier;
     
     gl_FragColor = vec4(finalValue, centerColor.gba); // DO NOT EDIT (see swizzle)
});
// clang-format on

Nms2Proc::Nms2Proc()
{
}

void Nms2Proc::setUniforms()
{
    Filter3x3Proc::setUniforms();
    glUniform1f(shParamUThreshold, threshold);
}

void Nms2Proc::getUniforms()
{
    Filter3x3Proc::getUniforms();
    shParamUInputTex = shader->getParam(UNIF, "inputImageTexture");
    shParamUThreshold = shader->getParam(UNIF, "threshold");
}

void Nms2Proc::swizzle(int channelIn, int channelOut)
{
    std::string new1, new2;

    if (channelOut < 0)
    {
        channelOut = channelIn;
    }

    fshaderNmsSwizzleSrc.clear();
    fshaderNmsSwizzleSrc += fshaderNmsSrc; // deep copy

    switch (channelIn)
    {
        case 0:
            break; // R
        case 1:
        {
            new1 = ".g";
            break;
        } // G
        case 2:
        {
            new1 = ".b";
            break;
        } // B
        case 3:
        {
            new1 = ".a";
            break;
        } // A
        default:
            assert(false);
    }

    switch (channelOut)
    {
        case 0:
            break;
        case 1:
        {
            new2 = "vec4(centerColor.r, finalValue, centerColor.ba)";
            break;
        }
        case 2:
        {
            new2 = "vec4(centerColor.rg, finalValue, centerColor.a)";
            break;
        }
        case 3:
        {
            new2 = "vec4(centerColor.rgb, finalValue)";
            break;
        }
        default:
            assert(false);
    }

    const std::regex pattern1("\\.r");
    const std::regex pattern2("vec4\\(finalValue, centerColor.gba\\)");

    if (!new1.empty())
    {
        auto fshader1 = std::regex_replace(fshaderNmsSwizzleSrc, pattern1, new1);
        std::swap(fshader1, fshaderNmsSwizzleSrc);
    }

    if (!new2.empty())
    {
        auto fshader2 = std::regex_replace(fshaderNmsSwizzleSrc, pattern2, new2);
        std::swap(fshader2, fshaderNmsSwizzleSrc);
    }
}
