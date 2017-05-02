#include "drishti/graphics/fade.h"
#include "drishti/core/make_unique.h"

#include "ogles_gpgpu/common/proc/fifo.h"
#include "ogles_gpgpu/common/proc/blend.h"
#include "ogles_gpgpu/common/proc/two.h"

BEGIN_OGLES_GPGPU

class DecayProc : public TwoInputProc
{
public:
    DecayProc(float alpha = 1.0f);
    virtual const char* getProcName() { return "DecayProc"; }
    virtual void getUniforms();
    virtual void setUniforms();
    virtual void setAlpha(float value) { alpha = value; }
    virtual int render(int position = 0);

private:
    GLint shParamUAlpha;
    GLfloat alpha = 0.5f;

    virtual const char* getFragmentShaderSource() { return fshaderBlendSrc; }
    static const char* fshaderBlendSrc; // fragment shader source
};

// clang-format off
const char* DecayProc::fshaderBlendSrc = OG_TO_STR(
#if defined(OGLES_GPGPU_OPENGLES)
    precision highp float;
#endif

    varying vec2 textureCoordinate;
    varying vec2 textureCoordinate2;

    uniform sampler2D inputImageTexture;
    uniform sampler2D inputImageTexture2;
    uniform float alpha;

    void main()
    {
        vec4 textureColor = texture2D(inputImageTexture, textureCoordinate);
        vec4 textureColor2 = texture2D(inputImageTexture2, textureCoordinate);
        gl_FragColor = vec4(max(textureColor.rgb, textureColor2.rgb * alpha), 1.0);
    }
);
// clang-format on

DecayProc::DecayProc(float alpha)
    : alpha(alpha)
{
}

void DecayProc::getUniforms()
{
    TwoInputProc::getUniforms();
    shParamUAlpha = shader->getParam(UNIF, "alpha");
}

void DecayProc::setUniforms()
{
    TwoInputProc::setUniforms();
    glUniform1f(shParamUAlpha, alpha);
}

int DecayProc::render(int position)
{
    // Always render on position == 0
    TwoInputProc::render(position);
    return position;
}

/////////////////////////////
//
// INPUT ===+= FADE ===>OUTUPUT
//             ^     |
//             |     |
//           FIFO <==+
/////////////////////////////

// Convenience initializer:
class FadeFilterProc::Impl
{
public:
    Impl(float decay)
        : decayProc(decay)
        , fifoProc(1)
    {
        decayProc.add(&fifoProc);
        fifoProc.add(&decayProc, 1);
        decayProc.setWaitForSecondTexture(false);
    }
    DecayProc decayProc;
    FIFOPRoc fifoProc;
    ProcInterface* lastProc = &decayProc; // default for low pass
};

FadeFilterProc::FadeFilterProc(float decay)
{
    // All procPasses filter will be initialized by superclass:
    m_impl = drishti::core::make_unique<Impl>(decay);
    procPasses.push_back(&m_impl->decayProc);
    procPasses.push_back(&m_impl->fifoProc);
}

int FadeFilterProc::init(int inW, int inH, unsigned int order, bool prepareForExternalInput)
{
    isFirst = true;
    return MultiPassProc::init(inW, inH, order, prepareForExternalInput);
}

int FadeFilterProc::reinit(int inW, int inH, bool prepareForExternalInput)
{
    isFirst = true;
    return MultiPassProc::reinit(inW, inH, prepareForExternalInput);
}

FadeFilterProc::~FadeFilterProc()
{
    procPasses.clear();
}
ProcInterface* FadeFilterProc::getInputFilter() const { return &m_impl->decayProc; }
ProcInterface* FadeFilterProc::getOutputFilter() const { return m_impl->lastProc; }

int FadeFilterProc::render(int position)
{ // Execute internal filter chain

    // calls next->useTexture(); next->process();
    // TODO: We don't really need this to call fifoProc->useTexture(),
    // if it is done by this class (see below for special case first frame handling)
    m_impl->decayProc.process(0);
    return 0;
}

void FadeFilterProc::useTexture(GLuint id, GLuint useTexUnit, GLenum target, int position)
{
    auto& fifoProc = m_impl->fifoProc;
    auto& decayProc = m_impl->decayProc;

    // FADE filter input (same image for first frame)
    if (isFirst)
    {
        isFirst = false;
        decayProc.useTexture(id, useTexUnit, target, 0);
        decayProc.useTexture2(id, useTexUnit, GL_TEXTURE_2D);

        // FIFO input (from FADE)
        fifoProc.useTexture(id, useTexUnit, target, 0);
    }
    else
    {
        // Connect FIFO output to second input of FADE for frames >= 1
        decayProc.useTexture(id, useTexUnit, target, 0);
        decayProc.useTexture2(fifoProc.getOutputTexId(), fifoProc.getTextureUnit(), GL_TEXTURE_2D);

        // FIFO input (from FADE)
        fifoProc.useTexture(decayProc.getOutputTexId(), decayProc.getTextureUnit(), GL_TEXTURE_2D, 0);
    }
}

END_OGLES_GPGPU
