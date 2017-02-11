/*!
  @file fade.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of ogles_gpgpu shader for exponential decay effect.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef OGLES_GPGPU_COMMON_FADE_PROC
#define OGLES_GPGPU_COMMON_FADE_PROC

#include "ogles_gpgpu/common/proc/base/multipassproc.h"

#include <memory>

BEGIN_OGLES_GPGPU

class FadeFilterProc : public MultiPassProc
{
public:

    class Impl;
    
    FadeFilterProc(float decay=0.99);
    ~FadeFilterProc();
    virtual void useTexture(GLuint id, GLuint useTexUnit = 1, GLenum target = GL_TEXTURE_2D, int position=0);
    virtual int render(int position=0);
    virtual const char *getProcName() { return "FadeFilterProc"; }
    virtual ProcInterface* getInputFilter() const;
    virtual ProcInterface* getOutputFilter() const;
    
    virtual int init(int inW, int inH, unsigned int order, bool prepareForExternalInput = false);
    virtual int reinit(int inW, int inH, bool prepareForExternalInput = false);
    
    bool isFirst = true;
    std::unique_ptr<Impl> m_impl;
};

END_OGLES_GPGPU

#endif 
