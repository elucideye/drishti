/*!
  @file swizzle2.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of an ogles_gpgpu shader for common texture channel swizzles.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef OGLES_GPGPU_SWIZZLE2_H
#define OGLES_GPGPU_SWIZZLE2_H

#include "drishti/acf/drishti_acf.h"
#include "ogles_gpgpu/common/proc/two.h"

BEGIN_OGLES_GPGPU

// ##### Simple multi texture swizzling: #######
class MergeProc : public TwoInputProc
{
public:

    enum SwizzleKind
    {
        kSwizzleABC1,
        kSwizzleAB12
    };

    MergeProc() {}
    virtual const char *getProcName()
    {
        return "MergeProc";
    }
    void setSwizzleType(SwizzleKind kind)
    {
        swizzleKind = kind;
    }

    /**
     * Perform a standard shader initialization.
     */
    virtual int init(int inW, int inH, unsigned int order, bool prepareForExternalInput);
    virtual void useTexture(GLuint id, GLuint useTexUnit = 1, GLenum target = GL_TEXTURE_2D, int position = 0);

private:

    SwizzleKind swizzleKind = kSwizzleABC1;

    virtual const char *getFragmentShaderSource()
    {
        switch(swizzleKind)
        {
            case kSwizzleABC1:
                return fshaderMergeSrcABC1;
            case kSwizzleAB12:
                return fshaderMergeSrcAB12;
            default:
                assert(false);
        }
    }
    static const char *fshaderMergeSrcABC1;   // fragment shader source
    static const char *fshaderMergeSrcAB12;   // fragment shader source
};


END_OGLES_GPGPU

#endif // OGLES_GPGPU_SWIZZLE2_H
