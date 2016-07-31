/*!
  @file   triangle.cpp
  @author David Hirvonen (C++ implementation) <dhirvonen elucideye com>
  @brief Implementation of a separable ogles_gpgpu triangle filter shader.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "acf/gpu/triangle.h"

BEGIN_OGLES_GPGPU

TriangleProc::TriangleProc(int radius, bool doNorm, float normConst)
{
    TriangleProcPass *triPass1 = new TriangleProcPass(1, radius, doNorm);
    TriangleProcPass *triPass2 = new TriangleProcPass(2, radius, doNorm, normConst);

    procPasses.push_back(triPass1);
    procPasses.push_back(triPass2);
}

END_OGLES_GPGPU
