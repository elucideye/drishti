// getPointsOnLine -  given an homogeneous line return two homogeneous points on it
//              - Pierluigi Taddei (pierluigi.taddei@polimi.it)
//
// Usage:    [p1 p2] = getPointsOnLine(l)
//
// Arguments:
//           l - line vector
//           p1 p2 - two points on l
//
// 06.3.2007 : Created

// Copyright (c) 2015, Pierluigi Taddei
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in
//       the documentation and/or other materials provided with the distribution
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef __drishti_geometry_getPointsOnLine_h__
#define __drishti_geometry_getPointsOnLine_h__ 1

#include "drishti/geometry/drishti_geometry.h"

DRISHTI_GEOMETRY_BEGIN

// function [p1 p2] = getPointsOnLine(l)
//    if (l(1) == 0 && l(2) == 0) %line at infinity
//        p1 = [1 0 0]';
//        p2 = [0 1 0]';
//    else
//     p2 = [-l(2), l(1), 0]';
//     if (abs(l(1)) < abs(l(2)))
//         p1 = [0, -l(3), l(2)]';
//     else
//         p1 = [-l(3), 0, l(1)]';
//     end
//    end
// end

template <typename T>
void getPointsOnLine(const cv::Vec<T, 3>& l, cv::Vec<T, 3>& p1, cv::Vec<T, 3>& p2)
{
    if ((l[0] == T(0.0)) && (l[1] == T(0.0)))
    {
        p1 = { T(1.0), T(0.0), T(0.0) };
        p2 = { T(0.0), T(1.0), T(0.0) };
    }
    else
    {
        p2 = { -l[1], l[0], T(0.0) };
        if (std::abs(l[0]) < std::abs(l[1]))
        {
            p1 = { T(0.0), -l[2], l[1] };
        }
        else
        {
            p1 = { -l[2], T(0.0), l[0] };
        }
    }
}

DRISHTI_GEOMETRY_END

#endif // __drishti_geometry_getPointsOnLine_h__ 1
