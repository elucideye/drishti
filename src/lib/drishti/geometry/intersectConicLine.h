// intersectConicLine - given a line and a conic detect the real intersection
// points
//          - Pierluigi Taddei (pierluigi.taddei@polimi.it)
//
// Usage:   P = intersectConicLine(C, l)
//
// Arguments:
//           C - homogeneous conic matrix
//           l - homogeneous line vector
//
//           P - matrix of homogeneous intersection points (each column is a
//           point)
// 08.3.2007 : Created

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

#ifndef __drishti_geometry_intersectConicLine_h__
#define __drishti_geometry_intersectConicLine_h__ 1

#include "drishti/geometry/drishti_geometry.h"
#include "drishti/geometry/getPointsOnLine.h"

DRISHTI_GEOMETRY_BEGIN

// function P = intersectConicLine(C, l)
//     P = [];
//     [p1 p2] = getPointsOnLine(l);
//
//     %p1'Cp1 + 2k p1'Cp2 + k^2 p2'Cp2 = 0
//     p1Cp1 = p1' *C* p1;
//     p2Cp2 = p2' *C* p2;
//     p1Cp2 = p1' *C* p2;
//
//     if (p2Cp2 == 0) %linear
//        k1 = -0.5*p1Cp1 / p1Cp2;
//        P = [p1 + k1*p2];
//     else
//         delta = p1Cp2^2 - p1Cp1*p2Cp2;
//         if (delta >= 0)
//             deltaSqrt = sqrt(delta);
//             k1 = (-p1Cp2 + deltaSqrt)/p2Cp2;
//             k2 = (-p1Cp2 - deltaSqrt)/p2Cp2;
//
//             P = [p1 + k1*p2, p1 + k2*p2];
//        % else
//        %     disp('no intersection points detected');
//         end
//     end

template <typename T>
int intersectConicLine(const cv::Matx<T, 3, 3>& C, const cv::Vec<T, 3>& l, cv::Vec<T, 3> P[2])
{
    int n = 0;

    cv::Vec<T, 3> p1, p2;
    getPointsOnLine(l, p1, p2);

    const auto p1Cp1 = (p1.t() * C * p1)[0];
    const auto p2Cp2 = (p2.t() * C * p2)[0];
    const auto p1Cp2 = (p1.t() * C * p2)[0];

    if (p2Cp2 == T(0.0))
    {
        n = 1;
        const auto k1 = T(-0.5) * p1Cp1 / p1Cp2;
        P[0] = p1 + (k1 * p2);
    }
    else
    {
        const auto delta = (p1Cp2 * p1Cp2) - (p1Cp1 * p2Cp2);
        if (delta >= T(0.0))
        {
            n = 2;
            const auto deltaSqrt = std::sqrt(delta);
            const auto k1 = (-p1Cp2 + deltaSqrt) / p2Cp2;
            const auto k2 = (-p1Cp2 - deltaSqrt) / p2Cp2;
            P[0] = p1 + k1 * p2;
            P[1] = p1 + k2 * p2;
        }
        else
        {
            // not intersection points detected
            n = 0;
        }
    }

    return n;
}

template <typename T>
int intersectConicLine(const cv::Matx<T, 3, 3>& C, const cv::Point3_<T>& l, cv::Vec<T, 3> P[2])
{
    return intersectConicLine(C, cv::Vec<T, 3>(l.x, l.y, l.z), P);
}

DRISHTI_GEOMETRY_END

#endif // __drishti_geometry_intersectConicLine_h__
