/*! -*-c++-*-
  @file   FaceIO.h
  @author David Hirvonen
  @brief  Declaration of utilities for insantiating face models.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceIO_h__
#define __drishti_face_FaceIO_h__

#include "drishti/face/drishti_face.h"
#include "drishti/face/Face.h"
#include "drishti/core/Shape.h"

DRISHTI_FACE_NAMESPACE_BEGIN

using PointVec = std::vector<cv::Point2f>;

struct FaceSpecification
{
    enum Format
    {
        kHELEN,
        kibug68,
        kibug68_inner
    };

    Format format = kibug68;

    using IntVec = std::vector<int>;

    // Populate indices for eyes and nose
    IntVec eyeL;
    IntVec eyeR;
    IntVec nose;
    IntVec brow;
    IntVec mouthOuter;
    IntVec mouthInner;
    IntVec browL;
    IntVec browR;

    IntVec sideL;
    IntVec sideR;

    IntVec noseFull;

    static FaceSpecification create(Format format);
};

FaceModel shapeToFace(drishti::core::Shape& shape, FaceSpecification::Format kind = FaceSpecification::kibug68);

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceIO_h__
