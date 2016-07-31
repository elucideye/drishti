/*!
  @file   FaceIO.h
  @author David Hirvonen
  @brief  Declaration of utilities for insantiating face models.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishtisdk__FaceIO__
#define __drishtisdk__FaceIO__

#include "face/drishti_face.h"
#include "face/Face.h"
#include "core/Shape.h"

BEGIN_FACE_NAMESPACE

using PointVec = std::vector<cv::Point2f>;

struct FaceSpecification
{
    enum Format
    {
        HELEN,
        LFPW
    };

    Format format = HELEN;

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

FaceModel shapeToFace(drishti::core::Shape &shape, FaceSpecification::Format kind = FaceSpecification::HELEN );


END_FACE_NAMESPACE

#endif // __drishtisdk__FaceIO__
