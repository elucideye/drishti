/*! -*-c++-*-
  @file   StaticObject.h
  @author David Hirvonen
  @brief  Abstraction for object w/ position and size.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_geometry_StaticObject_h__
#define __drishti_geometry_StaticObject_h__

#include "drishti/geometry/drishti_geometry.h"
#include <opencv2/core.hpp>

DRISHTI_GEOMETRY_BEGIN

class StaticObject3D
{
public:
    cv::Point3f position;
    cv::Vec3f size;
};

DRISHTI_GEOMETRY_END

#endif // __drishti_geometry_StaticObject_h__
