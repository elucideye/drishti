/*! -*-c++-*-
  @file   DynamicObject.h
  @author David Hirvonen
  @brief  Abstraction for object w/ position, size, and velocity.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_geometry_DynamicObject_h__
#define __drishti_geometry_DynamicObject_h__

#include "drishti/geometry/drishti_geometry.h"
#include "drishti/geometry/StaticObject.h"

DRISHTI_GEOMETRY_BEGIN

class DynamicObject3D : public StaticObject3D
{
public:
    DynamicObject3D();
    cv::Vec3f velocity;
};

DRISHTI_GEOMETRY_END

#endif // __drishti_geometry_DynamicObject_h__
