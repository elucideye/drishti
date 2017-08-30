/*!
  @file   Sensor.h
  @author David Hirvonen
  @brief  Implementation of a simple sensor/camera abstraction.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_drishti_Sensor_hpp__
#define __drishti_drishti_Sensor_hpp__

#include "drishti/drishti_sdk.hpp"
#include "drishti/Image.hpp"

#include <array>

_DRISHTI_SDK_BEGIN

class DRISHTI_EXPORT SensorModel
{
public:
    // ### Intrisnic camera parameters:
    struct Intrinsic
    {
        Intrinsic(const Vec2f& c, float fx, const Vec2i& size);
        const Vec2f m_c;
        const float m_fx;
        const Vec2i m_size;
    };

    // ### Extrinsic camera parameters:
    struct Extrinsic
    {
        Extrinsic(const Matrix33f &R);
        Matrix33f m_R;
    };

    SensorModel(const Intrinsic& intrinsic, const Extrinsic& extrinsic);
    ~SensorModel();

    struct Impl;
    
    std::unique_ptr<Impl> &getImpl()
    {
        return impl;
    }

    const std::unique_ptr<Impl> &getImpl() const
    {
        return impl;
    }   

protected:

    std::unique_ptr<Impl> impl;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_Sensor_hpp__
