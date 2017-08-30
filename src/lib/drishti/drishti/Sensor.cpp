/*!
  @file   Sensor.cpp
  @author David Hirvonen
  @brief  Implementation of a simple sensor/camera abstraction.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_drishti_Sensor_h__
#define __drishti_drishti_Sensor_h__

#include "drishti/Sensor.hpp"
#include "drishti/SensorImpl.h" // private
#include "drishti/sensor/Sensor.h"

#include <opencv2/core.hpp>

#include <array>
#include <memory>

_DRISHTI_SDK_BEGIN

template <typename Value, typename... Arguments>
std::unique_ptr<Value> make_unique(Arguments&&... arguments_for_constructor)
{
    return std::unique_ptr<Value>(new Value(std::forward<Arguments>(arguments_for_constructor)...));
}

SensorModel::Intrinsic::Intrinsic(const Vec2f& c, float fx, const Vec2i& size)
    : m_c(c)
    , m_fx(fx)
    , m_size(size)
{
}

SensorModel::Extrinsic::Extrinsic(const Matrix33f &R)
    : m_R(R)
{
}

SensorModel::SensorModel(const Intrinsic& intrinsic, const Extrinsic& extrinsic)
{
    // Convert intrinsic:
    const cv::Point2f c(intrinsic.m_c[0], intrinsic.m_c[1]);
    const cv::Size size(intrinsic.m_size[0], intrinsic.m_size[1]);
    drishti::sensor::SensorModel::Intrinsic intrinsic_(c, intrinsic.m_fx, size);

    // Convert extrinsic:
    cv::Matx33f R;
    for (int y = 0; y < 3; y++)
    {
        for (int x = 0; x < 3; x++)
        {
            R(y,x) = extrinsic.m_R(y,x);
        }
    }
    drishti::sensor::SensorModel::Extrinsic extrinsic_(R);
    
    impl = make_unique<Impl>(intrinsic_, extrinsic_);
}

SensorModel::~SensorModel() = default;

_DRISHTI_SDK_END

#endif
