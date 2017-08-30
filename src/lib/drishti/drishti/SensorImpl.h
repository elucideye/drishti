/*!
  @file   SensorImpl.h
  @author David Hirvonen
  @brief  Private implementation of a simple sensor/camera abstraction.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_drishti_SensorImpl_h__
#define __drishti_drishti_SensorImpl_h__

#include "drishti/Sensor.hpp"
#include "drishti/sensor/Sensor.h"

_DRISHTI_SDK_BEGIN

struct SensorModel::Impl
{
    Impl(const sensor::SensorModel::Intrinsic &intrinsic, const sensor::SensorModel::Extrinsic &extrinsic)
    {
        sensor = std::make_shared<sensor::SensorModel>(intrinsic, extrinsic);
    }

    // Use shared_ptr<> to support sharing w/ public SDK classes
    std::shared_ptr<drishti::sensor::SensorModel> sensor;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_SensorImpl_h__
