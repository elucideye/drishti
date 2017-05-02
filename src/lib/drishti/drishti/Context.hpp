/**
  @file   Context.hpp
  @author David Hirvonen
  @brief  Public API for continuous face filter.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the public API of the Context class.  This class 
  will store and allocate internal state necessary for real time 
  face tracking.
  
*/

#ifndef __drishti_drishti_Context_hpp__
#define __drishti_drishti_Context_hpp__

#include "drishti/drishti_sdk.hpp"
#include "drishti/drishti_gl.hpp"
#include "drishti/Image.hpp"

// Note: This must be a public header
#include "drishti/sensor/Sensor.h"

#include <memory>

_DRISHTI_SDK_BEGIN

class DRISHTI_EXPORT Context
{
public:
    class Impl;
    Context(const drishti::sensor::SensorModel& sensor);
    ~Context();

    Impl* get() { return m_impl.get(); }

    void setMinDetectionDistance(float value);
    float getMinDetectionDistance() const;

    void setMaxDetectionDistance(float value);
    float getMaxDetectionDistance() const;

protected:
    std::unique_ptr<Impl> m_impl;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_Context_hpp__
