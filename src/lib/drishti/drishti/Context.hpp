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
#include "drishti/Sensor.hpp"

#include <memory>

_DRISHTI_SDK_BEGIN

class DRISHTI_EXPORT Context
{
public:
    struct Impl;

    Context(drishti::sdk::SensorModel& sensor);
    ~Context();

    Impl* get() { return impl.get(); }

    void setDoSingleFace(bool flag);
    bool getDoSingleFace() const;

    void setMinDetectionDistance(float value);
    float getMinDetectionDistance() const;

    void setMaxDetectionDistance(float value);
    float getMaxDetectionDistance() const;

    void setFaceFinderInterval(float value);
    float getFaceFinderInterval() const;

    void setAcfCalibration(float value);
    float getAcfCalibration() const;

    void setRegressorCropScale(float value);
    float getRegressorCropScale() const;

    void setMinTrackHits(int hits);
    int getMinTrackHits() const;

    void setMaxTrackMisses(int hits);
    int getMaxTrackMisses() const;

    void setMinFaceSeparation(float value);
    float getMinFaceSeparation() const;

    void setDoOptimizedPipeline(bool flag);
    bool getDoOptimizedPipeline() const;

protected:
    std::unique_ptr<Impl> impl;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_Context_hpp__
