/*!
  @file   Sensor.h
  @author David Hirvonen

  @brief  Implementation of a simple sensor/camera abstraction.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_sensor_Sensor_h__
#define __drishti_sensor_Sensor_h__

#include "drishti/sensor/drishti_sensor.h"
#include "drishti/core/Field.h"

#include <opencv2/core/core.hpp>

#include <array>

DRISHTI_SENSOR_NAMESPACE_BEGIN

inline float microToMeters(float microns)
{
    return microns * 1e6f;
}

class SensorModel
{
public:
    // ### Intrisnic camera parameters:
    struct Intrinsic
    {
        Intrinsic();
        Intrinsic(const cv::Point2f& c, float fx, const cv::Size& size);
        float getFocalLength() const { return m_fx; }

        const cv::Size& getSize() const { return *m_size; }
        void setSize(const cv::Size& size) { m_size = size; }

        cv::Matx33f getK() const;
        cv::Point3f getDepth(const std::array<cv::Point2f, 2>& pixels, float widthMeters) const;

        // Sample parameters for iPhone 5s front facing "Photo" mode shown below:
        core::Field<cv::Size> m_size;   // = {640, 852};
        core::Field<cv::Point2f> m_c;   // = {320.f, 426.f};
        core::Field<float> m_fx;        // = 768.0;
        core::Field<float> m_pixelSize; // = 0.0;
    };

    // ### Extrinsic camera parameters:
    struct Extrinsic
    {
        Extrinsic() {}
        Extrinsic(const cv::Matx33f &R) : R(R) {}
        cv::Matx33f R;
    };

    SensorModel() {} // init with defaults

    SensorModel(const float fx)
    {
        m_intrinsic.m_fx = fx;
    }

    SensorModel(const Intrinsic& intrinsic)
        : m_intrinsic(intrinsic)
    {
    }

    SensorModel(const Intrinsic& intrinsic, const Extrinsic& extrinsic)
        : m_intrinsic(intrinsic)
        , m_extrinsic(extrinsic)
    {
    }

    const Intrinsic& intrinsic() const
    {
        return m_intrinsic;
    }

    Intrinsic& intrinsic()
    {
        return m_intrinsic;
    }

    const Extrinsic& extrinsic() const
    {
        return m_extrinsic;
    }

    Extrinsic& extrinsic()
    {
        return m_extrinsic;
    }

protected:
    Intrinsic m_intrinsic;
    Extrinsic m_extrinsic;
};

struct DeviceModel
{
    SensorModel m_sensor;

    std::vector<cv::Point2f> m_points // screen coordinates (wcs/centimeters)
        {
          { -2.0, -3.0 },
          { +2.0, -8.0 },
          { +2.0, -3.0 },
          { -2.0, -8.0 }
        };
};

DRISHTI_SENSOR_NAMESPACE_END

#endif
