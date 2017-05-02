/**
  @file   FaceMonitorAdapter.h
  @author David Hirvonen
  @brief  Private utility class to map extern "C" callbacks to C++ API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

 */

#ifndef __drishti_drishti_FaceMonitorAdapter_h__
#define __drishti_drishti_FaceMonitorAdapter_h__ 1

#include "drishti/drishti_sdk.hpp"
#include "drishti/drishti_cv.hpp"
#include "drishti/EyeSegmenterImpl.hpp"

#include "drishti/face/Face.h"
#include "drishti/hci/FaceFinder.h"

_DRISHTI_SDK_BEGIN

/*
 * FaceMonitorAdapter
 *
 * Provide an interface for converting drishti::hc::FaceMonitor output to the public extern "C" API
 */

struct FaceMonitorAdapter : public drishti::hci::FaceMonitor
{
public:
    using HighResolutionClock = std::chrono::high_resolution_clock;
    using TimePoint = HighResolutionClock::time_point; // <std::chrono::system_clock>;

    FaceMonitorAdapter(drishti_face_tracker_t& table)
        : m_start(HighResolutionClock::now())
        , m_table(table)
    {
    }

    ~FaceMonitorAdapter()
    {
    }

    virtual bool isValid(const cv::Point3f& position, const TimePoint& timeStamp)
    {
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(timeStamp - m_start).count();
        return m_table.trigger(m_table.context, cvToDrishti(position), elapsed);
    }

    virtual void grab(const std::vector<FaceImage>& frames, bool isInitialized)
    {
        // Populate public API buffer using public SDK wrapper types w/ shallow copy:
        drishti::sdk::Array<drishti_face_tracker_result_t, 64> results(frames.size());

        for (size_t i = 0; i < frames.size(); i++)
        {
            if (!frames[i].image.empty())
            {
                results[i].image = cvToDrishti<cv::Vec4b, drishti::sdk::Vec4b>(frames[i].image);
            }

            if (!frames[i].eyes.empty())
            {
                results[i].eyes = cvToDrishti<cv::Vec4b, drishti::sdk::Vec4b>(frames[i].eyes);
            }

            if (!frames[i].extra.empty())
            {
                results[i].extra = cvToDrishti<cv::Vec4b, drishti::sdk::Vec4b>(frames[i].extra);
            }

            for (int j = 0; j < 2; j++)
            {
                if (frames[i].eyeModels[j].eyelids.size())
                {
                    results[i].eyeModels[j] = drishti::sdk::convert(frames[i].eyeModels[j]);
                }
            }

            // TODO:
            //results[i].faceModels;
        }

        m_table.callback(m_table.context, results);
    }

protected:
    TimePoint m_start;
    drishti_face_tracker_t m_table;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_FaceMonitorAdapter_hpp__
