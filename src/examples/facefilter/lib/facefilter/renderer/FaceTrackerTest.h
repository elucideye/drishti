/*! -*-c++-*-
  @file  FaceTrackerTest.h
  @brief Declaration of virtual callback "table" for drishti FaceTracker public interface.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
*/

#ifndef __facefilter_renderer_FaceTrackerTest_h__
#define __facefilter_renderer_FaceTrackerTest_h__

#include <facefilter/facefilter.h>
#include <drishti/FaceTracker.hpp>
#include <drishti/drishti_cv.hpp>

#include <spdlog/spdlog.h> // for portable logging

#include <memory>

BEGIN_FACEFILTER_NAMESPACE

// See: https://github.com/elucideye/drishti/blob/master/src/lib/drishti/drishti/ut/test-FaceTracker.cpp
class FaceTrackTest
{
public:
    struct FrameStorage
    {
        cv::Mat frame;
        cv::Mat eyes;
        drishti_face_tracker_result_t result;
    };
    using StackType = std::vector<FrameStorage>;

    FaceTrackTest(std::shared_ptr<spdlog::logger>& logger, const std::string& sOutput);
    ~FaceTrackTest();

    // Callback definitions: {
    int allocator(const drishti_image_t& spec, drishti::sdk::Image4b& image);
    static int allocatorFunc(void* context, const drishti_image_t& spec, drishti::sdk::Image4b& image);

    int callback(drishti_face_tracker_results_t& results);
    static int callbackFunc(void* context, drishti_face_tracker_results_t& results);

    drishti_request_t trigger(const drishti_face_tracker_result_t& faces, double timestamp, std::uint32_t tex);
    static drishti_request_t triggerFunc(void* context, const drishti_face_tracker_result_t& faces, double timestamp, std::uint32_t tex);
    // }

    // Define the public callback table:
    drishti_face_tracker_t table{
        this,
        triggerFunc,
        callbackFunc,
        allocatorFunc
    };

protected:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

END_FACEFILTER_NAMESPACE

#endif // __facefilter_renderer_FaceTrackerTest_h__
