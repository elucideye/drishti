/*!
  @file   FaceTrackerTest.h
  @author David Hirvonen
  @brief  Sample eye/face tracking using drishti.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __FaceTrackerTest_h__
#define __FaceTrackerTest_h__

#include <drishti/FaceTracker.hpp>
#include <drishti/drishti_cv.hpp>

#include <spdlog/spdlog.h> // for portable logging

#include <memory>

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
    static int allocatorFunc
    (
        void* context,
        const drishti_image_t& spec,
        drishti::sdk::Image4b& image
    );

    int callback(drishti::sdk::Array<drishti_face_tracker_result_t, 64>& results);
    static int callbackFunc
    (
        void* context,
        drishti::sdk::Array<drishti_face_tracker_result_t, 64>& results
     );

    drishti_request_t trigger
    (
        const drishti_face_tracker_result_t& faces,
        double timestamp,
        std::uint32_t tex
    );
    
    static drishti_request_t triggerFunc
    (
        void* context,
        const drishti_face_tracker_result_t& faces,
        double timestamp,
        std::uint32_t tex
    );
    // }

    // A user definable stack operation, with a default implementatino that performs
    // simple logging for the purpose of illustration.
    virtual void process(StackType& stack);

    // Logging: {
    void setCaptureSphere(const std::array<float, 3>& center, float radius, double seconds);
    bool shouldCapture(const drishti_face_tracker_result_t& faces);
    // }

    // Utility methods: {
    void initPreview(const cv::Size& size, GLenum textureFormat);
    void setPreviewGeometry(float tx, float ty, float sx, float sy);
    void setSizeHint(const cv::Size& size);
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

#endif // __FaceTrackerTest_h__
