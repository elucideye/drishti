/*! -*-c++-*-
  @file  FaceTrackerTest.cpp
  @brief Implementation of virtual callback "table" for drishti FaceTracker public interface.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
*/

#include "FaceTrackerTest.h"

#include <facefilter/make_unique.h>

BEGIN_FACEFILTER_NAMESPACE

struct FaceTrackTest::Impl
{
    Impl(std::shared_ptr<spdlog::logger>& logger, const std::string& output)
    {
        // CUSTOM user constructor
    }

    ~Impl() = default;

    // CUSTOM user fields
};

// See: https://github.com/elucideye/drishti/blob/master/src/lib/drishti/drishti/ut/test-FaceTracker.cpp
FaceTrackTest::FaceTrackTest(std::shared_ptr<spdlog::logger>& logger, const std::string& sOutput)
{
    m_impl = facefilter::make_unique<Impl>(logger, sOutput);
}

FaceTrackTest::~FaceTrackTest() = default;

int FaceTrackTest::callback(drishti_face_tracker_results_t& results)
{
    //m_impl->logger->info("callback: Received results");

    if (results.size() > 0)
    {
        // If you have arrived here, then a valid frame request was performed in
        // the trigger() callback based on available face images.
    }

    return 0;
}

drishti_request_t
FaceTrackTest::trigger(const drishti_face_tracker_result_t& faces, double timestamp, std::uint32_t tex)
{
    //m_impl->logger->info("trigger: Received results at time {}}", timestamp);

    // User provided face monitor would go here.
    
    return // formulate a frame request based on input faces
    {
        0, // Retrieve the last N frames.
        false, // Get frames in user memory.
        true,  // Get OpenGL textures.
        true,  // Grab full frame images/textures.
        true,  // Grab eye pair images/textures.
    };

}

int FaceTrackTest::allocator(const drishti_image_t& spec, drishti::sdk::Image4b& image)
{
    //m_impl->logger->info("allocator: {} {}", spec.width, spec.height);
    
    return 0;
}

int FaceTrackTest::callbackFunc(void* context, drishti_face_tracker_results_t& results)
{
    if (FaceTrackTest* ft = static_cast<FaceTrackTest*>(context))
    {
        return ft->callback(results);
    }
    return -1;
}

drishti_request_t
FaceTrackTest::triggerFunc(void* context, const drishti_face_tracker_result_t& faces, double timestamp, std::uint32_t tex)
{
    if (FaceTrackTest* ft = static_cast<FaceTrackTest*>(context))
    {
        return ft->trigger(faces, timestamp, tex);
    }
    return { 0, false, true, true, true }; // compiler warnings
}

int FaceTrackTest::allocatorFunc(void* context, const drishti_image_t& spec, drishti::sdk::Image4b& image)
{
    if (FaceTrackTest* ft = static_cast<FaceTrackTest*>(context))
    {
        return ft->allocator(spec, image);
    }
    return -1;
}

END_FACEFILTER_NAMESPACE
