/**
  @file   FaceTracker.cpp
  @author David Hirvonen
  @brief  Implementation of public API for eye model estimation using simple portable types.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

 */

#include "drishti/FaceTracker.hpp"

#include "drishti/face/Face.h"
#include "drishti/hci/FaceFinder.h"
#include "drishti/core/make_unique.h"

#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>

_DRISHTI_SDK_BEGIN

/*
 * Impl
 */

static ogles_gpgpu::FrameInput convert(const VideoFrame &frame)
{
    ogles_gpgpu::FrameInput input;
    input.inputTexture = frame.inputTexture;
    input.pixelBuffer = frame.pixelBuffer;
    input.size = { frame.size[0] ,frame.size[1] };
    input.textureFormat = frame.textureFormat;
    input.useRawPixels = frame.useRawPixels;
    return input;
}

class FaceTracker::Impl
{
public:

    using Config = drishti::hci::FaceFinder::Config;

    Impl(Manager *manager, const FaceTrackerResources &resources)
    {
        Config config;
        
        assert(false);

        // TODO: convert resources to factory:
        std::shared_ptr<drishti::face::FaceDetectorFactory> factory;
        
        m_faceFinder = drishti::core::make_unique<drishti::hci::FaceFinder>(factory, config, nullptr);
    }
    
    int operator()(const VideoFrame &frame)
    {
        return (*m_faceFinder)(convert(frame));
    }
    
protected:
    
    std::unique_ptr<drishti::hci::FaceFinder> m_faceFinder;
};

/*
 * FaceTracker
 */

int FaceTracker::operator()(const VideoFrame &image)
{
    return (*m_impl)(image);
}

FaceTracker::FaceTracker(Manager *manager, const FaceTrackerResources &factory)
{
    m_impl = drishti::core::make_unique<Impl>(manager, factory);
}

FaceTracker::~FaceTracker()
{

}

bool FaceTracker::good() const
{
    return static_cast<bool>(m_impl.get()); 
}

FaceTracker::operator bool() const
{
    return good();
}

_DRISHTI_SDK_END

/*
 * Extern "C" interface (dlopen/dlsym)
 */

DRISHTI_EXTERN_C_BEGIN

drishti::sdk::FaceTracker* drishti_face_tracker_create_from_file(drishti::sdk::Manager *manager, const FaceTrackerResources &resources)
{
    return new drishti::sdk::FaceTracker(manager, resources);
}

void drishti_face_tracker_destroy(drishti::sdk::FaceTracker *tracker)
{
    if(tracker)
    {
        delete tracker;
    }
}

void drishti_face_tracker_track(drishti::sdk::FaceTracker *tracker, const drishti::sdk::VideoFrame &image, drishti::sdk::Face &face)
{
    (*tracker)(image);
}

DRISHTI_EXTERN_C_END
