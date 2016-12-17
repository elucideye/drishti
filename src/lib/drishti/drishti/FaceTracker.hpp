/**
  @file   FaceTracker.hpp
  @author David Hirvonen
  @brief  Public API for continuous face filter.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the FaceTracker public API class.
*/

#ifndef __drishti_drishti_FaceTracker_hpp__
#define __drishti_drishti_FaceTracker_hpp__ 1

#include "drishti/drishti_sdk.hpp"
#include "drishti/Image.hpp"
#include "drishti/Face.hpp"
#include "drishti/VideoFrame.hpp"
#include "drishti/Manager.hpp"

#include <memory>

_DRISHTI_SDK_BEGIN

/**
    \class FaceFilter
    \brief This class implements the public API for face filtering.
*/

class DRISHTI_EXPORTS FaceTracker
{
public:
    
    struct Resources
    {
        std::istream *sFaceDetector;
        std::vector<std::istream *> sFaceRegressors;
        std::istream *sEyeRegressor;
        std::istream *sFaceModel;
    };
    
    class Impl;
    
    FaceTracker(Manager *manager, const Resources &factory);

    // FaceTracker cannot be moved or copied:
    FaceTracker(const FaceTracker &) = delete;
    FaceTracker& operator=(const FaceTracker &) = delete;
    FaceTracker(FaceTracker &&) = delete;

    int operator()(const VideoFrame &image);

    ~FaceTracker();

    bool good() const;
    explicit operator bool() const;

protected:

    std::unique_ptr<Impl> m_impl;
};

_DRISHTI_SDK_END

/*
 * Extern "C" interface (dlopen/dlsym)
 */

DRISHTI_EXTERN_C_BEGIN

DRISHTI_EXPORTS drishti::sdk::FaceTracker*
drishti_face_tracker_create_from_file(drishti::sdk::Manager *manager, const drishti::sdk::FaceTracker::Resources &resources);
DRISHTI_EXPORTS void
drishti_face_tracker_destroy(drishti::sdk::FaceTracker *tracker);
DRISHTI_EXPORTS void
drishti_face_tracker_track(drishti::sdk::FaceTracker *tracker, const drishti::sdk::VideoFrame &frame, drishti::sdk::Face &face);

DRISHTI_EXTERN_C_END

#endif // __drishti_drishti_FaceTracker_hpp__
