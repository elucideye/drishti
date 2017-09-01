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
#include "drishti/Context.hpp"

#include <memory>

struct drishti_image_tex_t
{
    drishti::sdk::Texture texture;
    drishti::sdk::Image4b image;
};

struct drishti_face_tracker_result_t
{
    double time; //TimePoint time;

    drishti_image_tex_t image;
    drishti::sdk::Array<drishti::sdk::Face, 2> faceModels;

    drishti_image_tex_t eyes; // eye pair image [ left | right ]
    drishti::sdk::Array<drishti::sdk::Eye, 2> eyeModels;

    drishti::sdk::Image4b filtered;
};

DRISHTI_EXTERN_C_BEGIN

typedef struct drishti_image
{
    size_t width;
    size_t height;
    size_t channels;
} drishti_image_t;

typedef struct drishti_request
{
    int n;
    bool getImage;
    bool getTexture;
} drishti_request_t;

typedef drishti::sdk::Array<drishti_face_tracker_result_t, 64> drishti_face_tracker_results_t;

// User defined callbacks:
typedef int (*drishti_face_tracker_callback_t)(void* context, drishti_face_tracker_results_t& results);
typedef drishti_request_t (*drishti_face_tracker_trigger_t)(void* context, const drishti::sdk::Vec3f& point, double timestamp);
typedef int (*drishti_face_tracker_allocator_t)(void* context, const drishti_image_t& spec, drishti::sdk::Image4b& image);

typedef struct drishti_face_tracker
{
    void* context;
    drishti_face_tracker_trigger_t trigger;
    drishti_face_tracker_callback_t callback;
    drishti_face_tracker_allocator_t allocator;
} drishti_face_tracker_t;

DRISHTI_EXTERN_C_END

_DRISHTI_SDK_BEGIN

/**
    \class FaceFilter
    \brief This class implements the public API for face filtering.
*/

class DRISHTI_EXPORT FaceTracker
{
public:
    // Maintain raw pointers as this code may need to interface with non-coypable istream sources.
    // The referenced streams must stay in scope for the scope of the construction.
    struct Resources
    {
        std::istream* sFaceDetector;
        std::istream* sFaceRegressor;
        std::istream* sEyeRegressor;
        std::istream* sFaceModel;
        std::string logger; // logger name
    };

    class Impl;

    FaceTracker(Context* manager, Resources& factory);

    // FaceTracker cannot be moved or copied:
    FaceTracker(const FaceTracker&) = delete;
    FaceTracker& operator=(const FaceTracker&) = delete;
    FaceTracker(FaceTracker&&) = delete;

    static void tryEnablePlatformOptimizations();

    int operator()(const VideoFrame& image);

    ~FaceTracker();

    bool good() const;
    explicit operator bool() const;

    void add(drishti_face_tracker_t& table);

protected:
    std::unique_ptr<Impl> m_impl;
};

_DRISHTI_SDK_END

/*
 * Extern "C" interface (dlopen/dlsym)
 */

DRISHTI_EXTERN_C_BEGIN

DRISHTI_EXPORT drishti::sdk::FaceTracker*
drishti_face_tracker_create_from_streams(drishti::sdk::Context* manager, drishti::sdk::FaceTracker::Resources& resources);

DRISHTI_EXPORT void
drishti_face_tracker_destroy(drishti::sdk::FaceTracker* tracker);

DRISHTI_EXPORT int
drishti_face_tracker_track(drishti::sdk::FaceTracker* tracker, const drishti::sdk::VideoFrame& frame);

DRISHTI_EXPORT int
drishti_face_tracker_callback(drishti::sdk::FaceTracker* tracker, drishti_face_tracker_t& table);

DRISHTI_EXTERN_C_END

#endif // __drishti_drishti_FaceTracker_hpp__
