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

/**
 * @brief An image container with an OpenGL texture and/or memory buffer
 *
 * A simple image container class with fields for the OpenGL texture
 * and/or a standard row major memory buffer.
 *
 */
struct drishti_image_tex_t
{
    /**
     * An OpenGL texture container class.
     */
    drishti::sdk::Texture texture;

    /**
     * A 32-bit RGBA image container class.
     */
    drishti::sdk::Image4b image;
};

struct drishti_face_tracker_result_t
{
    /**
     * Acquisition time of the contained frame.
     */
    double time; // TimePoint

    /**
     * The image description (memory and/or OpenGL texture)
     */
    drishti_image_tex_t image;
    
    /**
     * Face models for the current frame.
     */
    drishti::sdk::Array<drishti::sdk::Face, 2> faceModels;

    /**
     * A filtered eye image pair (subject's rigth and left eyes) for the nearest face in the scene.
     */
    drishti_image_tex_t eyes; // eye pair image [ left | right ]
    
    /**
     * Detailed eye models corresponding to the filtered eye pair image.
     */
    drishti::sdk::Array<drishti::sdk::Eye, 2> eyeModels;

    /**
     * Reserved image.
     */
    drishti::sdk::Image4b filtered;
};

DRISHTI_EXTERN_C_BEGIN

/**
 * @brief An image format specification
 *
 * Provides an image specification sufficient for memory allocation.
 */

typedef struct drishti_image
{
    /**
     * Image width in pixels
     */
    std::size_t width;
    
    /**
     * Image height in pixels
     */
    std::size_t height;
    
    /**
     * Number of channels
     */
    std::size_t channels;
    
    /**
     * Row stride in bytes
     */
    std::size_t stride;
    

} drishti_image_t;

/**
 * @brief A "request" object specifying the # of frames to retrieve and
 * the desired format: (1) OpenGL texture or; (2) user memory.
 */

typedef struct drishti_request
{
    /**
     * Retrieve the last N frames.
     */
    int n;
    
    /**
     * Get frames in user memory.
     */
    bool getImage;
    
    /**
     * Get OpenGL textures.
     */
    bool getTexture;
    
} drishti_request_t;

/**
 * An alias for a vector of drishti_face_tracker_result_t objects.
 */

typedef drishti::sdk::Array<drishti_face_tracker_result_t, 64> drishti_face_tracker_results_t;

// User defined callbacks:

/** 
 * @brief Returns face detections for current frame
 *
 * @param context Allocated context with internal library state.
 * @param results A vector containing frames + associated face metadata for the past N frames
 * @return Error code (reserved).
 */
typedef int (*drishti_face_tracker_callback_t)(void* context, drishti_face_tracker_results_t& results);

/** 
 * @brief This function is called (back) with the face+eye models for the current frame.
 * 
 * This function is called for each frame with a list of detected faces.  The user provided
 * implementation must assess the face models and determine if any additional history is
 * required.  The user must then populate and return a <drishti_request_t> which can retrieve
 * a history of images.  Typically fetching the full image history from the GPU is expensive,
 * and this mechanism allows a lazy retrieval of frames for interesting cases (as determined
 * by the user).
 *
 * @param context Allocated context with internal library state.
 * @param faces A list of face models for the current frame.
 * @param timestamp The acquisition time for the reported frame results.
 * @return A request structure for frame history + associated metadata.
 */
typedef drishti_request_t (*drishti_face_tracker_update_t)(void* context, const drishti_face_tracker_result_t& faces, double timestamp);

/** 
 * @brief Allocation routine.
 *
 * This is a user provided function to perform image allocation in cases where images are requested.
 * The library does not return any allocated memory across the API boundary.
 *
 * @param context Allocated context with internal library state.
 * @param spec A specification describing how the image to be allocated.
 * @return Error code (reserved).
 */
typedef int (*drishti_face_tracker_allocator_t)(void* context, const drishti_image_t& spec, drishti::sdk::Image4b& image);

/**
 * @brief A table of user defined callbacks to assist in face tracking.
 *
 * For each frame the <update> callback will provide a description of the
 * detected faces in the scene (if any), and the user provided callback
 * can return a <drishti_request_t> containing a request for the past N
 * frames as Opengl textures (fast) and/or standard user memory images (slower).
 * An additional callback allows the user to perform memory allocation, since
 * drishti does not exported allocated memory across the SDK boundary.
 */

typedef struct drishti_face_tracker
{
    /**
     * A pointer to allocated FaceTracker state.
     */
    void* context;
    
    /**
     * A callback containins a list of all detected faces (if any) for each frame.
     * Note: This will be called for every frame (after an initializationp period)
     * even if no faces are deteted in the current frame.
     */
    drishti_face_tracker_update_t update;
    
    /**
     * A callback returning frames requested by the <update> functino.
     */
    drishti_face_tracker_callback_t callback;
    
    /**
     * A callback for memory allocation.
     */
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

    /**
     * Constructor
     *
     * @param context Face tracker configuration and state details
     * @param factory Container of istream objects sufficient to allocate a 
     * face tracker and eye model fitting
     */
    FaceTracker(Context* context, Resources& factory);

    // FaceTracker cannot be moved or copied:
    FaceTracker(const FaceTracker&) = delete;
    FaceTracker& operator=(const FaceTracker&) = delete;
    FaceTracker(FaceTracker&&) = delete;
    
    /**
     * Destructor
     */
    ~FaceTracker();

    /**
     * Try to configuration platform GPGPU platform optimizations.
     */
    static void tryEnablePlatformOptimizations();

    /**
     * Runs face tracking on a single input video frame.
     *
     * @param image The input video frame object.
     */
    int operator()(const VideoFrame& image);

    /**
     * Return true if object was successfully allocated.
     */
    bool good() const;
    
    /**
     * Returns true if object was successfully allcoated.
     */
    explicit operator bool() const;

    /**
     * Install a set of user defined callback functions to manage tracking.
     * 
     * @param table The set of user defined callbacks for tracking.
     */
    void add(drishti_face_tracker_t& table);

protected:
    
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

_DRISHTI_SDK_END

/*
 * Extern "C" interface (dlopen/dlsym)
 */

DRISHTI_EXTERN_C_BEGIN

/**
 * @brief Stream based allocation of FaceTracker
 *
 * Allocates a FaceTracker object given a pointer to a pre-allocated Context.
 *
 * @param context Allocated context with internal library state.
 * @param resources A container with istream pointers for various detector and regressor models.
 * @return The allocated FaceTracker object.
 */

DRISHTI_EXPORT drishti::sdk::FaceTracker*
drishti_face_tracker_create_from_streams(drishti::sdk::Context* manager, drishti::sdk::FaceTracker::Resources& resources);

/**
 * @brief Stream based deallocation of FaceTracker
 *
 * Deallocates a FaceTracker object
 *
 * @param tracker The FaceTracker object to be deallocated.
 */

DRISHTI_EXPORT void
drishti_face_tracker_destroy(drishti::sdk::FaceTracker* tracker);

/**
 * @brief Perform face tracking for a single frame.
 *
 * Receive an allocated FaceTracker and input frame and perform a single step of detection + tracking.
 *
 * @param tracker The FaceTracker object
 * @param frame The video frame for face detection + tracking
 * @param return Error code.
 */

DRISHTI_EXPORT int
drishti_face_tracker_track(drishti::sdk::FaceTracker* tracker, const drishti::sdk::VideoFrame& frame);

/**
 * @brief Define a set of callbacks to return face tracking results.
 *
 * A container of function pointers is sent to the allocated FaceTracker
 * object to report results, and optional retrieve frame + face history
 * for interesting cases.
 *
 * @see drishti_face_tracker_allocator_t
 * @see drishti_face_tracker_update_t
 * @see drishti_face_tracker_callback_t
 *
 * @param tracker The FaceTracker object
 * @param frame The video frame for face detection + tracking
 * @param return Error code.
 */

DRISHTI_EXPORT int
drishti_face_tracker_callback(drishti::sdk::FaceTracker* tracker, drishti_face_tracker_t& table);

DRISHTI_EXTERN_C_END

#endif // __drishti_drishti_FaceTracker_hpp__
