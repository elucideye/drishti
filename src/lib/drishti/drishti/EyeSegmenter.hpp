/**
  @file   EyeSegmenter.hpp
  @author David Hirvonen
  @brief  Public API for eye model estimation using simple portable types.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the Eyesegmenter public API class for eye model estimation.
*/

#ifndef __drishti_drishti_EyeSegmenter_hpp__
#define __drishti_drishti_EyeSegmenter_hpp__

#include "drishti/drishti_sdk.hpp" // TODO: get rid of this
#include "drishti/Image.hpp"
#include "drishti/Eye.hpp"

#include <memory> // unique_ptr, shared_ptr
#include <vector> // for eyelid contour

_DRISHTI_SDK_BEGIN

/**
    \class EyeSegmenter
    \brief This class implements the public API for eye model estimation.

*/

enum ArchiveKind
{
    kCPB,
    kAuto // automatic
};

class DRISHTI_EXPORT EyeSegmenter
{
public:
    class Impl;
    EyeSegmenter(const std::string& filename, ArchiveKind kind = kAuto);
    EyeSegmenter(std::istream& is, ArchiveKind kind = kAuto);

    // EyeSegmenter cannot be moved or copied:
    EyeSegmenter(const EyeSegmenter&) = delete;
    EyeSegmenter& operator=(const EyeSegmenter&) = delete;
    EyeSegmenter(EyeSegmenter&&) = delete;

    ~EyeSegmenter();

    bool good() const;

    explicit operator bool() const;

    int operator()(const Image3b& image, Eye& eye, bool isRight);
    Eye getMeanEye(int width) const;

    void setEyelidInits(int count);
    int getEyelidInits() const;

    void setIrisInits(int count);
    int getIrisInits() const;

    int getMinWidth() const;

    void setOptimizationLevel(int level);

    // Aspect ratio as width/height
    float getRequiredAspectRatio() const;

protected:
    void init(std::istream& is, ArchiveKind kind = kAuto);
    std::unique_ptr<Impl> m_impl;
};

_DRISHTI_SDK_END

/*
 * Extern "C" interface (dlopen/dlsym)
 */

DRISHTI_EXTERN_C_BEGIN

DRISHTI_EXPORT drishti::sdk::EyeSegmenter*
drishti_eye_segmenter_create_from_file(const std::string& filename, drishti::sdk::ArchiveKind kind);

DRISHTI_EXPORT drishti::sdk::EyeSegmenter*
drishti_eye_segmenter_create_from_stream(std::istream& is, drishti::sdk::ArchiveKind kind);

DRISHTI_EXPORT void
drishti_eye_segmenter_destroy(drishti::sdk::EyeSegmenter* segmenter);

DRISHTI_EXPORT int
drishti_eye_segmenter_segment(
    drishti::sdk::EyeSegmenter* segmenter,
    const drishti::sdk::Image3b& image,
    drishti::sdk::Eye& eye, bool isRight);

DRISHTI_EXTERN_C_END

#endif /* defined(__drishti_drishti_EyeSegmenter_hpp__) */
