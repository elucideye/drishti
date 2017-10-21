/*! -*-c++-*-
  @file   FaceTracker.h
  @author David Hirvonen
  @brief  Declaration of a face landmark tracker.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceTracker_h__
#define __drishti_face_FaceTracker_h__

#include "drishti/face/drishti_face.h"
#include "drishti/face/Face.h" // FaceModel.h

#include <memory>

DRISHTI_FACE_NAMESPACE_BEGIN

class FaceTracker
{
public:
    struct TrackInfo
    {
        TrackInfo() = default;
        TrackInfo(std::size_t identifier)
            : identifier(identifier)
            , age(1)
            , hits(1)
            , misses(0)
        {
        }

        void hit()
        {
            age++;
            hits++;
            misses = 0;
        }

        void miss()
        {
            age++;
            hits = 0;
            misses++;
        }

        std::size_t identifier = 0;
        std::size_t age = 0;
        std::size_t hits = 0;   // consecutive hits
        std::size_t misses = 0; // consecutive misses
    };

    using FaceTrack = std::pair<drishti::face::FaceModel, TrackInfo>;
    using FaceTrackVec = std::vector<FaceTrack>;
    using FaceModelVec = std::vector<drishti::face::FaceModel>;

    struct Impl;

    FaceTracker(float costThreshold = 0.15f, std::size_t minTrackHits = 3, std::size_t maxTrackMisses = 3);
    ~FaceTracker();
    void operator()(const FaceModelVec& facesIn, FaceTrackVec& facesOut);

protected:
    std::unique_ptr<Impl> m_impl;
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceDetectorAndTracker_h__
