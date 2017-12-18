/*! -*-c++-*-
  @file   FacePatchTracker.h
  @author David Hirvonen
  @brief  Face and eye tracking, optical flow, corner detection, etc.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/Face.h"

#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>

#ifndef __drishti_hci_FaceTracker_h__
#define __drishti_hci_FaceTracker_h__

/**
 * @brief Short term face patch tracker
 *
 * Given an input face model, the FacePatchTracker will initialize roughtly
 * pose invariant (pose indexed (landmark interpolated) face patches and
 * perform robust tracking for each successive frame until init() is called
 * again.  This is added to initially provide a stable referene coordinate
 * system from which to measure eye motion (iris centers).
 */

class FacePatchTracker
{
public:
    
    typedef cv::TrackerMedianFlow PatchTracker;
    typedef cv::TrackerMedianFlow::Params PatchParams;

    /**
     * @brief Encapsulate a tracker object, an roi, and a human readable feature name
     */
    struct Tracker
    {
        std::string name;
        cv::Rect2d roi;
        cv::Ptr<PatchTracker> tracker;
    };
    
    /**
     *  Return true if empty.
     */
    bool empty() const;
    
    /**
     * Initialize pose indexed face patches given a face and an input image.
     *
     * @param[in] face Input face model
     * @param[in] image Input face image
     */
    void init(const drishti::face::FaceModel &face, const cv::Mat4b &image);
    
    /**
     * Updated position estimates for set of face patches for the current frame
     *
     * @param[in] face Input face model
     * @param[in] image Input face image
     */
    void update(const drishti::face::FaceModel &frace, const cv::Mat4b &image);
    
    /**
     * Return a rectangle centered on the dorsal bridge for the given face model
     *
     * @param[in] face Input face model
     * @param[in] scale Scale term for created rectangle
     * @return Created rectangle
     */
    cv::Rect2d getDorsalBridgeRoi(const drishti::face::FaceModel &face, float scale) const;
    
    /**
     * Create a rectangle of given size center on the specified point
     *
     * @param[in] center Center of rectangle
     * @param[in] size Size of rectangle to be created.
     * @return Created rectangle.
     */
    static cv::Rect makeRect(const cv::Point &center, const cv::Size &size);
    
    /**
     * Draw tracker patches for current state in input image.
     *
     * @param[in] face Input face model
     * @param[in] image Input face image
     * @return Allocated image with face and track annotations.
     */
    cv::Mat draw(const drishti::face::FaceModel &face, const cv::Mat4b &image);
    
    /**
     * Fill user defined features for the input face model.
     *
     * @param[in,out] Input face model, where Face::Model::userFeatures is updated with named track positions
     */
    void fill(drishti::face::FaceModel &face); // fill in patch features

    /**
     * Return a pose invariant noramlized eye pair
     *
     * @param[in] Input face model
     * @return Output eye pair.
     */
    std::array<cv::Point2f, 2> getFrontalizedEyePair(const drishti::face::FaceModel &faceIn);
    
protected:
    
    /**
     * The initialization parameters used to instantiate each tracked patch
     */
    PatchParams m_params;
    
    /**
     * The set of allocated patch trackers.
     */
    std::vector< Tracker > m_trackers;
};

#endif // __drishti_hci_FaceTracker_h__
