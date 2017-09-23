/*! -*-c++-*-
  @file   drishti/hci/FaceMonitor.h
  @author David Hirvonen
  @brief Simple position dependent frame grabbing callback API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_FaceMonitor_h__
#define __drishti_hci_FaceMonitor_h__

#include "drishti/hci/drishti_hci.h"
#include "drishti/face/Face.h"
#include "drishti/eye/Eye.h"
#include "drishti/core/ImageView.h" // Image and/or Texture

#include <opencv2/core/core.hpp>

#include <array>  // std::array<>
#include <vector> // std::vector<>
#include <chrono> // std::chrono::time_point

DRISHTI_HCI_NAMESPACE_BEGIN

class FaceMonitor
{
public:

    //! An alias for the system high resolution clock
    using HighResolutionClock = std::chrono::high_resolution_clock;

    //! An alias for a time point from the high resolution clock
    using TimePoint = HighResolutionClock::time_point;
    
    //! An alias for a vector of positions
    using Positions = std::vector<cv::Point3f>;

    //! An alias for a vector of positions
    using Faces = std::vector<drishti::face::FaceModel>;
    
    /**
     * A face image structure containing metadata with associated l
     * andmarks and eye images.
     */
    
    struct FaceImage
    {
        //! Timestamp corresponding to acquisition time.
        TimePoint time;

        //! A captured face image.
        core::ImageView image;

        //! A vector of face models within the captured image
        std::vector<drishti::face::FaceModel> faceModels;

        //! Eye pair image [ left | right ]
        core::ImageView eyes;

        //! An array of eye models in the coordinate system of the eyes image.
        std::array<drishti::eye::EyeModel, 2> eyeModels;

        //! Additional image data (reserved).
        core::ImageView filtered;
    };

    struct Request
    {
        Request() = default;        
        Request(int n, bool getImage, bool getTexture)
            : n(n)
            , getImage(getImage)
            , getTexture(getTexture)
        {
            
        }
        
        int n = 0;                //! Number of frames requested (last n)
        bool getImage = false;    //! Request an image (typically incurs some overhead)
        bool getTexture = false;  //! Request a texture (typically no overhead)
        
        Request & operator |=(const Request &src)
        {
            n = std::max(n, src.n);
            getTexture |= src.getTexture;
            getImage |= src.getImage;
            return (*this);
        }
    };
    
    /**
     * A user defined virtual method callback that should report the number
     * of frames that should be captured from teh FIFO buffer based on the 
     * reported face location.
     * @param faces a vector of faces for the current frame
     * @param timestmap the acquisition timestamp for the frame
     * @return a frame request for the last n frames with requested image formats
     */
    virtual Request request(const Faces& faces, const TimePoint& timeStamp) = 0;

    /**
     * A user defined virtual method callback that will be called with a
     * a populated vector of FaceImage objects for the last N frames, where
     * N is the number of frames requested in the preceding request callback.
     * @param frames A vector containing the last N consecutive FaceImage objects
     * @param isInitialized Return true if the FIFO buffer is fully initialized.
     */    
    virtual void grab(const std::vector<FaceImage>& frames, bool isInitialized) = 0;
};

DRISHTI_HCI_NAMESPACE_END

#endif // __drishti_hci_FaceMonitor_h__
