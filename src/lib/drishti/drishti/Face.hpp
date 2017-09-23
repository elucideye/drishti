/*! -*-c++-*-
  @file   Face.hpp
  @author David Hirvonen
  @brief  Top level API eye model declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the eye model structure used
  to report results for the top level SDK.
*/

#ifndef __drishti_drishti_Face_hpp__
#define __drishti_drishti_Face_hpp__ 1

#include "drishti/drishti_sdk.hpp"
#include "drishti/Eye.hpp"
#include "drishti/Image.hpp"
#include "drishti/Array.hpp"

_DRISHTI_SDK_BEGIN

/**
 * @brief A simple face model class.
 *
 * This model contains a face bounding box, landmarks, detailed eye models
 * and an estimated 3D position for the midpoint between the eyes.
 */

struct DRISHTI_EXPORT Face
{
    /**
     * A region of interest, typically provided by an object detector.
     */
    drishti::sdk::Recti roi;
    
    /**
     * Detailed eye models for the subject's right and left eyes.
     */
    drishti::sdk::Array<drishti::sdk::Eye, 2> eyes;
    
    /**
     * Face landmarks provided for the operative annotation style.
     */
    drishti::sdk::Array<drishti::sdk::Vec2f, 128> landmarks;
    
    /**
     * Estimated 3D position for point between the eyes.
     */
    drishti::sdk::Vec3f position;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_Face_hpp__
