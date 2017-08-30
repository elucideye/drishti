/*!
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

/*
 * Simple Face description
 */

struct DRISHTI_EXPORT Face
{
    drishti::sdk::Array<drishti::sdk::Eye, 2> eyes;
    drishti::sdk::Array<drishti::sdk::Vec2f, 128> landmarks;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_Face_hpp__
