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

#include <array>
#include <iostream>

_DRISHTI_SDK_BEGIN

/*
 * Eye type
 */

class DRISHTI_EXPORT Face
{
public:

    Face() {}

    Eye & getEye(int i) { return m_eyes[i]; }
    const Eye & getEye(int i) const { return m_eyes[i]; }

protected:

    std::array<Eye,2> m_eyes;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_Face_hpp__
