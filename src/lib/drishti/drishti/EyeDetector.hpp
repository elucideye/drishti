/**
  @file   EyeDetector.hpp
  @author David Hirvonen
  @brief  Top level eye detection API class declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the eye detection class used
  for the top level SDK.
*/

#ifndef __drishti_drishti_EyeDetector_hpp__
#define __drishti_drishti_EyeDetector_hpp__

#include "drishti/drishti_sdk.hpp"
#include "drishti/Image.hpp"

#include <memory> // unique_ptr, shared_ptr
#include <vector> // for eyelid contour

_DRISHTI_SDK_BEGIN

/*
 * EyeDetector
 */

class DRISHTI_EXPORT EyeDetector
{
public:
    struct Rect
    {
        Rect(int x, int y, int width, int height)
            : x(x)
            , y(y)
            , width(width)
            , height(height)
        {
        }
        int x, y, width, height;
    };

    class Impl;

    EyeDetector();
    EyeDetector(const std::string& filename);
    ~EyeDetector();
    int operator()(const Image3b& image, std::vector<Rect>& objects);

protected:
    void init(const std::string& filename);
    std::unique_ptr<Impl> m_impl;
};

_DRISHTI_SDK_END

#endif /* defined(__drishti_drishti_EyeDetector_hpp__) */
