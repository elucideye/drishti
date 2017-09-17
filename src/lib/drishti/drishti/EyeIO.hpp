/*! -*-c++-*-
  @file   EyeIO.hpp
  @author David Hirvonen
  @brief  Eye model serialization.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Contains serializaiton and ostream operators for the Eye model. 

  NOTE: Use of this file requires full ABI compatibility: 
  * libstdc++
  * libc++ 
  * etc

*/

#ifndef __drishti_drishti_EyeIO_hpp__
#define __drishti_drishti_EyeIO_hpp__

#include "drishti/drishti_sdk.hpp"
#include "drishti/Eye.hpp"

#include <vector>
#include <iostream>

_DRISHTI_SDK_BEGIN

struct DRISHTI_EXPORT EyeStream
{
    enum Format
    {
        XML,
        JSON
    };
    EyeStream(const Format& format)
        : format(format)
    {
    }
    std::string ext() const;
    Format format = XML;
};

struct DRISHTI_EXPORT EyeOStream : public EyeStream
{
    EyeOStream(const Eye& eye, Format format)
        : EyeStream(format)
        , eye(eye)
    {
    }
    const Eye& eye;
};

struct DRISHTI_EXPORT EyeIStream : public EyeStream
{
    EyeIStream(Eye& eye, Format format)
        : EyeStream(format)
        , eye(eye)
    {
    }
    Eye& eye;
};

DRISHTI_EXPORT std::ostream& operator<<(std::ostream& os, const EyeOStream& eye);
DRISHTI_EXPORT std::istream& operator>>(std::istream& is, EyeIStream& eye);

_DRISHTI_SDK_END

#endif // __drishti_drishti_EyeIO_hpp__
