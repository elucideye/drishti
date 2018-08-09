/*! -*-c++-*-
  @file   exceptions.hpp
  @brief  Exception definition

*/

#ifndef __facefilter_Exception_hpp__
#define __facefilter_Exception_hpp__ 1

#include <facefilter/facefilter.h> // BEGIN_FACEFILTER_NAMESPACE

BEGIN_FACEFILTER_NAMESPACE

class Exception
{
public:
    enum Type
    {
        INSTANTIATE_TRACKER,
        FILE_OPEN,
    };

    explicit Exception(Type type)
        : type_(type)
    {
    }

private:
    Type type_;
};

END_FACEFILTER_NAMESPACE

#endif // __facefilter_Exception_hpp__ 1
