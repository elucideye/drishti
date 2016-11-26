/*!
  @file   Field.h
  @author David Hirvonen
  @brief  Declaration of optional (de)serializable value

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
 
  NOTE: GENERIC_NVP should be defined prior to including this class for either
  boost or cereal archives.

*/

#ifndef __drishtisdk__Field__
#define __drishtisdk__Field__

#include "drishti/core/drishti_core.h"
#include "drishti/core/drishti_serialize.h" // needed for GENERIC_NVP()

#include <opencv2/core.hpp>

DRISHTI_CORE_NAMESPACE_BEGIN

template <typename T> struct Field
{
    Field()
    {
        has = false;
    }
    Field(const T &t) : value(t), has(true) {}
    ~Field()
    {
    }

    Field<T>& operator=(const T &src)
    {
        has = true;
        value = src;
        return *this;
    }

    void clear()
    {
        has = false;
    }

    operator T() const
    {
        CV_Assert(has);
        return value;
    }
    const T &get() const
    {
        return value;
    }
    T &get()
    {
        return value;
    }

    T& operator *()
    {
        return value;
    }
    const T& operator *() const
    {
        return value;
    }

    T* operator->()
    {
        return &value;
    }
    const T* operator->() const
    {
        return &value;
    }

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & GENERIC_NVP("has", has);
        ar & GENERIC_NVP("value", value);
    }

    T value;
    bool has = false;
};

DRISHTI_CORE_NAMESPACE_END

#endif // __drishtisdk__Field__


