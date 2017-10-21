/*! -*-c++-*-
  @file   Field.h
  @author David Hirvonen
  @brief  Declaration of optional (de)serializable value

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
 
  NOTE: GENERIC_NVP should be defined prior to including this class for either
  boost or cereal archives.

*/

#ifndef __drishti_core_Field_h__
#define __drishti_core_Field_h__

#include "drishti/core/drishti_core.h"

#include <string>
#include <assert.h>

DRISHTI_CORE_NAMESPACE_BEGIN

template <typename T>
struct Field
{
    Field()
        : has(false)
    {
    }
    Field(const T& t)
        : has(true)
        , value(t)
    {
    }
    ~Field() = default;

    Field<T>& operator=(const T& src)
    {
        has = true;
        value = src;
        return *this;
    }

    void merge(const Field<T>& df, int checkExtra)
    {
        if (!has && df.has)
        {
            set(df.has, df.value);
        }
    }

    void set(bool has_, const T& value_)
    {
        has = has_;
        value = value_;
    }

    void clear()
    {
        has = false;
    }

    operator T() const
    {
        assert(has);
        return value;
    }
    const T& get() const
    {
        return value;
    }
    T& get()
    {
        return value;
    }

    T& operator*()
    {
        return value;
    }
    const T& operator*() const
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

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& has;
        ar& value;
    }

    bool has = false;
    T value;
};

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_Field_h__
