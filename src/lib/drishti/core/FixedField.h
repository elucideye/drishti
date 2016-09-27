/*!
  @file   FixedField.h
  @author David Hirvonen
  @brief  Declaration of "syntatic sugar" ostream formatting for floating point values.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

DRISHTI_CORE_NAMESPACE_BEGIN

#ifndef DRISHTI_CORE_FIXED_FIELD_H
#define DRISHTI_CORE_FIXED_FIELD_H 1

struct FixedField
{
    FixedField(float f, int p=2, int w=4) : f(f), p(p), w(w) {}
    float f;
    int p;
    int w;
};

inline std::ostream& operator<<(std::ostream &os, const FixedField &f)
{
    os << std::fixed << std::showpoint << std::setw(f.w) << std::setprecision(f.p) << std::setfill('0') << f.f;
    return os;
}

#endif // DRISHTI_CORE_FIXED_FIELD_H 

DRISHTI_CORE_NAMESPACE_END
