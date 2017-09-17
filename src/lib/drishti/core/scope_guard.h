/*! -*-c++-*-
  @file   make_unique.h
  @brief  Simple macro to add make_unique functionality currently lacking in C++11

  http://stackoverflow.com/a/28413370
  LICENSE: creative commons

*/

#ifndef __drishti_core_scope_guard_h__
#define __drishti_core_scope_guard_h__

#include <functional>

#include "drishti/core/drishti_core.h"

DRISHTI_CORE_NAMESPACE_BEGIN

class scope_guard
{
public:
    template <class Callable>
    scope_guard(Callable&& undo_func)
        : f(std::forward<Callable>(undo_func))
    {
    }

    scope_guard(scope_guard&& other)
        : f(std::move(other.f))
    {
        other.f = nullptr;
    }

    ~scope_guard()
    {
        if (f)
            f(); // must not throw
    }

    void dismiss() noexcept
    {
        f = nullptr;
    }

    scope_guard(const scope_guard&) = delete;
    void operator=(const scope_guard&) = delete;

private:
    std::function<void()> f;
};

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_scope_guard_h__
