/*
 * Copyright (c) 2012 Nathan L. Binkert <nate@binkert.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __MAKE_UNIQUE_HH__
#define __MAKE_UNIQUE_HH__

#include <memory>

namespace std
{
namespace detail
{

    // helper to construct a non-array unique_ptr
    template <typename T>
    struct make_unique_helper
    {
        typedef std::unique_ptr<T> unique_ptr;

        template <typename... Args>
        static inline unique_ptr make(Args&&... args)
        {
            return unique_ptr(new T(std::forward<Args>(args)...));
        }
    };

    // helper to construct an array unique_ptr
    template <typename T>
    struct make_unique_helper<T[]>
    {
        typedef std::unique_ptr<T[]> unique_ptr;

        template <typename... Args>
        static inline unique_ptr make(Args&&... args)
        {
            return unique_ptr(new T[sizeof...(Args)]{ std::forward<Args>(args)... });
        }
    };

    // helper to construct an array unique_ptr with specified extent
    template <typename T, std::size_t N>
    struct make_unique_helper<T[N]>
    {
        typedef std::unique_ptr<T[]> unique_ptr;

        template <typename... Args>
        static inline unique_ptr make(Args&&... args)
        {
            static_assert(N >= sizeof...(Args),
                "For make_unique<T[N]> N must be as largs as the number of arguments");
            return unique_ptr(new T[N]{ std::forward<Args>(args)... });
        }

#if __GNUC__ == 4 && __GNUC_MINOR__ <= 6
        // G++ 4.6 has an ICE when you have no arguments
        static inline unique_ptr make()
        {
            return unique_ptr(new T[N]);
        }
#endif
    };

} // namespace detail

template <typename T, typename... Args>
inline typename detail::make_unique_helper<T>::unique_ptr
make_unique(Args&&... args)
{
    return detail::make_unique_helper<T>::make(std::forward<Args>(args)...);
}

} // namespace std

#endif // __MAKE_UNIQUE_HH__
