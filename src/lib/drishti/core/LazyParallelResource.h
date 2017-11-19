/*! -*-c++-*-
  @file   LazyParallelResource.h
  @author David Hirvonen
  @brief  Declaration of parallel_for abstraction classes

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_LazyParallelResource_h__
#define __drishti_core_LazyParallelResource_h__

#include "drishti/core/drishti_core.h"

#include <functional>
#include <map>
#include <mutex>

DRISHTI_CORE_NAMESPACE_BEGIN

template <typename Key, typename Value>
struct LazyParallelResource
{
    template <class Callable>
    LazyParallelResource(Callable&& func)
        : m_alloc(std::forward<Callable>(func))
    {
    }
    LazyParallelResource(LazyParallelResource&& other)
        : m_alloc(std::move(other.m_alloc))
    {
        other.m_alloc = nullptr;
    }

    virtual Value& operator[](const Key& key)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        auto iter = m_map.find(key);
        if (iter == m_map.end())
        {
            return m_map[key] = m_alloc();
        }
        return iter->second;
    }

    std::map<Key, Value>& getMap() { return m_map; }
    const std::map<Key, Value>& getMap() const { return m_map; }

    std::map<Key, Value> m_map;
    std::mutex m_mutex;
    std::function<Value()> m_alloc; // default allocator
};

DRISHTI_CORE_NAMESPACE_END

#endif
