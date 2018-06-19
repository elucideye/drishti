/**
  @file   Array.hpp
  @brief  Lightweight portable container.

  \copyright Copyright 2014-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the image structure used
  to store images that are processed by the top level SDK.
*/

#ifndef __drishti_drishti_Array_hpp__
#define __drishti_drishti_Array_hpp__

#include <drishti/drishti_sdk.hpp>
#include <cstdint>
#include <iterator>
#include <algorithm>

_DRISHTI_SDK_BEGIN

template <typename T, std::size_t N = 128>
class Array
{
public:
    // http://stackoverflow.com/a/7759622
    class iterator
    {
    public:
        using difference_type = typename std::ptrdiff_t;
        using value_type = T;
        using reference = T&;
        using pointer = T*;
        using iterator_category = std::forward_iterator_tag;
        using size_type = std::size_t;
        
        iterator() = default;
        iterator(const iterator& other)
            : ptr_(other.ptr_)
        {
        }
        iterator(pointer ptr_)
            : ptr_(ptr_)
        {
        }
        ~iterator() = default;

        iterator& operator=(const iterator& other)
        {
            ptr_ = other.ptr_;
            return *this;
        }
        bool operator==(const iterator& other) const { return ptr_ == other.ptr_; }
        bool operator!=(const iterator& other) const { return ptr_ != other.ptr_; }
        bool operator<(const iterator& other) const { return ptr_ < other.ptr; }    //optional
        bool operator>(const iterator& other) const { return ptr_ > other.ptr; };   //optional
        bool operator<=(const iterator& other) const { return ptr_ <= other.ptr; }; //optional
        bool operator>=(const iterator& other) const { return ptr_ >= other.ptr; }; //optional

        iterator& operator++()
        {
            ptr_++;
            return *this;
        }
        const iterator operator++(int)
        {
            iterator i = *this;
            ptr_++;
            return i;
        } // optional
        iterator& operator+=(size_type n)
        {
            ptr_ += n;
            return *this;
        } //optional
        iterator operator+(size_type n) const
        {
            iterator i = *this;
            i += n;
            return i;
        } //optional
        //friend iterator operator+(size_type, const iterator&); //optional

        reference operator*() const { return *ptr_; }
        pointer operator->() const { return ptr_; }
        reference operator[](size_type i) const { return ptr_[i]; } //optional

    protected:
        pointer ptr_;
    };

    class const_iterator
    {
    public:
        using difference_type = typename std::ptrdiff_t;
        using value_type = T;
        using const_reference = const T &;
        using const_pointer = const T *;
        using iterator_category = std::forward_iterator_tag;
        using size_type = std::size_t;

        const_iterator() = default;
        const_iterator(const const_iterator& other)
            : ptr_(other.ptr_)
        {
        }
        const_iterator(const iterator& other)
            : ptr_(other.ptr_)
        {
        }
        const_iterator(const_pointer ptr_)
            : ptr_(ptr_)
        {
        }
        ~const_iterator() = default;

        const_iterator& operator=(const const_iterator& other)
        {
            ptr_ = other.ptr_;
            return *this;
        }
        bool operator==(const const_iterator& other) const { return ptr_ == other.ptr_; }
        bool operator!=(const const_iterator& other) const { return ptr_ != other.ptr_; }
        bool operator<(const const_iterator& other) const { return ptr_ < other.ptr; }    //optional
        bool operator>(const const_iterator& other) const { return ptr_ > other.ptr; };   //optional
        bool operator<=(const const_iterator& other) const { return ptr_ <= other.ptr; }; //optional
        bool operator>=(const const_iterator& other) const { return ptr_ >= other.ptr; }; //optional

        const_iterator& operator++()
        {
            ptr_++;
            return *this;
        }
        const const_iterator operator++(int)
        {
            const_iterator i = *this;
            ptr_++;
            return i;
        } // optional

        const_iterator& operator+=(size_type n)
        {
            ptr_ += n;
            return *this;
        } //optional
        const_iterator operator+(size_type n) const
        {
            const_iterator i = *this;
            i += n;
            return i;
        } //optional
        //friend const_iterator operator+(size_type, const const_iterator&); //optional

        const_reference operator*() const { return *ptr_; }
        const_pointer operator->() const { return ptr_; }
        const_reference operator[](size_type i) const { return ptr_[i]; } //optional

        const_pointer ptr_;
    };

    Array()
         
    = default;
    Array(std::size_t size)
        : size_(std::min(size, N))
    {
    } // clippinpg semantics
    Array(const Array& src)
        : size_(src.size_)
    {
        for (std::size_t i = 0; i < size_; i++)
        {
            data_[i] = src.data_[i];
        }
    }

    void clear() { size_ = 0; }
    void resize(std::size_t size) { size_ = std::min(N, size); }

    std::size_t size() const { return size_; }

    constexpr std::size_t limit() const { return N; };

    T& operator[](std::size_t index) { return data_[index]; }
    const T& operator[](std::size_t index) const { return data_[index]; }

    iterator begin() { return data_; }
    iterator end() { return (data_ + size_); }

    const_iterator begin() const { return const_iterator(data_); }
    const_iterator end() const { return (data_ + size_); }

private:
    T data_[N]{};
    std::size_t size_{0};
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_Array_hpp__
