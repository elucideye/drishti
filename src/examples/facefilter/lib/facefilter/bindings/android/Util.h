/*! -*-c++-*-
  @file  Util.h
  @brief Android utility logging macros.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved. [All modifications]
  \license{This project is released under the 3 Clause BSD License.}

  ---

  Lineage: Created by fricke on 8/20/17. 

  TODO: Full attributes? Remove in favor of spdlog, etc?

*/

#ifndef __facefilter_bindings_android_Util_h__
#define __facefilter_bindings_android_Util_h__

#include <unistd.h>
#include <android/log.h>

// used to get logcat outputs which can be regex filtered by the LOG_TAG we give
// So in Logcat you can filter this example by putting OpenCV-NDK
#define LOG_TAG "OpenCV-NDK-Native"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define ASSERT(cond, fmt, ...)                                    \
    if (!(cond))                                                  \
    {                                                             \
        __android_log_assert(#cond, LOG_TAG, fmt, ##__VA_ARGS__); \
    }

// A Data Structure to communicate resolution between camera and ImageReader
struct ImageFormat
{
    int32_t width;
    int32_t height;
    int32_t format; // ex) YUV_420
};

/**
 * A helper class to assist image size comparison, by comparing the absolute
 * size
 * regardless of the portrait or landscape mode.
 */
class Display_Dimension
{
public:
    Display_Dimension(int32_t w, int32_t h)
        : w_(w)
        , h_(h)
        , portrait_(false)
    {
        if (h > w)
        {
            // make it landscape
            w_ = h;
            h_ = w;
            portrait_ = true;
        }
    }
    Display_Dimension(const Display_Dimension& other)
    {
        w_ = other.w_;
        h_ = other.h_;
        portrait_ = other.portrait_;
    }

    Display_Dimension(void)
    {
        w_ = 0;
        h_ = 0;
        portrait_ = false;
    }
    Display_Dimension& operator=(const Display_Dimension& other)
    {
        w_ = other.w_;
        h_ = other.h_;
        portrait_ = other.portrait_;

        return (*this);
    }

    bool IsSameRatio(Display_Dimension& other)
    {
        return (w_ * other.h_ == h_ * other.w_);
    }
    bool operator>(Display_Dimension& other)
    {
        return (w_ >= other.w_ & h_ >= other.h_);
    }
    bool operator==(Display_Dimension& other)
    {
        return (w_ == other.w_ && h_ == other.h_ && portrait_ == other.portrait_);
    }
    Display_Dimension operator-(Display_Dimension& other)
    {
        Display_Dimension delta(w_ - other.w_, h_ - other.h_);
        return delta;
    }
    void Flip(void) { portrait_ = !portrait_; }
    bool IsPortrait(void) { return portrait_; }
    int32_t width(void) { return w_; }
    int32_t height(void) { return h_; }
    int32_t org_width(void) { return (portrait_ ? h_ : w_); }
    int32_t org_height(void) { return (portrait_ ? w_ : h_); }

private:
    int32_t w_, h_;
    bool portrait_;
};
#endif // __facefilter_bindings_android_Util_h__
