/*! -*-c++-*-
  @file  android_assets_istream.h
  @brief Declaration of a custom Android NDK AssetManager istream abstraction.

  \copyright Copyright 2018 Elucideye, Inc. All rights reserved. [All modifications]
  \license{This file is released under the 3 Clause BSD License.}
  
  Lineage:

 * Copyright 2016 Google Inc. All Rights Reserved.

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __facefilter_renderer_android_android_asset_istream_h__
#define __facefilter_renderer_android_android_asset_istream_h__

// clang-format off
// Add namespace macros for consistent indentiation (editors, etc)
#define BEGIN_NAMESPACE_ANDROID  namespace android  {
#define END_NAMESPACE_ANDROID    }
#define BEGIN_NAMESPACE_INTERNAL namespace internal {
#define END_NAMESPACE_INTERNAL   }
// clang-format on

#if defined(__ANDROID__)

#include <android/asset_manager.h>
#include <cstring>
#include <exception>
#include <memory>
#include <sstream>
#include <vector>

BEGIN_NAMESPACE_ANDROID
BEGIN_NAMESPACE_INTERNAL

std::unique_ptr<std::streambuf> make_asset_istreambuf(
    AAssetManager* mgr, const char* filename);

struct asset_istream_base
{
    asset_istream_base(AAssetManager* mgr, const char* filename)
        : streambuf(std::unique_ptr<std::streambuf>(make_asset_istreambuf(
              mgr, filename)))
    {
    }
    std::unique_ptr<std::streambuf> streambuf;
};

END_NAMESPACE_INTERNAL

struct asset_istream
    : private internal::asset_istream_base,
      public std::istream
{

    asset_istream(AAssetManager* mgr, const char* filename)
        : internal::asset_istream_base(mgr, filename)
        , std::istream(streambuf.get())
    {
    }
};

END_NAMESPACE_ANDROID

#endif // __ANDROID__

#endif // __facefilter_renderer_android_android_asset_istream_h__
