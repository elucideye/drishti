/*! -*-c++-*-
  @file  FaceTrackerFactoryJsonAndroid.h
  @brief Declaration of a FaceTrackerFactory to load models from a JSON file using the NDK AssetManager.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This file is released under the 3 Clause BSD License.}
*/

#ifndef __facefilter_renderer_android_FaceTrackerFactoryJsonAndroid_h__
#define __facefilter_renderer_android_FaceTrackerFactoryJsonAndroid_h__

#include <facefilter/facefilter.h>

#include "facefilter/renderer/FaceTrackerFactoryJson.h"

// clang-format off
namespace spdlog
{
    class logger;
};
// clang-format on

BEGIN_FACEFILTER_NAMESPACE

class FaceTrackerFactoryJsonAndroid : public FaceTrackerFactoryJson
{
public:
    FaceTrackerFactoryJsonAndroid(void* assetManager, const std::string& sModels, std::shared_ptr<spdlog::logger>& logger);
    virtual std::shared_ptr<std::istream> open_stream(const std::string& filename, std::ios_base::openmode mode = std::ios_base::in);

protected:
    void* assetManager = nullptr;
};

END_FACEFILTER_NAMESPACE

#endif // __facefilter_renderer_android_FaceTrackerFactoryJsonAndroid_h__
