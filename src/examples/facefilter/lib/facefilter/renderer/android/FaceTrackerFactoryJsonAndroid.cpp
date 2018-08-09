/*! -*-c++-*-
  @file  FaceTrackerFactoryJsonAndroid.h
  @brief Implementation of a FaceTrackerFactory to load models from a JSON file using the NDK AssetManager.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This file is released under the 3 Clause BSD License.}
*/

#include "facefilter/renderer/android/FaceTrackerFactoryJsonAndroid.h"
#include "facefilter/renderer/android/android_asset_istream.h"

BEGIN_FACEFILTER_NAMESPACE

static std::string cat(const std::string& a, const std::string& b) { return a + b; }

FaceTrackerFactoryJsonAndroid::FaceTrackerFactoryJsonAndroid
(
    void* assetManager, 
    const std::string& sModels, 
    std::shared_ptr<spdlog::logger>& logger
)
    : assetManager(assetManager)
{
    if (assetManager == nullptr)
    {
        throw std::runtime_error("FaceTrackerFactoryJsonAndroid::FaceTrackerFactoryJsonAndroid(): assetManager is null");
    }
    init(sModels, logger);
}

std::shared_ptr<std::istream> 
FaceTrackerFactoryJsonAndroid::open_stream(const std::string& filename, std::ios_base::openmode /* mode */)
{
    auto ifs = std::make_shared<android::asset_istream>(static_cast<AAssetManager*>(assetManager), filename.c_str());
    if (!ifs || !ifs->good())
    {
        throw std::runtime_error(cat("FaceTrackerFactoryJsonAndroid::open_stream() failed to open ", filename));
    }
    return ifs;
}

END_FACEFILTER_NAMESPACE
