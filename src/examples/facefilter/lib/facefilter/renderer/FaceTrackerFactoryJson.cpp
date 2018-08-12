/*! -*-c++-*-
  @file   FaceTrackerFactoryJson.cpp
  @brief  Implemenation of a FaceTrackerFactory to load models from a JSON file.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This file is released under the 3 Clause BSD License.}
*/

#include "facefilter/renderer/FaceTrackerFactoryJson.h"

#include <drishti/drishti_cv.hpp>

#include <nlohmann/json.hpp> // nlohman-json
#include <spdlog/spdlog.h>

#include <boost/filesystem.hpp> // for portable path (de)construction

#include <facefilter/make_unique.h>

// clang-format off
#if defined(__ANDROID__)
#  include "facefilter/renderer/android/FaceTrackerFactoryJsonAndroid.h"
#endif
// clang-format on

namespace bfs = boost::filesystem;

static std::string cat(const std::string& a, const std::string& b) { return a + b; }

BEGIN_FACEFILTER_NAMESPACE

std::shared_ptr<std::istream> FaceTrackerFactoryJson::open_stream(const std::string& filename, std::ios_base::openmode mode)
{
    auto ifs = std::make_shared<std::ifstream>(filename, mode);
    if (!ifs || !ifs->good())
    {
        throw std::runtime_error(cat("FaceTrackerFactoryJson::open_stream() failed to open ", filename));
    }
    return ifs;
}

FaceTrackerFactoryJson::FaceTrackerFactoryJson() = default;

FaceTrackerFactoryJson::FaceTrackerFactoryJson(const std::string& sModels, std::shared_ptr<spdlog::logger>& logger)
{
    init(sModels, logger);
}

void FaceTrackerFactoryJson::init(const std::string& sModels, std::shared_ptr<spdlog::logger>& logger)
{
    auto ifs = open_stream(sModels); // NOLINT (TODO)

    nlohmann::json json = nlohmann::json::parse(*ifs);

    std::string contents = json.dump();
    logger->info("contents: {} {}", contents.size(), contents);

    std::vector<std::pair<const char*, std::istream**>> bindings = {
        { "face_detector", &factory.sFaceDetector },
        { "eye_model_regressor", &factory.sEyeRegressor },
        { "face_landmark_regressor", &factory.sFaceRegressor },
        { "face_detector_mean", &factory.sFaceModel }
    };

    // Get the directory name:
    auto path = bfs::path(sModels);
    for (auto& binding : bindings)
    {
        auto filename = path.parent_path() / json[binding.first].get<std::string>();
        auto stream = open_stream(filename.string(), std::ios_base::binary | std::ios::in);
        (*binding.second) = stream.get();
        streams.push_back(stream);
    }

    good = true;
}

std::unique_ptr<FaceTrackerFactoryJson> FaceTrackerFactoryJson::create(void* assetManager, const std::string& sModels, std::shared_ptr<spdlog::logger>& logger)
{
#if defined(__ANDROID__)
    return facefilter::make_unique<FaceTrackerFactoryJsonAndroid>(assetManager, sModels, logger);
#else
    return facefilter::make_unique<FaceTrackerFactoryJson>(sModels, logger);
#endif
}

END_FACEFILTER_NAMESPACE

