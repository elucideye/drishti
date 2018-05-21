/*!
  @file   FaceTrackerFactoryJson.cpp
  @author David Hirvonen
  @brief  Utility class implementation for loading FaceTracker from a JSON descriptor.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FaceTrackerFactoryJson.h"

#include <drishti/drishti_cv.hpp>

// Need std:: extensions for android targets
#if defined(DRISHTI_HUNTER_TEST_ADD_TO_STRING)
#include "stdlib_string.h"
#endif

#include <nlohmann/json.hpp> // nlohman-json

#include <boost/filesystem.hpp> // for portable path (de)construction

namespace bfs = boost::filesystem;

static std::string cat(const std::string& a, const std::string& b) { return a + b; }

FaceTrackerFactoryJson::FaceTrackerFactoryJson(const std::string& sModels, const std::string& logger)
{
    std::ifstream ifs(sModels);
    if (!ifs)
    {
        throw std::runtime_error(cat("FactoryLoader::FactoryLoader() failed to open ", sModels));
    }

    nlohmann::json json;
    ifs >> json;

    factory.logger = logger; // logger name
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
        std::shared_ptr<std::istream> stream = std::make_shared<std::ifstream>(filename.string(), std::ios_base::binary | std::ios::in);
        if (!stream || !stream->good())
        {
            throw std::runtime_error(cat("FactoryLoader::FactoryLoader() failed to open ", binding.first));
        }

        (*binding.second) = stream.get();
        streams.push_back(stream);
    }

    good = true;
}
