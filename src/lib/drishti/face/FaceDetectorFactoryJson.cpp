/*!
  @file   face/FaceDetectorFactoryJson.cpp
  @author David Hirvonen
  @brief  Utility class implementation for loading FaceDetectorFactory from a JSON descriptor.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceDetectorFactoryJson.h"

// Need std:: extensions for android targets
#include "drishti/core/drishti_stdlib_string.h"

#include <nlohmann/json.hpp> // nlohman-json

#include <boost/filesystem.hpp> // for portable path (de)construction

namespace bfs = boost::filesystem;

DRISHTI_FACE_NAMESPACE_BEGIN

static std::string cat(const std::string &a, const std::string &b) { return a + b; }

FaceDetectorFactoryJson::FaceDetectorFactoryJson(const std::string &sModels)
{
    std::ifstream ifs(sModels);
    if (!ifs)
    {
        throw std::runtime_error(cat("FaceDetectorFactoryJson::FaceDetectorFactoryJson() failed to open ", sModels));
    }

    nlohmann::json json;
    ifs >> json;

    std::vector< std::pair<const char *, std::string *> > bindings =
    {
        { "face_detector", &sFaceDetector },
        { "eye_model_regressor", &sEyeRegressor },
        { "face_landmark_regressor", &sFaceRegressor },
        { "face_detector_mean", &sFaceDetectorMean }
    };

    // Get the directory name:
    auto path = bfs::path(sModels);
    for(auto &binding : bindings)
    {
        auto filename = path.parent_path() / json[binding.first].get<std::string>();
        (*binding.second) = filename.string();
        if(binding.second->empty())
        {
            throw std::runtime_error(cat("FaceDetectorFactoryJson::FaceDetectorFactoryJson()", binding.first));
        }
    }
}

DRISHTI_FACE_NAMESPACE_END
