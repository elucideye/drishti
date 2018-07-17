/*! -*-c++-*-
  @file  FaceTrackerFactoryJson.h
  @brief Declaration of a FaceTrackerFactory to load models from a JSON file.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This file is released under the 3 Clause BSD License.}
*/

#ifndef __facefilter_renderer_FaceTrackerFactoryJson_h__
#define __facefilter_renderer_FaceTrackerFactoryJson_h__

#include <facefilter/facefilter.h>
#include <drishti/FaceTracker.hpp>
#include <string>
#include <memory>

// clang-format off
namespace spdlog
{
    class logger;
};
// clang-format on

BEGIN_FACEFILTER_NAMESPACE

class FaceTrackerFactoryJson
{
public:
    FaceTrackerFactoryJson(const std::string& sModels, std::shared_ptr<spdlog::logger>& logger);

    operator bool() const { return good; }

    virtual std::shared_ptr<std::istream> open_stream(const std::string& filename, std::ios_base::openmode mode = std::ios_base::in);

    drishti::sdk::FaceTracker::Resources factory;

    static std::unique_ptr<FaceTrackerFactoryJson> create(void* assetManager, const std::string& sModels, std::shared_ptr<spdlog::logger>& logger);

protected:
    FaceTrackerFactoryJson();

    void init(const std::string& sModels, std::shared_ptr<spdlog::logger>& logger);

    bool good = false;

    std::vector<std::shared_ptr<std::istream>> streams;
};

END_FACEFILTER_NAMESPACE

#endif // __facefilter_renderer_FaceTrackerFactoryJson_h__
