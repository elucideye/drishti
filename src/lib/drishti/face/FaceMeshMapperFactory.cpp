/*! -*-c++-*-
 @file   FaceMeshMapperFactory.cpp
 @author David Hirvonen
 @brief  Internal implementation of a factory for FaceMeshMapper modules
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "drishti/face/FaceMeshMapperFactory.h"
#include "drishti/face/FaceMeshMapperEOSLandmark.h"
#include "drishti/face/FaceMeshMapperEOSLandmarkContour.h"
#include "drishti/core/make_unique.h"

// Need std:: extensions for android targets
#include "drishti/core/drishti_stdlib_string.h"

#include <nlohmann/json.hpp> // nlohman-json

#include <boost/filesystem.hpp> // for portable path (de)construction

namespace bfs = boost::filesystem;

DRISHTI_FACE_NAMESPACE_BEGIN

/*
 * FaceMeshMapperFactor (string)
 */

struct FaceMeshMapperFactory::Impl
{
    static std::string cat(const std::string& a, const std::string& b) { return a + b; }
    
    Impl() = default;
    
    Impl(const std::string &filename)
    {
        // binding...
        std::ifstream ifs(filename);
        if (!ifs)
        {
            throw std::runtime_error(cat("FaceMeshMapperFactory::Impl::Impl(): failed to open ", filename));
        }

        nlohmann::json json;
        ifs >> json;        

        // Get the directory name:
        auto path = bfs::path(filename);
        for (auto& binding : bindings)
        {
            auto filename = path.parent_path() / json[binding.first].get<std::string>();
            (*binding.second) = filename.string();
            if (binding.second->empty())
            {
                throw std::runtime_error(cat("FaceMeshMapperFactory::Impl::Impl(): failed to open ", binding.first));
            }
        }
    }
    
    ~Impl() = default;
    
    const std::string flatten()
    {
        nlohmann::json json;
        for (auto& binding : bindings)
        {
            json[binding.first] = "<filename>";
        }
        
        return json.dump(0);
    }
    
    drishti::face::FaceMeshMapperEOSLandmarkContour::Assets assets;

    std::vector<std::pair<const char*, std::string*>> bindings = {
        { "model", &assets.model },
        { "mapping", &assets.mappings },
        { "model-contour", &assets.contour },
        { "edge-topology", &assets.edgetopology },
        { "blendshapes", &assets.blendshapes }
    };
};

FaceMeshMapperFactory::FaceMeshMapperFactory(const std::string &filename)
{
    impl = drishti::core::make_unique<Impl>(filename);
}

FaceMeshMapperFactory::~FaceMeshMapperFactory() = default;

std::shared_ptr<drishti::face::FaceMeshMapper> FaceMeshMapperFactory::create(AlignmentStrategy kind)
{
    switch(kind)
    {
    case kLandmarks:
        return std::make_shared<FaceMeshMapperEOSLandmark>(impl->assets.model, impl->assets.mappings);
    case kLandmarksContours:
        return std::make_shared<FaceMeshMapperEOSLandmarkContour>(impl->assets);
    }
}

void FaceMeshMapperFactory::serialize(const std::string &filename)
{
    std::string json = Impl().flatten();
    std::ofstream ofs(filename);
    if (ofs)
    {
        ofs << json;
    }
}

DRISHTI_FACE_NAMESPACE_END
