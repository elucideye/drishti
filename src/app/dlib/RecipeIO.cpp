#include "RecipeIO.h"

#include "drishti/core/drishti_stdlib_string.h" // must be first!!!
#include "drishti/core/drishti_cv_cereal.h"

#include "drishti/core/drishti_stdlib_string.h"
#include "nlohmann/json.hpp" // nlohman-json

#include <fstream>

void loadJSON(const std::string& filename, drishti::dlib::Recipe& recipe)
{
    std::ifstream is(filename);
    if (is)
    {
        nlohmann::json json;
        is >> json;

        recipe.do_pca = json["do_pca"].get<bool>();
        recipe.do_affine = json["do_affine"].get<bool>();
        recipe.do_interpolate = json["do_interpolate"].get<bool>();
        recipe.npd = json["npd"].get<bool>();
        recipe.lambda = json["lambda"].get<float>();
        recipe.nu = json["nu"].get<float>();
        recipe.padding = json["padding"].get<float>();
        recipe.cascades = json["cascades"].get<int>();
        recipe.depth = json["depth"].get<int>();
        recipe.ellipse_count = json["ellipse_count"].get<int>();
        recipe.features = json["features"].get<int>();
        recipe.oversampling = json["oversampling"].get<int>();
        recipe.splits = json["splits"].get<int>();
        recipe.trees_per_level = json["trees_per_level"].get<int>();
        recipe.width = json["width"].get<int>();
        recipe.dimensions = json["dimensions"].get<std::vector<int>>();
        
        // Per parameter weighting
        recipe.weights = json["weights"].get<std::map<std::string, float>>();
    }
    else
    {
        throw std::runtime_error("failed to upon JSON file");
    }
}

void saveJSON(const std::string& filename, const drishti::dlib::Recipe& recipe)
{
    std::ofstream os(filename);
    if (os)
    {
        nlohmann::json json;

        json["do_pca"] = recipe.do_pca;
        json["do_affine"] = recipe.do_affine;
        json["do_interpolate"] = recipe.do_interpolate;
        json["npd"] = recipe.npd;
        json["lambda"] = recipe.lambda;
        json["nu"] = recipe.nu;
        json["padding"] = recipe.padding;
        json["cascades"] = recipe.cascades;
        json["depth"] = recipe.depth;
        json["ellipse_count"] = recipe.ellipse_count;
        json["features"] = recipe.features;
        json["oversampling"] = recipe.oversampling;
        json["splits"] = recipe.splits;
        json["trees_per_level"] = recipe.trees_per_level;
        json["width"] = recipe.width;
        json["dimensions"] = recipe.dimensions;
        
        // Per parameter weighting
        json["weights"] = recipe.weights;

        os << json.dump(4);
    }
    else
    {
        throw std::runtime_error("failed to save JSON file");
    }
}
