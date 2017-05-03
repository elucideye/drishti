#include "RecipeIO.h"
#include "drishti/core/drishti_stdlib_string.h" // must be first!!!
#include "drishti/core/drishti_cv_cereal.h"

#include <cereal/archives/json.hpp>

#include <fstream>

CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(drishti::rcpr::Recipe, cereal::specialization::non_member_serialize);

typedef cereal::JSONInputArchive IArchive;
typedef cereal::JSONOutputArchive OArchive;

DRISHTI_RCPR_NAMESPACE_BEGIN

template <class Archive>
void serialize(Archive& ar, Recipe& recipe, const unsigned int version)
{
    ar& GENERIC_NVP("maxLeafNodes", recipe.maxLeafNodes);
    ar& GENERIC_NVP("maxDepth", recipe.maxDepth);
    ar& GENERIC_NVP("treesPerLevel", recipe.treesPerLevel);
    ar& GENERIC_NVP("featurePoolSize", recipe.featurePoolSize);
    ar& GENERIC_NVP("featureSampleSize", recipe.featureSampleSize);
    ar& GENERIC_NVP("featureRadius", recipe.featureRadius);
    ar& GENERIC_NVP("learningRate", recipe.learningRate);
    ar& GENERIC_NVP("dataSampleRatio", recipe.dataSampleRatio);
    ar& GENERIC_NVP("useNPD", recipe.useNPD);
    ar& GENERIC_NVP("doMask", recipe.doMask);
    ar& GENERIC_NVP("paramIndex", recipe.paramIndex);
    ar& GENERIC_NVP("lambda", recipe.lambda);
}

template void serialize<IArchive>(IArchive&, Recipe&, const unsigned int version);
template void serialize<OArchive>(OArchive&, Recipe&, const unsigned int version);

DRISHTI_RCPR_NAMESPACE_END

DRISHTI_BEGIN_NAMESPACE(cpr)
void loadJSON(const std::string& filename, std::vector<drishti::rcpr::Recipe>& recipes)
{
    std::ifstream is(filename);
    if (is)
    {
        cereal::JSONInputArchive ia(is);
        typedef decltype(ia) Archive;
        ia(GENERIC_NVP("recipe", recipes));
    }
}

void saveJSON(const std::string& filename, const std::vector<drishti::rcpr::Recipe>& recipes)
{
    std::ofstream os(filename);
    if (os)
    {
        cereal::JSONOutputArchive oa(os);
        typedef decltype(oa) Archive;
        oa(GENERIC_NVP("recipe", recipes));
    }
}
DRISHTI_END_NAMESPACE(cpr)
