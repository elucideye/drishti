#include "EyeIO.h"
#include "drishti/core/drishti_stdlib_string.h" // must be first!!!
#include "drishti/core/drishti_cv_cereal.h"

// JSON archive must be included in "private" scope
#include <cereal/archives/json.hpp>

#include <fstream>

DRISHTI_BEGIN_NAMESPACE(cpr)
void loadJSON(const std::string& filename, drishti::eye::EyeModel& eye)
{
    std::ifstream is(filename);
    if (is)
    {
        cereal::JSONInputArchive ia(is);
        typedef decltype(ia) Archive;
        ia(GENERIC_NVP("eye", eye));
    }
}

void saveJSON(const std::string& filename, const drishti::eye::EyeModel& eye)
{
    std::ofstream os(filename);
    if (os)
    {
        cereal::JSONOutputArchive oa(os);
        typedef decltype(oa) Archive;
        oa(GENERIC_NVP("eye", eye));
    }
}
DRISHTI_END_NAMESPACE(cpr)
