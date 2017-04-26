#ifndef __train_rcpr_Recipe_h__
#define __train_rcpr_Recipe_h__

#include "drishti/core/drishti_core.h"
#include "drishti/rcpr/Recipe.h"

DRISHTI_BEGIN_NAMESPACE(cpr)
void loadJSON(const std::string &filename, std::vector<drishti::rcpr::Recipe> &recipes);
void saveJSON(const std::string &filename, const std::vector<drishti::rcpr::Recipe> &recipes);
DRISHTI_END_NAMESPACE(cpr)

#endif // __train_rcpr_Recipe_h__
