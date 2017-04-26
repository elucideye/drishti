#ifndef __train_rcpr_Eye_h__
#define __train_rcpr_Eye_h__

#include "drishti/core/drishti_core.h"
#include "drishti/eye/Eye.h"

DRISHTI_BEGIN_NAMESPACE(cpr)
void loadJSON(const std::string &filename, drishti::eye::EyeModel &eye);
void saveJSON(const std::string &filename, const drishti::eye::EyeModel &eye);
DRISHTI_END_NAMESPACE(cpr)

#endif // __train_rcpr_Eye_h__
