#ifndef __drishti_cpr_EyeIO_h__
#define __drishti_cpr_EyeIO_h__

#include "drishti/core/drishti_core.h"
#include "drishti/eye/Eye.h"

DRISHTI_BEGIN_NAMESPACE(cpr)
void loadJSON(const std::string& filename, drishti::eye::EyeModel& eye);
void saveJSON(const std::string& filename, const drishti::eye::EyeModel& eye);
DRISHTI_END_NAMESPACE(cpr)

#endif // __drishti_cpr_EyeIO_h__
