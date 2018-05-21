/*!
  @file   FaceTrackerFactoryJson.h
  @author David Hirvonen
  @brief  Utility class to load a FaceTracker from a JSON descriptor specifying various models.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceTrackerFactory_h__
#define __drishti_face_FaceTrackerFactory_h__

#include <drishti/FaceTracker.hpp>
#include <string>
#include <memory>

class FaceTrackerFactoryJson
{
public:
    FaceTrackerFactoryJson(const std::string& sModels, const std::string& logger);
    operator bool() const { return good; }

    drishti::sdk::FaceTracker::Resources factory;

protected:
    bool good = false;
    std::vector<std::shared_ptr<std::istream>> streams;
};

#endif // __drishti_face_FaceTrackerFactory_h__
