#include "QtFaceDetectorFactory.h"
#include "drishti/face/Face.h"
#include "drishti/core/drishti_stdlib_string.h"

#include <cereal/archives/xml.hpp>

drishti::face::FaceModel QtFaceDetectorFactory::getMeanFace()
{
    drishti::face::FaceModel faceDetectorMean;

    LoaderFunction loader = [&](std::istream &is, const std::string &hint)
    {
        cereal::XMLInputArchive ia(is);
        ia >> faceDetectorMean;
        return true;
    };
    load(sFaceDetectorMean, loader);
    return faceDetectorMean;
}
