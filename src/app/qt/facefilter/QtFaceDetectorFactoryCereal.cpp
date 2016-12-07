#include "QtFaceDetectorFactory.h"
#include "drishti/face/Face.h"

#include <cereal/archives/xml.hpp>

drishti::face::FaceModel QtFaceDetectorFactory::getMeanFace()
{
    drishti::face::FaceModel faceDetectorMean;

    std::function<bool(std::istream &is)> loader = [&](std::istream &is)
    {
        cereal::XMLInputArchive ia(is);
        typedef decltype(ia) Archive;
        ia >> faceDetectorMean;
        return true;
    };
    load(sFaceDetectorMean, loader);
    return faceDetectorMean;
}
