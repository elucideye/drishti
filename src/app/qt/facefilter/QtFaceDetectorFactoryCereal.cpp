#include "QtFaceDetectorFactory.h"
#include "drishti/face/Face.h"

#include <cereal/archives/xml.hpp>

drishti::face::FaceModel QtFaceDetectorFactory::getMeanFace()
{
    drishti::face::FaceModel faceDetectorMean;

    LoaderFunction loader = [&](std::istream &is, const std::string &hint)
    {
        cereal::XMLInputArchive ia(is);
        typedef decltype(ia) Archive;
        ia >> faceDetectorMean;
        return true;
    };
    load(sFaceDetectorMean, loader);
    return faceDetectorMean;
}
