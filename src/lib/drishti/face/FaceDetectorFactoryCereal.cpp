#include "drishti/face/FaceDetectorFactory.h"
#include "drishti/core/drishti_stdlib_string.h" // FIRST

#include <cereal/archives/xml.hpp>
#include <istream>
#include <fstream>

#define DRISHTI_CEREAL_XML_JSON 1

DRISHTI_FACE_NAMESPACE_BEGIN

drishti::face::FaceModel loadFaceModel(std::istream &is)
{
    drishti::face::FaceModel faceDetectorMean;
    {
#if DRISHTI_CEREAL_XML_JSON
        cereal::XMLInputArchive ia(is);
        typedef decltype(ia) Archive;
        ia >> faceDetectorMean;
#else
        std::cerr << "Skipping JSON archive" << std::endl;
        abort();
#endif
    }
    return faceDetectorMean;
}

drishti::face::FaceModel loadFaceModel(const std::string &filename)
{
    drishti::face::FaceModel faceDetectorMean;
    std::ifstream is(filename);
    if(is)
    {
        faceDetectorMean = loadFaceModel(is);
    }
    return faceDetectorMean;
}

DRISHTI_FACE_NAMESPACE_END

