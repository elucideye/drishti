#include "drishti/face/FaceDetectorFactory.h"
#include "drishti/core/drishti_stdlib_string.h" // FIRST
#include "drishti/core/drishti_cv_cereal.h"

#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>

#include <istream>
#include <fstream>

DRISHTI_FACE_NAMESPACE_BEGIN

struct Landmarks
{
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(GENERIC_NVP("landmarks", landmarks));
    }
    std::vector<cv::Point2f> landmarks;
};

drishti::face::FaceModel loadFaceModel(std::istream& is)
{
    drishti::face::FaceModel face;

    if (is)
    {
        cereal::JSONInputArchive ia(is);
        typedef decltype(ia) Archive;

        std::vector<cv::Point2f> landmarks;
        ia(GENERIC_NVP("landmarks", landmarks));

        CV_Assert(landmarks.size() == 5);

        face.eyeRightCenter = landmarks[0];
        face.eyeLeftCenter = landmarks[1];
        face.noseTip = landmarks[2];
        face.mouthCornerRight = landmarks[3];
        face.mouthCornerLeft = landmarks[4];
    }

    return face;
}

drishti::face::FaceModel loadFaceModel(const std::string& filename)
{
    drishti::face::FaceModel faceDetectorMean;
    std::ifstream is(filename);
    if (is)
    {
        faceDetectorMean = loadFaceModel(is);
    }
    return faceDetectorMean;
}

DRISHTI_FACE_NAMESPACE_END
