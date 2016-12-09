#ifndef ASSET_MANAGER_H
#define ASSET_MANAGER_H

#include "drishti/face/FaceDetectorFactory.h"

namespace drishti
{
    namespace ml
    {
        class ObjectDetector;
        class ShapeEstimator;
    }
}

class QtFaceDetectorFactory : public drishti::face::FaceDetectorFactory
{
public:
    
    using LoaderFunction = std::function<bool(std::istream &is, const std::string &hint)>;
    
    QtFaceDetectorFactory();
    
    virtual std::unique_ptr<drishti::ml::ObjectDetector> getFaceDetector();
    virtual std::unique_ptr<drishti::ml::ShapeEstimator> getInnerFaceEstimator();
    virtual std::unique_ptr<drishti::ml::ShapeEstimator> getOuterFaceEstimator();
    virtual std::unique_ptr<drishti::eye::EyeModelEstimator> getEyeEstimator();
    
    virtual  drishti::face::FaceModel getMeanFace();

    static bool load(const std::string &filename, LoaderFunction &loader);
};

#endif // ASSET_MANAGER_H
