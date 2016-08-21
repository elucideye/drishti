#ifndef ASSET_MANAGER_H
#define ASSET_MANAGER_H

#include "face/FaceDetectorFactory.h"

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
    QtFaceDetectorFactory();
    
    virtual std::unique_ptr<drishti::ml::ObjectDetector> getFaceDetector();
    virtual std::unique_ptr<drishti::ml::ShapeEstimator> getInnerFaceEstimator();
    virtual std::unique_ptr<drishti::ml::ShapeEstimator> getOuterFaceEstimator();
    virtual std::unique_ptr<drishti::eye::EyeModelEstimator> getEyeEstimator();
    
    virtual  drishti::face::FaceModel getMeanFace();
};

#endif // ASSET_MANAGER_H
