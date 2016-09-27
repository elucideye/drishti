#include "QtFaceDetectorFactory.h"

#include <QDirIterator>
#include <QFile>
#include <QTextStream>
#include <QDirIterator>
#include "QtStream.h"

#include "drishti/core/drishti_core.h"
#include "drishti/core/make_unique.h"
#include "drishti/acf/ACF.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/eye/EyeModelEstimator.h"

#include <iostream>

// TODO: Move these regressor names to a config file
#define DRISHTI_FACE_INNER_DETECT "drishti_face_inner_48x48.mat"
#define DRISHTI_FACE_MEAN_5_POINT "drishti_face_5_point_mean_48x48.xml"
#define DRISHTI_FACE_INNER "drishti_face_inner.pba.z"
#define DRISHTI_EYE_FULL "drishti_eye_full_npd_eix.pba.z"

static bool load(const std::string &filename, std::function<bool(std::istream &is)> &loader)
{
    std::string resource;
    
    QDirIterator it(":", QDirIterator::Subdirectories);
    while (it.hasNext())
    {
        std::string name = it.next().toStdString();
        auto pos = name.find(filename);
        if(pos != std::string::npos)
        {
            resource = name;
            break;
        }
    }
    
    if(resource.empty())
    {
        return false;
    }
    
    // Then open it:
    QFile inputFile(resource.c_str());
    if (!inputFile.open(QIODevice::ReadOnly))
    {
        std::cerr << "Can't open file" << std::endl;
        return false;
    }
    
    QByteArray raw(inputFile.readAll());  std::cout << "SIZE: " << raw.size() << std::endl;
    QtStream streambuf(raw);
    std::istream is(&streambuf);
    return loader(is);
}

QtFaceDetectorFactory::QtFaceDetectorFactory()
{
    sFaceDetector = DRISHTI_FACE_INNER_DETECT;
    sFaceRegressors = {{ DRISHTI_FACE_INNER }};
    sEyeRegressor = DRISHTI_EYE_FULL;
    sFaceDetectorMean = DRISHTI_FACE_MEAN_5_POINT;
}

std::unique_ptr<drishti::ml::ObjectDetector> QtFaceDetectorFactory::getFaceDetector()
{
    std::unique_ptr<drishti::ml::ObjectDetector> ptr;
    std::function<bool(std::istream &is)> loader = [&](std::istream &is)
    {
        ptr = drishti::core::make_unique<drishti::acf::Detector>(is);
        return true;
    };
    load(sFaceDetector, loader);
    return ptr;
}

std::unique_ptr<drishti::ml::ShapeEstimator> QtFaceDetectorFactory::getInnerFaceEstimator()
{
    std::unique_ptr<drishti::ml::ShapeEstimator> ptr;
    std::function<bool(std::istream &is)> loader = [&](std::istream &is)
    {
        ptr = drishti::core::make_unique<drishti::ml::RegressionTreeEnsembleShapeEstimator>(is);
        return true;
    };
    if(sFaceRegressors.size())
    {
        load(sFaceRegressors[0], loader);
    }
    return ptr;
}

std::unique_ptr<drishti::ml::ShapeEstimator> QtFaceDetectorFactory::getOuterFaceEstimator()
{
    std::unique_ptr<drishti::ml::ShapeEstimator> ptr;
    std::function<bool(std::istream &is)> loader = [&](std::istream &is)
    {
        ptr = drishti::core::make_unique<drishti::ml::RegressionTreeEnsembleShapeEstimator>(is);
        return true;
    };
    if(sFaceRegressors.size() > 1)
    {
        load(sFaceRegressors[1], loader);
    }
    return ptr;
}

std::unique_ptr<drishti::eye::EyeModelEstimator> QtFaceDetectorFactory::getEyeEstimator()
{
    std::unique_ptr<drishti::eye::EyeModelEstimator> ptr;
    std::function<bool(std::istream &is)> loader = [&](std::istream &is)
    {
        ptr = std::unique_ptr<DRISHTI_EYE::EyeModelEstimator>(new DRISHTI_EYE::EyeModelEstimator);
        load_pba_z(is, *ptr);
        return true;
    };
    load(sEyeRegressor, loader);
    return ptr;
}

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
