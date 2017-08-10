#include "QtFaceDetectorFactory.h"

#include <QDirIterator>
#include <QFile>
#include <QTextStream>
#include <QDirIterator>
#include "QtStream.h"

#include "drishti/core/drishti_core.h"
#include "drishti/core/drishti_string_hash.h"
#include "drishti/core/make_unique.h"
#include "drishti/acf/ACF.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/eye/EyeModelEstimator.h"

#include "drishti/core/drishti_cereal_pba.h"

#include <iostream>

using namespace string_hash;

// TODO: Move these regressor names to a config file

#define DRISHTI_ARCHIVE "cpb"

#define DRISHTI_FACE_MEAN_5_POINT "drishti_face_tight_64x64_gray_V5_mean.json"
#define DRISHTI_FACE_INNER_DETECT "drishti_face_tight_64x64_gray_V5." DRISHTI_ARCHIVE
#define DRISHTI_FACE_FULL "drishti_full_face_model." DRISHTI_ARCHIVE
#define DRISHTI_EYE_FULL "drishti_full_eye_model." DRISHTI_ARCHIVE

bool QtFaceDetectorFactory::load(const std::string& filename, LoaderFunction& loader)
{
    std::string resource;

    QDirIterator it(":", QDirIterator::Subdirectories);
    while (it.hasNext())
    {
        std::string name = it.next().toStdString();
        auto pos = name.find(filename);
        if (pos != std::string::npos)
        {
            resource = name;
            break;
        }
    }

    if (resource.empty())
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

    QByteArray raw(inputFile.readAll());
    QtStream streambuf(raw);
    std::istream is(&streambuf);
    return loader(is, filename);
}

QtFaceDetectorFactory::QtFaceDetectorFactory()
{
    sFaceDetector = DRISHTI_FACE_INNER_DETECT;
    sFaceRegressor = DRISHTI_FACE_FULL;
    sEyeRegressor = DRISHTI_EYE_FULL;
    sFaceDetectorMean = DRISHTI_FACE_MEAN_5_POINT;
}

std::unique_ptr<drishti::ml::ObjectDetector> QtFaceDetectorFactory::getFaceDetector()
{
    std::unique_ptr<drishti::ml::ObjectDetector> ptr;

    // clang-format off
    LoaderFunction loader = [&](std::istream& is, const std::string& hint)
    {
        ptr = drishti::core::make_unique<drishti::acf::Detector>(is, hint);
        return true;
    };
    // clang-format on

    load(sFaceDetector, loader);

    return ptr;
}

std::unique_ptr<drishti::ml::ShapeEstimator> QtFaceDetectorFactory::getFaceEstimator()
{
    std::unique_ptr<drishti::ml::ShapeEstimator> ptr;

    // clang-format off
    LoaderFunction loader = [&](std::istream& is, const std::string& hint)
    {
        ptr = drishti::core::make_unique<drishti::ml::RegressionTreeEnsembleShapeEstimator>(is, hint);
        return true;
    };
    // clang-format on
    if (!sFaceRegressor.empty())
    {
        load(sFaceRegressor, loader);
    }
    return ptr;
}

std::unique_ptr<drishti::eye::EyeModelEstimator> QtFaceDetectorFactory::getEyeEstimator()
{
    std::unique_ptr<drishti::eye::EyeModelEstimator> ptr;
    LoaderFunction loader = [&](std::istream& is, const std::string& hint) {
        ptr = drishti::core::make_unique<DRISHTI_EYE::EyeModelEstimator>(is, hint);
        return true;
    };
    load(sEyeRegressor, loader);
    return ptr;
}
