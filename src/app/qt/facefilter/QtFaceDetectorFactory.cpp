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

#if DRISHTI_SERIALIZE_WITH_CEREAL
#  include "drishti/core/drishti_cereal_pba.h"
#endif

#if DRISHTI_SERIALIZE_WITH_BOOST
#  include "drishti/core/boost_serialize_common.h"
#endif

#include <iostream>

using namespace string_hash;

// TODO: Move these regressor names to a config file

// { "cpb", "pba.z", "mat" }
#define DRISHTI_ARCHIVE "cpb"

#define DRISHTI_FACE_MEAN_5_POINT "drishti_face_5_point_mean_48x48.xml"
#define DRISHTI_FACE_INNER_DETECT "drishti_face_inner_48x48." DRISHTI_ARCHIVE
#define DRISHTI_FACE_INNER "drishti_face_inner." DRISHTI_ARCHIVE
#define DRISHTI_EYE_FULL "drishti_eye_full_npd_eix." DRISHTI_ARCHIVE

bool QtFaceDetectorFactory::load(const std::string &filename, LoaderFunction &loader)
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
    
    QByteArray raw(inputFile.readAll());
    QtStream streambuf(raw);
    std::istream is(&streambuf);
    return loader(is, filename);
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
    LoaderFunction loader = [&](std::istream &is, const std::string &hint)
    {
        ptr = drishti::core::make_unique<drishti::acf::Detector>(is, hint);
        return true;
    };

    load(sFaceDetector, loader);
    
    return ptr;
}

std::unique_ptr<drishti::ml::ShapeEstimator> QtFaceDetectorFactory::getInnerFaceEstimator()
{
    std::unique_ptr<drishti::ml::ShapeEstimator> ptr;
    LoaderFunction loader = [&](std::istream &is, const std::string &hint)
    {
        ptr = drishti::core::make_unique<drishti::ml::RegressionTreeEnsembleShapeEstimator>(is, hint);
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
    LoaderFunction loader = [&](std::istream &is, const std::string &hint)
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
    LoaderFunction loader = [&](std::istream &is, const std::string &hint)
    {
        ptr = drishti::core::make_unique<DRISHTI_EYE::EyeModelEstimator>(is, hint);
        return true;
    };
    load(sEyeRegressor, loader);
    return ptr;
}
