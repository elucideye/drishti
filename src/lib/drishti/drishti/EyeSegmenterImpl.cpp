/**
  @file   EyeSegmenterImpl.cpp
  @author David Hirvonen
  @brief  Top level API eye model declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of the private implementation structure
  used for the top level SDK -- this provides a cleaner API, minimizes dependencies
  and improve build times.
*/

#include "drishti/EyeSegmenterImpl.hpp"

#include "drishti/eye/Eye.h" // internal sdk eye model
#include "drishti/eye/EyeModelEstimator.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "drishti/drishti_cv.hpp" // Must come after opencvx

// auto-generated from ${DRISHTI}/assets/model.pba.z
//#include "drishti/eye_model_rsc.h"

#include "drishti/core/Logger.h"

#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

// TODO: Store this somewhere in the file
#define EYE_ASPECT_RATIO (4.0/3.0)

#define MINIMUM_EYE_WIDTH 32

_DRISHTI_SDK_BEGIN

static Eye convert(DRISHTI_EYE::EyeModel &model);

EyeSegmenter::Impl::Impl(bool doLoad)
{

}

EyeSegmenter::Impl::Impl(const std::string &filename)
{
    std::ifstream is(filename);
    init(is);
}

EyeSegmenter::Impl::Impl(std::istream &is)
{
    init(is);
}

void EyeSegmenter::Impl::init(std::istream &is)
{
    // First try loading from resource
    m_eme = std::unique_ptr<DRISHTI_EYE::EyeModelEstimator>(new DRISHTI_EYE::EyeModelEstimator());
    DRISHTI_EYE::EyeModelEstimator::load(is, (*m_eme));
    m_eme->setDoPupil(false);
    m_eme->setDoVerbose(false);
    m_eme->setTargetWidth(128);
    m_eme->setEyelidInits(1); // fast configuration:
    m_eme->setIrisInits(1);

    m_streamLogger = core::Logger::create("SEGMENT");
    m_eme->setStreamLogger(m_streamLogger);
}

EyeSegmenter::Impl::~Impl() {}

int EyeSegmenter::Impl::operator()(const Image3b &image, Eye &eye, bool isRight)
{
    DRISHTI_STREAM_LOG_FUNC(1,1,m_streamLogger);

    int status = 0;

    const int minWidth = getMinWidth();
    const int minHeight = int(float(getMinWidth()) / getRequiredAspectRatio() + 0.5f);
    if(image.getCols() < minWidth || image.getRows() < minHeight)
    {
        return 1;
    }

    //const float aspectRatio  = float(image.cols) / image.rows;

    try
    {
        // Create shallow copy of input image
        cv::Mat3b I = drishtiToCv<Vec3b, cv::Vec3b>(image), If;

        // If input is left eye, we flop and must allocate a new image:
        if(!isRight)
        {
            cv::flip(I, If, 1);
        }
        else
        {
            If = I;
        }

        DRISHTI_EYE::EyeModel model;
        status = (*m_eme)(If, model);

        if(!isRight)
        {
            model.flop(If.cols);
        }
        model.refine();
        model.roi = cv::Rect({0,0}, If.size()); // default roi
        eye = convert(model);
    }
    catch(...)
    {
        std::cerr << "exception: EyeSegmenter::Impl::operator()" << std::endl;
        status = 1;
    }

    core::Logger::increment();

    return status;
}

Eye EyeSegmenter::Impl::getMeanEye(int width) const
{
    int height = int(float(width) / EYE_ASPECT_RATIO + 0.5f);
    DRISHTI_EYE::EyeModel model = m_eme->getMeanShape( cv::Size(width, height));
    drishti::sdk::Eye eye = convert(model);
    eye.setRoi( {0, 0, width, height} );

    return eye;
}

int EyeSegmenter::Impl::getIrisInits() const
{
    return m_eme->getIrisInits();
}

void EyeSegmenter::Impl::setIrisInits(int count)
{
    m_eme->setIrisInits(count);  // not reentrant when > 1
}

int EyeSegmenter::Impl::getEyelidInits() const
{
    return m_eme->getEyelidInits();
}

void EyeSegmenter::Impl::setEyelidInits(int count)
{
    m_eme->setEyelidInits(count);  // not reentrant when > 1
}

void EyeSegmenter::Impl::setOptimizationLevel(int level)
{
    m_eme->setOptimizationLevel(level);
}

// Static utility:

int EyeSegmenter::Impl::getMinWidth()
{
    return MINIMUM_EYE_WIDTH;
}

float EyeSegmenter::Impl::getRequiredAspectRatio()
{
    return EYE_ASPECT_RATIO;
}

static Eye convert(DRISHTI_EYE::EyeModel &model)
{
    Eye e;

    const auto & inner = model.getInnerCorner();
    const auto & outer = model.getOuterCorner();

    e.setIris(cvToDrishti(model.irisEllipse));
    e.setPupil(cvToDrishti(model.pupilEllipse));
    e.setEyelids(drishti::sdk::cvToDrishti(model.eyelidsSpline));
    e.setCrease(drishti::sdk::cvToDrishti(model.creaseSpline));
    e.setCorners(cvToDrishti(inner), cvToDrishti(outer));
    e.setRoi(cvToDrishti(model.roi.has ? *model.roi : cv::Rect()));

    return e;
}

static DRISHTI_EYE::EyeModel convert(drishti::sdk::Eye &eye)
{
    DRISHTI_EYE::EyeModel e;

    const auto & inner = eye.getInnerCorner();
    const auto & outer = eye.getOuterCorner();

    e.irisEllipse = drishtiToCv(eye.getIris());
    e.pupilEllipse = drishtiToCv(eye.getPupil());
    e.eyelidsSpline = drishtiToCv(eye.getEyelids());
    e.innerCorner = drishtiToCv(inner);
    e.outerCorner = drishtiToCv(outer);

    return e;
}

_DRISHTI_SDK_END

