/**
  @file   EyeDetector.cpp
  @author David Hirvonen
  @brief  Top level eye detection API class implementation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of the eye detection class used
  for the top level SDK.
*/

#include "drishti/EyeDetector.hpp"
#include "drishti/drishti_cv.hpp"

#include <acf/ACF.h>

#include <opencv2/core/core.hpp>

_DRISHTI_SDK_BEGIN

class EyeDetector::Impl
{
public:
    Impl() {}
    Impl(const std::string& filename)
    {
        init(filename);
    }
    ~Impl() {}

    void init(const std::string& filename)
    {
        m_detector = std::make_shared<acf::Detector>(filename);

#if 1
        // Perform modification
        acf::Detector::Modify dflt;
        dflt.cascThr = { "cascThr", -1.0 };
        dflt.cascCal = { "cascCal", -0.005 };
        m_detector->acfModify(dflt);
#endif
    }

    int operator()(const Image3b& image, std::vector<Rect>& objects)
    {
        cv::Mat3b input = drishtiToCv<Vec3b, cv::Vec3b>(image);
        std::vector<cv::Rect> hits;
        (*m_detector)(input, hits);

        //std::cout << hits.size() << std::endl;

        auto cvToDrishti = [](const cv::Rect& roi) {
            return Rect(roi.x, roi.y, roi.width, roi.height);
        };
        std::transform(hits.begin(), hits.end(), std::back_inserter(objects), cvToDrishti);

        return objects.size();
    }

    std::shared_ptr<acf::Detector> m_detector;
};

// ######### EyeDetector ############

EyeDetector::EyeDetector() {}
EyeDetector::EyeDetector(const std::string& filename)
{
    m_impl = std::unique_ptr<Impl>(new Impl(filename));
}
EyeDetector::~EyeDetector() {}
int EyeDetector::operator()(const Image3b& image, std::vector<Rect>& objects)
{
    return (*m_impl)(image, objects);
}

_DRISHTI_SDK_END
