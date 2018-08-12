/**
  @file   EyeDetector.cpp
  @author David Hirvonen
  @brief  Top level eye detection API class implementation.

  \copyright Copyright 2014-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of the eye detection class used
  for the top level SDK.
*/

#include <drishti/EyeDetector.hpp>
#include <drishti/drishti_cv.hpp>
#include <drishti/core/make_unique.h>

#include <acf/ACF.h>

#include <memory>
#include <opencv2/core/core.hpp>

_DRISHTI_SDK_BEGIN

struct EyeDetector::Impl
{
    Impl() = default;
    explicit Impl(const std::string& filename)
    {
        init(filename);
    }
    ~Impl() = default;

    Impl(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl& operator=(Impl&&) = delete;

    void init(const std::string& filename)
    {
        m_detector = drishti::core::make_unique<acf::Detector>(filename);

        // Perform modification
        acf::Detector::Modify dflt;
        dflt.cascThr = { "cascThr", -1.0 };
        dflt.cascCal = { "cascCal", +0.005 };
        m_detector->acfModify(dflt);
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

    std::unique_ptr<acf::Detector> m_detector;
};

// ######### EyeDetector ############

EyeDetector::EyeDetector() = default;
EyeDetector::EyeDetector(const std::string& filename)
{
    m_impl = drishti::core::make_unique<Impl>(filename);
}
EyeDetector::~EyeDetector() = default;
int EyeDetector::operator()(const Image3b& image, std::vector<Rect>& objects)
{
    return (*m_impl)(image, objects);
}

_DRISHTI_SDK_END
