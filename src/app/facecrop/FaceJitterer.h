/*!
  @file   FaceJitterer
  @author David Hirvonen
  @brief  Face jitter utility to facilitate creation of training data.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __facecrop_FaceJitterer_h__
#define __facecrop_FaceJitterer_h__

#include "FaceSpecification.h"
#include "JitterParams.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "landmarks/FACE.h"

#include <opencv2/core.hpp>

#include <array>

class FaceJitterer
{
public:
    
    using Landmarks = std::vector<cv::Point2f>;
    
    FaceJitterer(const FACE::Table &table, const JitterParams &params, const FaceSpecification &face);
    cv::Mat operator()(const cv::Mat &image, const Landmarks &landmarks, bool doJitter, bool doNoise);
    void setDoPreview(bool value) { m_doPreview = value; }
    bool getDoPreview() const { return m_doPreview; }    

protected:
    
    static cv::Point2f mean(const Landmarks &landmarks, const std::vector<int> &index);
    
    const FACE::Table &m_table;
    const JitterParams &m_params;
    const FaceSpecification &m_face;
    std::array<cv::Point2f, 2> m_eyes;
    bool m_doPreview = false;
    
    cv::RNG m_rng;
};

#endif // __facecrop_FaceJitterer_h__
