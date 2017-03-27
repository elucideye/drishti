/*!
  @file   FaceJitterer
  @author David Hirvonen
  @brief  Face jitter utility to facilitate creation of training data.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FaceJitterer.h"
#include "JitterParams.h"

#include "drishti/geometry/motion.h"
#include "drishti/geometry/Primitives.h"

template <typename Iter>
void denormalize(Iter first, Iter last, const cv::Size &size)
{
    for(auto it = first; it != last; it++)
    {
        (*it) = { it->x * size.width, it->y * size.height };
    }
}

FaceJitterer::FaceJitterer(const FACE::Table &table, const JitterParams &params, const FaceSpecification &face)
    : m_table(table)
    , m_params(params)
    , m_face(face)
{
}

#define EYE_NORMALIZATION 0

cv::Mat FaceJitterer::operator()(const cv::Mat &image, const Landmarks &landmarks, bool doJitter, bool doNoise)
{
#if EYE_NORMALIZATION
    const std::array<cv::Point2f, 2> eyesCenters =
    {{
        mean(landmarks, m_table.eyeR),
        mean(landmarks, m_table.eyeL)
    }};
    cv::Matx33f H = transformation::estimateSimilarity(eyesCenters, m_eyes);
#else
    Landmarks facePoints
    {
        mean(landmarks, m_table.eyeR),
        mean(landmarks, m_table.eyeL),
        mean(landmarks, m_table.nose),
        mean(landmarks, m_table.mouthR),
        mean(landmarks, m_table.mouthL)
    };

    const cv::Matx33f P = drishti::geometry::procrustes(facePoints);
    const cv::Matx33f S = m_face();
    cv::Matx33f H = S * P;
#endif
    
    if(doJitter)
    {
        H = m_params(m_rng, m_face.size) * H;
    }

    cv::Mat face(m_face.size, CV_8UC3, cv::Scalar::all(0));
    cv::warpAffine(image, face, H.get_minor<2,3>(0,0), face.size(), cv::INTER_CUBIC);
    
    if(doNoise)
    {
        const float scale = m_params.getGain(m_rng);
        face.convertTo(face, CV_8UC3, scale);
    }
    
    return face;
}
    
cv::Point2f FaceJitterer::mean(const std::vector<cv::Point2f> &landmarks, const std::vector<int> &index)
{
    cv::Point2f mu;
    for(const auto &i : index)
    {
        mu += landmarks[i];
    }
    return mu * (1.f / index.size());
}
