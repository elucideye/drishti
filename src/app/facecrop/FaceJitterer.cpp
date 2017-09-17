/*! -*-c++-*-
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
void denormalize(Iter first, Iter last, const cv::Size& size)
{
    for (auto it = first; it != last; it++)
    {
        (*it) = { it->x * size.width, it->y * size.height };
    }
}

FaceJitterer::FaceJitterer(const FACE::Table& table, const FaceSpecification& face, const JitterParams& params)
    : m_table(table)
    , m_params(params)
    , m_face(face)
{
}

FaceWithLandmarks FaceJitterer::operator()(const cv::Mat& image, const Landmarks& landmarks, CropMode mode, bool doNoise)
{
    Landmarks eyesNoseMouth{
        mean(landmarks, m_table.eyeR),
        mean(landmarks, m_table.eyeL),
        mean(landmarks, m_table.nose),
        mean(landmarks, m_table.mouthR),
        mean(landmarks, m_table.mouthL)
    };
    const cv::Matx33f P = drishti::geometry::procrustes(eyesNoseMouth); // normalization transformation
    const cv::Matx33f S = m_face();                                     // scale to target image dimensions
    cv::Matx33f H = S * P;

    std::pair<cv::Matx33f, bool> jitter;
    switch (mode)
    {
        case kCrop:
            jitter = std::make_pair(cv::Matx33f::eye(), false);
            break;
        case kMirror:
            jitter = std::make_pair(m_params.scale(m_face.size, m_face.tl(), -1.f, 1.f), true);
            break;
        case kJitter:
            jitter = m_params(m_rng, m_face.size, m_face.tl());
            break;
    }

    jitter.first = jitter.first * H;

    cv::Mat face(m_face.paddedSize(), CV_8UC3, cv::Scalar::all(0));
    cv::warpAffine(image, face, jitter.first.get_minor<2, 3>(0, 0), face.size(), cv::INTER_CUBIC);

    if (doNoise)
    {
        // Set gain and noise (TODO) in transformed image:
        const float scale = m_params.getGain(m_rng);
        face.convertTo(face, CV_8UC3, scale);
    }

    FaceWithLandmarks result;
    result.image = face;
    result.landmarks.resize(landmarks.size());
    result.eyesNoseMouth.resize(eyesNoseMouth.size());

    // clang-format off
    std::transform(eyesNoseMouth.begin(), eyesNoseMouth.end(), result.eyesNoseMouth.begin(), [&](const cv::Point2f& p)
    {
        cv::Point3f q = jitter.first * cv::Point3f(p.x, p.y, 1.f);
        return cv::Point2f(q.x / q.z, q.y / q.z);
    });
    
    std::transform(landmarks.begin(), landmarks.end(), result.landmarks.begin(), [&](const cv::Point2f& p)
    {
        cv::Point3f q = jitter.first * cv::Point3f(p.x, p.y, 1.f);
        return cv::Point2f(q.x / q.z, q.y / q.z);
    });
    // clang-format on

    if (jitter.second)
    {
        result.flop(); // account for mirror component
    }

    return result;
}

cv::Point2f FaceJitterer::mean(const std::vector<cv::Point2f>& landmarks, const std::vector<int>& index)
{
    cv::Point2f mu;
    for (const auto& i : index)
    {
        mu += landmarks[i];
    }
    return mu * (1.f / index.size());
}
