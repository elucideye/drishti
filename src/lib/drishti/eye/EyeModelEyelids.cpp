/*! -*-c++-*-
  @file   EyeModelEyelids.cpp
  @author David Hirvonen
  @brief  Implemenation of eyelid specific eye model estimation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of eyelid specific estimation/regression used
  to create the final eye model.  The eyelid model is augmented with a coarse
  iris and pupil model in shape space, which help provide a coarse global eye model --
  both of these can optionally be refined using (see EyeModel{Iris,Pupil}.cpp)

*/

#include "drishti/eye/EyeModelEstimatorImpl.h"
#include "drishti/core/drishti_stdlib_string.h" // FIRST
#include "drishti/eye/EyeIO.h"

#define DRISHTI_EYE_DEBUG_INITS 0

// clang-format off
#if DRISHTI_EYE_DEBUG_INITS
#  include <opencv2/highgui.hpp>
#endif
// clang-format on

DRISHTI_EYE_NAMESPACE_BEGIN

using PointVec = std::vector<cv::Point2f>;
static void jitter(const EyeModel& eye, const geometry::UniformSimilarityParams& params, std::vector<EyeModel>& poses, int n);
static void jitter(const cv::Rect& roi, const geometry::UniformSimilarityParams& params, std::vector<cv::Rect>& poses, int n);
static PointVec getMedianOfPoses(const std::vector<PointVec>& poses);

#if DRISHTI_EYE_DEBUG_INITS
static std::vector<cv::Point2f> operator*(const cv::Matx33f& H, const std::vector<cv::Point2f>& points);
static void drawEyes(const cv::Mat& I, const std::vector<EyeModel>& eyes, const std::string& name = "eyes");
static std::vector<EyeModel> shapesToEyes(const std::vector<PointVec>& shapes, const EyeModelSpecification& spec, const cv::Matx33f& S);
#endif

void EyeModelEstimator::Impl::segmentEyelids(const cv::Mat& I, EyeModel& eye) const
{
    PointVec mu = m_eyeEstimator->getMeanShape();

    cv::Rect roi({ 0, 0 }, I.size());
    std::vector<cv::Rect> rois = { roi };
    if (m_eyelidInits > 1)
    {
        jitter(roi, m_jitterEyelidParams, rois, m_eyelidInits - 1);
    }

    std::vector<PointVec> poses(rois.size(), mu);

    // Get the basic shape:
    std::vector<bool> mask; // occlusion mask
    for (int i = 0; i < rois.size(); i++)
    {
        (*m_eyeEstimator)(I(rois[i]), poses[i], mask);
        cv::Point2f shift = rois[i].tl();
        for (auto& p : poses[i])
        {
            p += shift;
        }
    }

    // Get median of poses:
    PointVec pose = (poses.size() > 1) ? getMedianOfPoses(poses) : poses[0];
    eye = shapeToEye(poses[0], m_eyeSpec);

#if DRISHTI_EYE_DEBUG_INITS
    drawEyes(I, shapesToEyes(poses, m_eyeSpec, cv::Matx33f::eye()), "poses-out");
#endif
}

void EyeModelEstimator::Impl::segmentEyelids_(const cv::Mat& I, EyeModel& eye) const
{
    std::vector<PointVec> poses{ m_eyeEstimator->getMeanShape() };

    // Only try multiple inits for non-pca:
    //const_cast<int&>(m_eyelidInits) = 5;

    if (m_eyelidInits > 1)
    {
        auto toShape = [&](const EyeModel& e) {
            return eyeToShape(e, m_eyeSpec);
        };
        std::vector<EyeModel> jittered;
        jitter(shapeToEye(m_eyeEstimator->getMeanShape(), m_eyeSpec), m_jitterEyelidParams, jittered, m_eyelidInits - 1);
        std::transform(jittered.begin(), jittered.end(), std::back_inserter(poses), toShape);
    }

#if DRISHTI_EYE_DEBUG_INITS
    drawEyes(I, shapesToEyes(poses, m_eyeSpec, cv::Matx33f::diag({ I.cols, I.cols, 1.f })), "poses-in");
#endif

    // Get the basic shape:
    std::vector<bool> mask; // occlusion mask
    for (int i = 0; i < poses.size(); i++)
    {
        (*m_eyeEstimator)(I, poses[i], mask);
    }

    // Get median of poses:
    PointVec pose = (poses.size() > 1) ? getMedianOfPoses(poses) : poses[0];
    eye = shapeToEye(poses[0], m_eyeSpec);

#if DRISHTI_EYE_DEBUG_INITS
    drawEyes(I, shapesToEyes(poses, m_eyeSpec, cv::Matx33f::eye()), "poses-out");
#endif
}

#if DRISHTI_EYE_DEBUG_INITS
// #### utility functions ####
static std::vector<EyeModel> shapesToEyes(const std::vector<PointVec>& shapes, const EyeModelSpecification& spec, const cv::Matx33f& S)
{
    auto toEye = [&](const PointVec& shape) {
        return S * shapeToEye(shape, spec);
    };
    std::vector<EyeModel> eyes;
    std::transform(shapes.begin(), shapes.end(), std::back_inserter(eyes), toEye);
    return eyes;
};

std::vector<cv::Point2f> operator*(const cv::Matx33f& H, const std::vector<cv::Point2f>& points)
{
    std::vector<cv::Point2f> points_ = points;
    for (auto& p : points_)
    {
        cv::Point3f q = H * cv::Point3f(p.x, p.y, 1.f);
        p = { q.x / q.z, q.y / q.z };
    }
    return points_;
}

static float getMaxSeparation(const PointVec& points)
{
    float maxSeparation = 0.f;
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = i + 1; j < points.size(); j++)
        {
            float separation = cv::norm(points[i] - points[j]);
            if (separation > maxSeparation)
            {
                maxSeparation = separation;
            }
        }
    }
    return maxSeparation;
}

static void drawEyes(const cv::Mat& I, const std::vector<EyeModel>& eyes, const std::string& name)
{
    cv::Mat canvas;
    cv::cvtColor(I, canvas, cv::COLOR_GRAY2BGR);

    for (const auto& v : eyes)
    {
        cv::Matx41d color = cv::Scalar::randu(100, 255);
        v.draw(canvas, 0, 0, { color(0), color(1), color(2) }, 1);
    }

    cv::imshow(name, canvas);
    cv::waitKey(0);
}

#endif // DRISHTI_EYE_DEBUG_INITS

static void jitter(const cv::Rect& roi, const geometry::UniformSimilarityParams& params, std::vector<cv::Rect>& poses, int n)
{
    cv::Point2f center = drishti::geometry::centroid<int, float>(roi);
    geometry::UniformSimilarityParams params_ = params;
    for (int i = 0; i < n; i++)
    {
        bool hasRoi = false;
        cv::Rect roi2;
        for (int j = 0; j < 100; j++)
        {
            cv::Matx33f H = geometry::randomSimilarity(params_, cv::theRNG(), center, false);
            roi2 = H * roi;

            cv::Rect valid = roi2 & roi;
            if (roi.contains(valid.tl()) && roi.contains(valid.br()))
            {
                hasRoi = true;
                break;
            }
        }
        if (hasRoi)
        {
            poses.push_back(roi2);
        }
    }
}

static void jitter(const EyeModel& eye, const geometry::UniformSimilarityParams& params, std::vector<EyeModel>& poses, int n)
{
    // TODO: move this to a config file:
    cv::Point2f center = drishti::core::centroid(eye.eyelids);
    geometry::UniformSimilarityParams params_ = params;
    for (int i = 0; i < n; i++)
    {
        cv::Matx33f H = geometry::randomSimilarity(params_, cv::theRNG(), center);
        poses.push_back(H * eye);
    }
}

static PointVec getMedianOfPoses(const std::vector<PointVec>& poses)
{
    std::vector<std::vector<float>> params[2];
    params[0].resize(poses[0].size());
    params[1].resize(poses[0].size());
    for (int i = 0; i < poses.size(); i++)
    {
        for (int j = 0; j < poses[i].size(); j++)
        {
            params[0][j].push_back(poses[i][j].x);
            params[1][j].push_back(poses[i][j].y);
        }
    }

    // Take the median:
    std::vector<cv::Point2f> pose(poses[0].size());
    for (int i = 0; i < params[0].size(); i++)
    {
        pose[i] = { median(params[0][i]), median(params[1][i]) };
    }

    return pose;
}

DRISHTI_EYE_NAMESPACE_END
