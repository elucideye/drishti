/*!
  @file   FaceMeshMapperLandmark.cpp
  @author David Hirvonen (from original code by Patrik Huber)
  @brief  Implementation of a FaceMeshMapper interface to the EOS library.
 
  This is based on sample code provided with the EOS library.
  See: https://github.com/patrikhuber/eos

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceMeshMapperLandmark.h"
#include "drishti/core/drishti_stdlib_string.h"

#include "eos/render/utils.hpp"
#include "eos/fitting/nonlinear_camera_estimation.hpp"
#include "eos/fitting/linear_shape_fitting.hpp"

DRISHTI_FACE_NAMESPACE_BEGIN

struct FaceMeshMapperLandmark::Impl
{
    using LandmarkSet = eos::core::LandmarkCollection<cv::Vec2f>;

    Impl(const std::string& modelfile, const std::string& mappingsfile)
    {
        morphable_model = eos::morphablemodel::load_model(modelfile);
        landmark_mapper = mappingsfile.empty() ? eos::core::LandmarkMapper() : eos::core::LandmarkMapper(mappingsfile);
    }

    Result operator()(const LandmarkSet& landmarks, const cv::Mat& image)
    {
        // These will be the final 2D and 3D points used for the fitting:
        std::vector<cv::Vec4f> model_points; // the points in the 3D shape model
        std::vector<int> vertex_indices;     // their vertex indices
        std::vector<cv::Vec2f> image_points; // the corresponding 2D landmark points

        // Sub-select all the landmarks which we have a mapping for (i.e. that are defined in the 3DMM):
        for (int i = 0; i < landmarks.size(); ++i)
        {
            auto converted_name = landmark_mapper.convert(landmarks[i].name);
            if (!converted_name)
            {
                // no mapping defined for the current landmark
                continue;
            }
            int vertex_idx = std::stoi(converted_name.get());
            auto vertex = morphable_model.get_shape_model().get_mean_at_point(vertex_idx);
            model_points.emplace_back(cv::Vec4f(vertex.x(), vertex.y(), vertex.z(), 1.0f));
            vertex_indices.emplace_back(vertex_idx);
            image_points.emplace_back(landmarks[i].coordinates);
        }

        // Estimate the camera (pose) from the 2D - 3D point correspondences
        auto rendering_params = eos::fitting::estimate_orthographic_camera(image_points, model_points, image.cols, image.rows);

        auto affine_from_ortho = get_3x4_affine_camera_matrix(rendering_params, image.cols, image.rows);

        // Estimate the shape coefficients by fitting the shape to the landmarks:
        auto fitted_coeffs = eos::fitting::fit_shape_to_landmarks_linear(morphable_model, affine_from_ortho, image_points, vertex_indices);

        // Obtain the full mesh with the estimated coefficients:
        auto mesh = morphable_model.draw_sample(fitted_coeffs, std::vector<float>());

        return Result{ mesh, rendering_params, affine_from_ortho };
    }

    Result operator()(const FaceModel& face, const cv::Mat& image)
    {
        return (*this)(extractLandmarks(face), image);
    }

    eos::morphablemodel::MorphableModel morphable_model;
    eos::core::LandmarkMapper landmark_mapper;
};

FaceMeshMapperLandmark::FaceMeshMapperLandmark(const std::string& modelfile, const std::string& mappingsfile)
{
    m_pImpl = std::make_shared<Impl>(modelfile, mappingsfile);
}

FaceMeshMapperLandmark::Result
FaceMeshMapperLandmark::operator()(const std::vector<cv::Point2f>& landmarks, const cv::Mat& image)
{
    return (*m_pImpl)(convertLandmarks(landmarks), image);
}

FaceMeshMapperLandmark::Result
FaceMeshMapperLandmark::operator()(const DRISHTI_FACE::FaceModel& face, const cv::Mat& image)
{
    return (*m_pImpl)(face, image);
}

DRISHTI_FACE_NAMESPACE_END
