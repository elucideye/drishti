/*
 @file   FaceLandmarContourkMeshMapper.cpp
 @author David Hirvonen (from original code by Patrik Huber)
 @brief  Implementation of a FaceMeshMapper interface to the EOS library.

 This is based on sample code provided with the EOS library.
 See: https://github.com/patrikhuber/eos
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/drishti_stdlib_string.h"

#include "drishti/face/FaceMeshMapperEOSLandmarkContour.h"
#include "drishti/face/FaceMeshMapperEOS.h"

#include "eos/core/Landmark.hpp"
#include "eos/core/LandmarkMapper.hpp"
#include "eos/morphablemodel/MorphableModel.hpp"
#include "eos/morphablemodel/Blendshape.hpp"
#include "eos/fitting/fitting.hpp"
#include "eos/fitting/nonlinear_camera_estimation.hpp"
#include "eos/render/utils.hpp"

#include "opencv2/core.hpp"

#include <vector>
#include <iostream>
#include <fstream>

DRISHTI_FACE_NAMESPACE_BEGIN

struct FaceMeshMapperEOSLandmarkContour::Impl
{
    using LandmarkSet = eos::core::LandmarkCollection<cv::Vec2f>;
    using FaceMeshContainerPtr = std::shared_ptr<FaceMeshContainer>;

    Impl(const FaceMeshMapperEOSLandmarkContour::Assets& assets)
    {
        morphable_model = eos::morphablemodel::load_model(assets.model);
        landmark_mapper = assets.mappings.empty() ? eos::core::LandmarkMapper() : eos::core::LandmarkMapper(assets.mappings);
        blendshapes = eos::morphablemodel::load_blendshapes(assets.blendshapes);
        model_contour = assets.contour.empty() ? eos::fitting::ModelContour() : eos::fitting::ModelContour::load(assets.contour);
        ibug_contour = eos::fitting::ContourLandmarks::load(assets.mappings);
        edge_topology = eos::morphablemodel::load_edge_topology(assets.edgetopology);
    }

    auto operator()(const LandmarkSet& landmarks, const cv::Mat& image) -> FaceMeshContainerPtr
    {
        // Fit the model, get back a mesh and the pose:
        eos::core::Mesh mesh;
        eos::fitting::RenderingParameters rendering_params;
        std::tie(mesh, rendering_params) = eos::fitting::fit_shape_and_pose(
            morphable_model,
            blendshapes,
            landmarks,
            landmark_mapper,
            image.cols,
            image.rows,
            edge_topology,
            ibug_contour,
            model_contour,
            50, boost::none, 30.0f);
        cv::Mat affine_from_ortho = eos::fitting::get_3x4_affine_camera_matrix(rendering_params, image.cols, image.rows);
        
        return std::make_shared<FaceMeshContainerEOS>(mesh, rendering_params, affine_from_ortho);
    }

    auto operator()(const FaceModel& face, const cv::Mat& image) -> FaceMeshContainerPtr
    {
        return (*this)(extractLandmarks(face), image);
    }

    eos::morphablemodel::MorphableModel morphable_model;
    eos::core::LandmarkMapper landmark_mapper;
    std::vector<eos::morphablemodel::Blendshape> blendshapes;
    eos::fitting::ModelContour model_contour;
    eos::fitting::ContourLandmarks ibug_contour;
    eos::morphablemodel::EdgeTopology edge_topology;
};

FaceMeshMapperEOSLandmarkContour::FaceMeshMapperEOSLandmarkContour(const Assets& assets)
{
    m_pImpl = std::make_shared<Impl>(assets);
}

auto FaceMeshMapperEOSLandmarkContour::operator()(const std::vector<cv::Point2f>& landmarks, const cv::Mat& image) -> FaceMeshContainerPtr
{
    return (*m_pImpl)(convertLandmarks(landmarks), image);
}

auto FaceMeshMapperEOSLandmarkContour::operator()(const FaceModel& face, const cv::Mat& image) -> FaceMeshContainerPtr
{
    return (*m_pImpl)(face, image);
}

DRISHTI_FACE_NAMESPACE_END
