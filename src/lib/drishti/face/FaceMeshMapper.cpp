/*! -*-c++-*-
  @file   FaceMeshMapper.cpp
  @author David Hirvonen (from original code by Patrik Huber)
  @brief  Implementation of a FaceMeshMapper interface to the EOS library.

  This is based on sample code provided with the EOS library.
  See: https://github.com/patrikhuber/eos
 
  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceMeshMapper.h"

#include "drishti/core/drishti_stdlib_string.h"

#include "eos/render/utils.hpp"
#include "eos/render/texture_extraction.hpp"

DRISHTI_FACE_NAMESPACE_BEGIN

struct GLMTransform
{
    glm::mat4x4 modelview;
    glm::mat4x4 projection;
    glm::vec4 viewport;
};

static void
draw_wireframe(cv::Mat& image, const eos::core::Mesh& mesh, GLMTransform& transform, const cv::Scalar& colour = { 0, 255, 0, 255 });

/// ===== UTILITY  ====

cv::Point3f getRotation(const eos::fitting::RenderingParameters& rendering_parameters)
{
    const auto euler = glm::eulerAngles(rendering_parameters.get_rotation());
    const float pitch = glm::degrees(euler[0]);
    const float yaw = glm::degrees(euler[1]);
    const float roll = glm::degrees(euler[2]);
    return cv::Point3f(pitch, yaw, roll);
}

cv::Mat extractTexture(const FaceMeshMapper::Result& result, const cv::Mat& image)
{
    //bool compute_view_angle = false;
    //TextureInterpolation mapping_type = TextureInterpolation::NearestNeighbour;
    //int isomap_resolution = 512;
    const auto interpolation = eos::render::TextureInterpolation::Bilinear;

    // Extract the texture from the image using given mesh and camera parameters:
    return eos::render::extract_texture(result.mesh, result.affine_from_ortho, image, false, interpolation);
}

eos::core::LandmarkCollection<cv::Vec2f> convertLandmarks(const std::vector<cv::Point2f>& points)
{
    int ibugId = 1;
    eos::core::LandmarkCollection<cv::Vec2f> landmarks;
    for (const auto& p : points)
    {
        eos::core::Landmark<cv::Vec2f> landmark;
        landmark.name = std::to_string(ibugId++);
        landmark.coordinates = { p.x, p.y };
        landmarks.emplace_back(landmark);
    }

    return landmarks;
}

static cv::Point2f interpolate(const cv::Point2f& p, const cv::Point2f& q, float f)
{
    return p + (q - p) * f;
}

eos::core::LandmarkCollection<cv::Vec2f> extractLandmarks(const DRISHTI_FACE::FaceModel& face)
{
    std::vector<cv::Point2f> points = face.points.get(); //  (*face.points);

    // Override global eye point estimates with eye models (if we have them):
    if (face.eyeFullL.has && face.eyeFullR.has)
    {
        std::vector<int> indexEyeR{ 36, 37, 38, 39, 40, 41 };
        std::vector<int> indexEyeL{ 45, 47, 46, 42, 44, 43 }; // mirrored

        const auto& eyeR = face.eyeFullR->eyelids;
        const auto& eyeL = face.eyeFullL->eyelids;

        float ratio = float(eyeR.size()) / float(indexEyeL.size());
        for (int i = 0; i < indexEyeR.size(); i++)
        {
            float f = float(i) * ratio;
            int k0 = int(f);
            int k1 = int(f + 0.5f);
            points[indexEyeR[i]] = interpolate(eyeR[k0], eyeR[k1], f - float(k0));
            points[indexEyeL[i]] = interpolate(eyeL[k0], eyeL[k1], f - float(k0));
        }
    }

    int ibugId = 1;
    eos::core::LandmarkCollection<cv::Vec2f> landmarks;
    for (const auto& p : points)
    {
        eos::core::Landmark<cv::Vec2f> landmark;
        landmark.name = std::to_string(ibugId++);
        landmark.coordinates = { p.x, p.y };
        landmarks.emplace_back(landmark);
    }

    return landmarks;
}

void draw_wireframe(cv::Mat& image, const eos::core::Mesh& mesh, GLMTransform& transform, const cv::Scalar& colour)
{
    const auto& modelview = transform.modelview;
    const auto& projection = transform.projection;
    const auto& viewport = transform.viewport;

    for (const auto& triangle : mesh.tvi)
    {
        const auto& v0 = mesh.vertices[triangle[0]];
        const auto& v1 = mesh.vertices[triangle[1]];
        const auto& v2 = mesh.vertices[triangle[2]];

        const auto p1 = glm::project({ v0[0], v0[1], v0[2] }, modelview, projection, viewport);
        const auto p2 = glm::project({ v1[0], v1[1], v1[2] }, modelview, projection, viewport);
        const auto p3 = glm::project({ v2[0], v2[1], v2[2] }, modelview, projection, viewport);
        if (eos::render::detail::are_vertices_ccw_in_screen_space(glm::vec2(p1), glm::vec2(p2), glm::vec2(p3)))
        {
            cv::line(image, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), colour);
            cv::line(image, cv::Point(p2.x, p2.y), cv::Point(p3.x, p3.y), colour);
            cv::line(image, cv::Point(p3.x, p3.y), cv::Point(p1.x, p1.y), colour);
        }
    }
};

void serialize(const eos::core::Mesh& mesh, const std::string& filename)
{
    // Save the mesh as textured obj:
    eos::core::write_textured_obj(mesh, filename);
}

void drawWireFrameOnIso(cv::Mat& iso, const eos::core::Mesh& meshIn)
{
    auto mesh = meshIn;
    for (auto& p : mesh.texcoords)
    {
        p[0] *= iso.cols;
        p[1] *= iso.rows;
    }

    for (int i = 0; i < mesh.tvi.size(); i++)
    {
        const auto& t = mesh.tvi[i];

        const auto& p0 = mesh.texcoords[t[0]];
        const auto& p1 = mesh.texcoords[t[1]];
        const auto& p2 = mesh.texcoords[t[2]];

        cv::Point2f v0(p0[0], p0[1]);
        cv::Point2f v1(p1[0], p1[1]);
        cv::Point2f v2(p2[0], p2[1]);

        cv::line(iso, v0, v1, { 0, 255, 0 }, 1, 8);
        cv::line(iso, v1, v2, { 0, 255, 0 }, 1, 8);
        cv::line(iso, v2, v0, { 0, 255, 0 }, 1, 8);
    }
}

void drawWireFrame(cv::Mat& iso, const FaceMeshMapper::Result& result)
{
    const auto& mesh = result.mesh;
    const auto& rendering_params = result.rendering_params;
    auto viewport = eos::fitting::get_opencv_viewport(iso.cols, iso.rows);
    GLMTransform transform{ rendering_params.get_modelview(), rendering_params.get_projection(), viewport };
    draw_wireframe(iso, mesh, transform);
}

DRISHTI_FACE_NAMESPACE_END
