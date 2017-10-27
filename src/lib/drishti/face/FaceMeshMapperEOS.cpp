/*! -*-c++-*-
  @file   FaceMeshMapperEOS.cpp
  @author David Hirvonen (from original code by Patrik Huber)
  @brief  Implementation of a FaceMeshMapper interface to the EOS library.

  This is based on sample code provided with the EOS library.
  See: https://github.com/patrikhuber/eos
 
  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceMeshMapper.h"
#include "drishti/face/FaceMeshMapperEOS.h"
#include "drishti/core/drishti_stdlib_string.h"

#include "eos/render/utils.hpp"
#include "eos/render/texture_extraction.hpp"

#include <glm/glm.hpp>

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

drishti::face::Frustum convert(const eos::fitting::Frustum& f)
{
    return { f.l, f.r, f.b, f.t };
}

Frustum FaceMeshContainerEOS::getFrustum() const
{
    return convert(rendering_params.get_frustum());
}

cv::Matx44f FaceMeshContainerEOS::getModelViewProjection() const
{
    const auto &mvp = rendering_params.get_modelview();
    
    cv::Matx44f MVP;
    for(int y = 0; y < 4; y++)
    {
        for(int x = 0; x < 4; x++)
        {
            MVP(y,x) = mvp[y][x];
        }
    }
    return MVP;
}

void FaceMeshContainerEOS::getFaceMesh(drishti::graphics::MeshTex &dest) const
{
    dest = { mesh.vertices, mesh.texcoords, mesh.tvi };
}

cv::Point3f FaceMeshContainerEOS::getRotation() const
{
    const auto euler = glm::eulerAngles(rendering_params.get_rotation());
    const float pitch = glm::degrees(euler[0]);
    const float yaw = glm::degrees(euler[1]);
    const float roll = glm::degrees(euler[2]);
    return cv::Point3f(pitch, yaw, roll);
}

void FaceMeshContainerEOS::setRotation(const cv::Point3f &R)
{
    // https://gamedev.stackexchange.com/a/13441
    rendering_params.set_rotation(glm::quat(glm::vec3(glm::radians(R.x), glm::radians(R.y), glm::radians(R.z))));
}

void FaceMeshContainerEOS::setQuaternion(const cv::Vec4f &Q)
{
    rendering_params.set_rotation(glm::quat(Q[0], Q[1], Q[2], Q[3]));
}

static cv::Vec4f convert(const glm::quat &Q)
{
    return {Q[0], Q[1], Q[2], Q[3]};
}

cv::Vec4f FaceMeshContainerEOS::getQuaternion() const
{
    return convert(rendering_params.get_rotation());
}

cv::Mat FaceMeshContainerEOS::extractTexture(const cv::Mat& image)
{
    //bool compute_view_angle = false;
    //TextureInterpolation mapping_type = TextureInterpolation::NearestNeighbour;
    //int isomap_resolution = 512;
    const auto interpolation = eos::render::TextureInterpolation::Bilinear;

    // Extract the texture from the image using given mesh and camera parameters:
    return eos::render::extract_texture(mesh, affine_from_ortho, image, false, interpolation);
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

void FaceMeshContainerEOS::serialize(const std::string& filename)
{
    // Save the mesh as textured obj:
    eos::core::write_textured_obj(mesh, filename);
}

void FaceMeshContainerEOS::drawWireFrameOnIso(cv::Mat& iso)
{
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

void FaceMeshContainerEOS::drawWireFrame(cv::Mat& iso)
{
    auto viewport = eos::fitting::get_opencv_viewport(iso.cols, iso.rows);
    GLMTransform transform{ rendering_params.get_modelview(), rendering_params.get_projection(), viewport };
    draw_wireframe(iso, mesh, transform);
}

DRISHTI_FACE_NAMESPACE_END
