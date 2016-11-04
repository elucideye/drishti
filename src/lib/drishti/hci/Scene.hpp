/*!
  @file   finder/Scene.hpp
  @author David Hirvonen
  @brief  Scene viewed by the camera represented by low level primitives: (corners, face, flow, etc.)

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef SCENE_H
#define SCENE_H

#include "drishti/hci/gpu/LineDrawing.hpp"
#include "drishti/face/Face.h"
#include "drishti/acf/ACF.h"
#include <opencv2/core/core.hpp>
#include <vector>

// *INDENT-OFF*
namespace drishti { namespace acf { class Detector; } };
// *INDENT-ON*

struct ScenePrimitives
{
    ScenePrimitives(uint64_t frameIndex) : m_frameIndex(frameIndex) {}

    const std::vector<drishti::face::FaceModel> &faces() const
    {
        return m_faces;
    }
    std::vector<drishti::face::FaceModel> &faces()
    {
        return m_faces;
    }

    const std::vector<cv::Rect> &objects() const
    {
        return m_objects;
    }
    std::vector<cv::Rect> &objects()
    {
        return m_objects;
    }

    const std::vector<cv::Point2f> &corners() const
    {
        return m_corners;
    }
    std::vector<cv::Point2f> &corners()
    {
        return m_corners;
    }

    const std::vector<cv::Vec4f> &flow() const
    {
        return m_flow;
    }
    std::vector<cv::Vec4f> &flow()
    {
        return m_flow;
    }

    std::vector<cv::Vec3f> colors; // should match flow

    const std::shared_ptr<drishti::acf::Detector::Pyramid> & pyramid() const
    {
        return m_P;
    }
    std::shared_ptr<drishti::acf::Detector::Pyramid> & pyramid()
    {
        return m_P;
    }

    const cv::Mat & image() const
    {
        return m_image;
    }
    cv::Mat & image()
    {
        return m_image;
    }

    void clear()
    {
        m_flow.clear();
        m_corners.clear();
        m_objects.clear();
        m_faces.clear();
    }

    uint64_t m_frameIndex = 0;
    cv::Mat m_image;

    std::vector<cv::Vec4f> m_flow; // Temporary
    std::vector<cv::Point2f> m_corners;
    std::vector<cv::Rect> m_objects;
    std::vector<drishti::face::FaceModel> m_faces;

    std::shared_ptr<drishti::acf::Detector::Pyramid> m_P;
};

// TODO: use more generic back_inserter<> approach:
using LineDrawingVec = std::vector<ogles_gpgpu::LineDrawing>;
void pointsToCrosses(const std::vector<cv::Point2f> &points, LineDrawingVec &crosses);
void rectanglesToDrawings(const std::vector<cv::Rect> &rectangles, LineDrawingVec &drawings);
void facesToDrawings(const std::vector<drishti::face::FaceModel> &faces, LineDrawingVec &drawings);
void rectangleToDrawing(const cv::Rect &r, ogles_gpgpu::LineDrawing &drawing, bool closed=true);
void flowToDrawings(const std::vector<cv::Vec4f> &flow, LineDrawingVec &drawings, const cv::Mat3f &colorMap);

#endif // SCENE_H
