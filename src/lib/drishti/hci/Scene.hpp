/*! -*-c++-*-
  @file   finder/Scene.hpp
  @author David Hirvonen
  @brief  Scene viewed by the camera represented by low level primitives: (corners, face, flow, etc.)

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_Scene_hpp__
#define __drishti_hci_Scene_hpp__

#include "drishti/hci/drishti_hci.h"
#include "drishti/hci/gpu/LineDrawing.hpp"
#include "drishti/face/Face.h"

#include <acf/ACF.h>

#include <opencv2/core/core.hpp>

#include <vector>

DRISHTI_HCI_NAMESPACE_BEGIN

using LineDrawingVec = std::vector<ogles_gpgpu::LineDrawing>;

struct FeaturePoint
{
    FeaturePoint() {}
    FeaturePoint(const cv::Point2f& point)
        : point(point)
    {
    }
    FeaturePoint(const cv::Point2f& point, float radius)
        : point(point)
        , radius(radius)
    {
    }
    cv::Point2f point;
    float radius = 1.f;
};

struct ScenePrimitives
{
    ScenePrimitives()
        : m_frameIndex(0)
    {
    }

    ScenePrimitives(uint64_t frameIndex)
        : m_frameIndex(frameIndex)
    {
    }

    const std::vector<drishti::face::FaceModel>& faces() const
    {
        return m_faces;
    }
    std::vector<drishti::face::FaceModel>& faces()
    {
        return m_faces;
    }

    const std::vector<cv::Rect>& objects() const
    {
        return m_objects;
    }
    std::vector<cv::Rect>& objects()
    {
        return m_objects;
    }

    const std::vector<cv::Point2f>& corners() const
    {
        return m_corners;
    }
    std::vector<cv::Point2f>& corners()
    {
        return m_corners;
    }

    const std::vector<cv::Vec4f>& flow() const
    {
        return m_flow;
    }
    std::vector<cv::Vec4f>& flow()
    {
        return m_flow;
    }

    std::vector<cv::Vec3f> colors; // should match flow

    const std::shared_ptr<acf::Detector::Pyramid>& pyramid() const
    {
        return m_P;
    }
    std::shared_ptr<acf::Detector::Pyramid>& pyramid()
    {
        return m_P;
    }

    const cv::Mat& image() const
    {
        return m_image;
    }
    cv::Mat& image()
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

    const std::vector<ogles_gpgpu::LineDrawing>& getDrawings() const
    {
        return m_drawings;
    }
    std::vector<ogles_gpgpu::LineDrawing>& getDrawings()
    {
        return m_drawings;
    }

    void draw(bool doFaces = true, bool doPupils = true, bool doCorners = true);

    uint64_t m_frameIndex = 0;
    cv::Mat m_image;

    std::vector<cv::Vec4f> m_flow; // Temporary
    std::vector<cv::Point2f> m_corners;
    std::vector<cv::Rect> m_objects;
    std::vector<drishti::face::FaceModel> m_faces;
    std::shared_ptr<acf::Detector::Pyramid> m_P;

    // Drawing cache:
    std::vector<ogles_gpgpu::LineDrawing> m_drawings;
    std::vector<std::vector<cv::Point2f>> m_eyeDrawings[2];
};

void extractPoints(const cv::Mat1b& input, std::vector<FeaturePoint>& points, float flowScale);
void pointsToCircles(const std::vector<FeaturePoint>& points, LineDrawingVec& circles, float width = 8.f);
void pointsToCrosses(const std::vector<cv::Point2f>& points, LineDrawingVec& crosses, float width = 8.f);
void pointsToCrosses(const std::vector<FeaturePoint>& points, LineDrawingVec& crosses, float width = 8.f);
void rectanglesToDrawings(const std::vector<cv::Rect>& rectangles, LineDrawingVec& drawings);
void facesToDrawings(const std::vector<drishti::face::FaceModel>& faces, LineDrawingVec& drawings);
void rectangleToDrawing(const cv::Rect& r, ogles_gpgpu::LineDrawing& drawing, bool closed = true);
void flowToDrawings(const std::vector<cv::Vec4f>& flow, LineDrawingVec& drawings, const cv::Mat3f& colorMap);

DRISHTI_HCI_NAMESPACE_END

#endif // __drishti_hci_Scene_hpp__
