/*! -*-c++-*-
  @file   finder/Scene.cpp
  @author David Hirvonen
  @brief  Scene viewed by the camera represented by low level primitives: (corners, face, flow, etc.)

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/Scene.hpp"

DRISHTI_HCI_NAMESPACE_BEGIN

void ScenePrimitives::draw(bool doFaces, bool doPupils, bool doCorners)
{
    if (doCorners)
    {
        pointsToCrosses(m_corners, m_drawings);
    }

    // Cache eye models:
    m_eyeDrawings[0].clear();
    m_eyeDrawings[1].clear();
    if (m_faces.size())
    {
        facesToDrawings(m_faces, m_drawings);
        if (m_faces.size() && m_faces[0].eyeFullL.has && m_faces[0].eyeFullR.has)
        {
            m_eyeDrawings[0] = m_faces[0].eyeFullR->getContours(doPupils);
            m_eyeDrawings[1] = m_faces[0].eyeFullL->getContours(doPupils);
        }
    }
}

static float getAngle(const cv::Point2f& p)
{
    return (std::atan2(p.y, p.x) + M_PI) * 180.0 / M_PI;
}

void flowToDrawings(const std::vector<cv::Vec4f>& flow, LineDrawingVec& drawings, const cv::Mat3f& colorMap)
{
    drawings.reserve(drawings.size() + flow.size());
    for (int i = 0; i < flow.size(); i++)
    {
        ogles_gpgpu::LineDrawing drawing;
        const auto& f = flow[i];
        const cv::Point2f p(f[0], f[1]);
        const cv::Point2f q(f[2], f[3]);
        const float angle = getAngle(q);
        const int index = std::max(0, std::min(int(angle + 0.5f), int(colorMap.cols - 1)));

        drawing.strip = false; // STRIP == FALSE
        drawing.color = colorMap(index);
        drawing.contours = { { p, p + q } };
        drawings.emplace_back(drawing);
    }
}

void pointsToCircles(const std::vector<FeaturePoint>& points, LineDrawingVec& circles, float width)
{
    cv::Point2f dx(width, 0.0), dy(0.0, width);

    float maxRadius = points.size() ? points.front().radius : std::numeric_limits<float>::max();

    static const int tics = 16;
    for (const auto& f : points)
    {
        if (f.radius > (maxRadius * 0.5f))
        {
            const auto& p = f.point;

            ogles_gpgpu::LineDrawing circle;
            circle.strip = false; // STRIP == FALSE
            circle.index = { 0 };
            circle.contours.resize(1);

            std::vector<cv::Point2f> contour(tics);
            for (int i = 0; i < contour.size(); i++)
            {
                const double theta = M_PI * 2.0 * static_cast<double>(i) / tics;
                const float scale = f.radius * width;
                const cv::Point2f r(std::cos(theta) * scale, std::sin(theta) * scale);
                contour[i] = (p + r);
            }

            circle.contours[0].push_back(contour.back());
            circle.contours[0].push_back(contour.front());
            for (int i = 1; i < contour.size(); i++)
            {
                circle.contours[0].push_back(contour[i - 1]);
                circle.contours[0].push_back(contour[i + 0]);
            }

            circles.emplace_back(circle);
        }
    }
}

// weights [0..1] (optional)
void pointsToCrosses(const std::vector<FeaturePoint>& points, LineDrawingVec& crosses, float width)
{
    cv::Point2f dx(width, 0.0), dy(0.0, width);

    for (const auto& f : points)
    {
        const auto& p = f.point;
        ogles_gpgpu::LineDrawing cross;
        cross.strip = false; // STRIP == FALSE
        cross.index = { 0 };
        cross.contours = { { p - dx * f.radius, p + dx * f.radius }, { p - dy * f.radius, p + dy * f.radius } };
        crosses.emplace_back(cross);
    }
}

void pointsToCrosses(const std::vector<cv::Point2f>& points, LineDrawingVec& crosses, float width)
{
    cv::Point2f dx(width, 0.0), dy(0.0, width);

    for (const auto& p : points)
    {
        ogles_gpgpu::LineDrawing cross;
        cross.strip = false; // STRIP == FALSE
        cross.index = { 0 };
        cross.contours = { { p - dx, p + dx }, { p - dy, p + dy } };
        crosses.emplace_back(cross);
    }
}

void rectangleToDrawing(const cv::Rect& r, ogles_gpgpu::LineDrawing& drawing, bool closed)
{
    const cv::Point2f tl = r.tl(), br = r.br(), tr(br.x, tl.y), bl(tl.x, br.y);
    drawing.strip = true; // STRIP == TRUE
    drawing.contours = { { tl, tr, br, bl } };
    if (closed)
    {
        drawing.contours.back().push_back(tl);
    }
    drawing.index = { 0 };
}

void rectanglesToDrawings(const std::vector<cv::Rect>& rectangles, LineDrawingVec& drawings)
{
    for (const auto& r : rectangles)
    {
        ogles_gpgpu::LineDrawing drawing;
        rectangleToDrawing(r, drawing);
        drawings.emplace_back(drawing);
    }
}

void faceToDrawing(const drishti::face::FaceModel& face, ogles_gpgpu::LineDrawing& drawing)
{
    auto parts = face.getFaceParts(true, false);
    drawing.strip = true; // STRIP == TRUE
    drawing.roi = face.roi;
    for (int i = 0; i < parts.size(); i++)
    {
        for (auto& c : parts[i])
        {
            if (c.size())
            {
                drawing.index.push_back(i);
                drawing.contours.push_back(c);
            }
        }
    }
}

void facesToDrawings(const std::vector<drishti::face::FaceModel>& faces, LineDrawingVec& drawings)
{
    for (const auto& f : faces)
    {
        {
            ogles_gpgpu::LineDrawing drawing;
            rectangleToDrawing(f.roi, drawing);
            drawings.emplace_back(drawing);
        }
        {
            ogles_gpgpu::LineDrawing drawing;
            faceToDrawing(f, drawing);
            drawings.emplace_back(drawing);
        }
    }
}

void extractPoints(const cv::Mat1b& input, std::vector<drishti::hci::FeaturePoint>& features, float scale)
{
    // ### Extract corners first: ###
    std::vector<cv::Point> points;
    try
    {
        cv::findNonZero(input, points);
    }
    catch (...)
    {
    }

    features.reserve(points.size());
    for (const auto& p : points)
    {
        uint8_t value = input(p);
        const float radius = static_cast<float>(value) / 255.f;
        features.emplace_back(cv::Point2f(scale * p.x, scale * p.y), radius);
    }

    std::sort(features.begin(), features.end(), [](const FeaturePoint& pa, const FeaturePoint& pb) {
        return (pa.radius > pb.radius);
    });
}

DRISHTI_HCI_NAMESPACE_END
