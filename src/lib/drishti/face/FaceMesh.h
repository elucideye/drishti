/*! -*-c++-*-
  @file   FaceMesh.h
  @author David Hirvonen
  @brief  Internal declaration of a face landmark triangulation.

  \copyright Copyright 2014-2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceMesh_h__
#define __drishti_face_FaceMesh_h__

#include "drishti/face/drishti_face.h"

#include <array> // std::array<cv::Mat1f, 2>

#include <opencv2/core.hpp>

DRISHTI_FACE_NAMESPACE_BEGIN

class FaceMesh
{
public:
    using Landmarks = std::vector<cv::Point2f>;
    using Triangles = std::vector<cv::Vec6f>;

    FaceMesh();
    FaceMesh(const std::string& filename);

    void operator()(const Landmarks& landmarks, Triangles& mesh);
    Triangles delaunay(const Landmarks& landmarks, const cv::Size& size, bool doHalf = false);

    int writeTriangulation(const std::string& filename) const;
    int readTriangulation(const std::string& filename);

    std::array<cv::Mat1f, 2> transform(const Landmarks& a, const Landmarks& b, const cv::Size& size);
    std::array<cv::Mat1f, 2> transform(const Triangles& a, const Triangles& b, const cv::Size& size);

    const cv::Rect& getRoi() const { return m_roi; }
    const cv::Mat1b& getLabels() const { return m_labels; }

    void draw(cv::Mat& canvas, const Landmarks& landmarks, const Triangles& triangles);

protected:
    Triangles mirrorTriangulation(const Landmarks& landmarks, const std::vector<std::array<int, 2>>& mirrorMap);

    cv::Mat1b m_labels;
    cv::Rect m_roi;

    std::vector<cv::Vec3i> m_indices;

    static const std::vector<cv::Range> kContours;
    static const std::vector<cv::Range> kCurves;
    static const std::vector<std::array<int, 2>> kMirrorMap;
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceMesh_h__
