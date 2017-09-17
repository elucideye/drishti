/*! -*-c++-*-
  @file   FaceMesh.cpp
  @author David Hirvonen
  @brief  Internal implementation of a face landmark triangulation.

  \copyright Copyright 2014-2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceMesh.h"

#include <opencv2/imgproc.hpp>

#include <iostream>
#include <array>

DRISHTI_FACE_NAMESPACE_BEGIN

static cv::Mat getAffineLeastSquares(const cv::Vec6f& T1, const cv::Vec6f& T2);

FaceMesh::FaceMesh()
{
}

FaceMesh::FaceMesh(const std::string& filename)
{
    readTriangulation(filename);
}

void FaceMesh::operator()(const Landmarks& landmarks, Triangles& mesh)
{
    cv::Size size(1000, 1000);
    mesh = delaunay(landmarks, size);
}

std::array<cv::Mat1f, 2> FaceMesh::transform(const Triangles& a, const Triangles& b, const cv::Size& size)
{
    CV_Assert(a.size() == b.size());

    const int n = a.size();
    std::vector<cv::Matx33f> M(n, cv::Matx33f::eye());

    cv::Mat1b mask(size, 0);
    for (int i = 0; i < n; i++)
    {
        const auto& tA = a[i];
        std::vector<std::vector<cv::Point>> pointsA{ { cv::Point(tA[0], tA[1]),
            cv::Point(tA[2], tA[3]),
            cv::Point(tA[4], tA[5]) } };

        const auto& tB = b[i];
        std::vector<std::vector<cv::Point>> pointsB{ { cv::Point(tB[0], tB[1]),
            cv::Point(tB[2], tB[3]),
            cv::Point(tB[4], tB[5]) } };

        int value = (i + 1);
        cv::fillPoly(mask, pointsA, value, 4);

        // Create affine transformations from a to b
        cv::Mat H = getAffineLeastSquares(tA, tB);
        H.convertTo(cv::Mat1f(M[i].rows, M[i].cols, (float*)&M[i](0, 0))(cv::Rect({ 0, 0 }, H.size())), CV_32F);
    }

    // Mirror image and take mean:
    cv::Mat1f mapx(size, 0.f);
    cv::Mat1f mapy(size, 0.f);
    for (int y = 0; y < size.height; y++)
    {
        for (int x = 0; x < size.width; x++)
        {
            int index = mask(y, x);
            if (index-- > 0)
            {
                cv::Point3f p(x, y, 1.f), q = M[index] * p;
                mapx(y, x) = q.x;
                mapy(y, x) = q.y;
            }
        }
    }

    std::array<cv::Mat1f, 2> flow{ { mapx, mapy } };

    return flow;
}

std::array<cv::Mat1f, 2> FaceMesh::transform(const Landmarks& a, const Landmarks& b, const cv::Size& size)
{
    CV_Assert(a.size() == b.size());
    return transform(delaunay(a, size), delaunay(b, size), size);
}

FaceMesh::Triangles FaceMesh::delaunay(const Landmarks& landmarks, const cv::Size& size, bool doHalf)
{
    std::vector<cv::Vec6f> trianglesL, trianglesR;

    if (m_indices.size())
    {
        // Create triangles from indices
        trianglesL.resize(m_indices.size());
        for (int i = 0; i < m_indices.size(); i++)
        {
            cv::Point2f p1 = landmarks[kMirrorMap[m_indices[i][0]][0]];
            cv::Point2f p2 = landmarks[kMirrorMap[m_indices[i][1]][0]];
            cv::Point2f p3 = landmarks[kMirrorMap[m_indices[i][2]][0]];
            trianglesL[i] = cv::Vec6f(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
        }
    }
    else
    {
        // Perform the delaunay triangulation

        // Use a delaunay subdivision and balance mirror triangles:
        cv::Rect roi({ 0, 0 }, size);
        cv::Subdiv2D subdiv;
        subdiv.initDelaunay(roi);
        for (int i = 0; i < kMirrorMap.size(); i++)
        {
            auto p = landmarks[kMirrorMap[i][0]];
            subdiv.insert(p);
        }
        std::vector<cv::Vec3i> indices;
        subdiv.getTriangleList(trianglesL);

        auto pruner = [&](const cv::Vec6f& triangle) {
            cv::Point2f p1(triangle[0], triangle[1]);
            cv::Point2f p2(triangle[2], triangle[3]);
            cv::Point2f p3(triangle[4], triangle[5]);
            return (!(roi.contains(p1) && roi.contains(p2) && roi.contains(p3)));
        };
        trianglesL.erase(std::remove_if(trianglesL.begin(), trianglesL.end(), pruner), trianglesL.end());
        m_indices.resize(trianglesL.size());

        for (int i = 0; i < trianglesL.size(); i++)
        {
            const auto& triangle = trianglesL[i];
            cv::Point2f p1(triangle[0], triangle[1]);
            cv::Point2f p2(triangle[2], triangle[3]);
            cv::Point2f p3(triangle[4], triangle[5]);

            int k1 = 0, k2 = 0, k3 = 0;
            for (k1 = 0; k1 < kMirrorMap.size(); k1++)
            {
                if (landmarks[kMirrorMap[k1][0]] == p1)
                    break;
            }
            for (k2 = 0; k2 < kMirrorMap.size(); k2++)
            {
                if (landmarks[kMirrorMap[k2][0]] == p2)
                    break;
            }
            for (k3 = 0; k3 < kMirrorMap.size(); k3++)
            {
                if (landmarks[kMirrorMap[k3][0]] == p3)
                    break;
            }

            m_indices[i] = cv::Vec3i(k1, k2, k3);
        }
    }

    trianglesR = mirrorTriangulation(landmarks, kMirrorMap);
    trianglesL.reserve(trianglesL.size() + trianglesR.size());
    std::copy(trianglesR.begin(), trianglesR.end(), std::back_inserter(trianglesL));

    return trianglesL;
}

// Input:
//  1) m_indices : for triangles on the left side
//  2) mirrorMap : mapping from vertices of triangles on left side to corresponding vertices on right
//  3) landmarks : the landmark vector
std::vector<cv::Vec6f> FaceMesh::mirrorTriangulation(const Landmarks& landmarks, const std::vector<std::array<int, 2>>& mirrorMap)
{
    std::vector<cv::Vec6f> b(m_indices.size());
    for (int i = 0; i < b.size(); i++)
    {
        cv::Point2f q1(landmarks[mirrorMap[m_indices[i][0]][1]]);
        cv::Point2f q2(landmarks[mirrorMap[m_indices[i][1]][1]]);
        cv::Point2f q3(landmarks[mirrorMap[m_indices[i][2]][1]]);
        b[i] = cv::Vec6f(q1.x, q1.y, q2.x, q2.y, q3.x, q3.y);
    }
    return b;
}

// Load in the left side triangles
int FaceMesh::readTriangulation(const std::string& filename)
{
    m_indices.clear();

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (fs.isOpened())
    {
        cv::FileNode n = fs["triangles"];
        if (n.type() != cv::FileNode::SEQ)
        {
            std::cerr << "triangles is not a sequence! FAIL" << std::endl;
            return 1;
        }

        cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
        for (; it != it_end; ++it)
        {
            cv::Vec3i triangle;
            (*it) >> triangle;
            m_indices.push_back(triangle);
        }
    }

    return 0;
}

int FaceMesh::writeTriangulation(const std::string& filename) const
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "triangles"
           << "[";
        for (int i = 0; i < m_indices.size(); i++)
        {
            fs << m_indices[i];
        }
        fs << "]";
    }

    return 0;
}

void FaceMesh::draw(cv::Mat& canvas, const Landmarks& landmarks, const Triangles& triangles)
{
    for (auto& t : triangles)
    {
        cv::Point2f p1(t[0], t[1]);
        cv::Point2f p2(t[2], t[3]);
        cv::Point2f p3(t[4], t[5]);
        cv::line(canvas, p1, p2, { 0, 255, 0 }, 1, 8);
        cv::line(canvas, p2, p3, { 0, 255, 0 }, 1, 8);
        cv::line(canvas, p3, p1, { 0, 255, 0 }, 1, 8);
    }

    for (const auto& p : landmarks)
    {
        cv::circle(canvas, p, 1, { 0, 255, 0 }, -1, 8);
    }
}

const std::vector<cv::Range> FaceMesh::kContours{
    { 36, 41 + 1 },
    { 42, 47 + 1 },
    { 27, 35 + 1 },
    { 48, 59 + 1 }
};

const std::vector<cv::Range> FaceMesh::kCurves{
    { 17, 21 + 1 },
    { 22, 26 + 1 }
};

const std::vector<std::array<int, 2>> FaceMesh::kMirrorMap{
    // http://ibug.doc.ic.ac.uk/resources/facial-point-annotations/

    // Contour
    { { 0, 16 } }, // 0
    { { 1, 15 } },
    { { 2, 14 } },
    { { 3, 13 } },
    { { 4, 12 } },
    { { 5, 11 } },
    { { 6, 10 } },
    { { 7, 9 } },
    { { 8, 8 } },

    // Eyebrow
    { { 17, 26 } }, // 9
    { { 18, 25 } },
    { { 19, 24 } }, // 11
    { { 20, 23 } },
    { { 21, 22 } }, // 13

    // Nose
    { { 27, 27 } }, // 14
    { { 28, 28 } },
    { { 29, 29 } },
    { { 30, 30 } },

    { { 31, 35 } },
    { { 32, 34 } },
    { { 33, 33 } },

    // Eye
    { { 39, 42 } },
    { { 38, 43 } },
    { { 37, 44 } },
    { { 36, 45 } },
    { { 40, 47 } },
    { { 41, 46 } },

    // Mouth
    { { 48, 54 } },
    { { 49, 53 } },
    { { 50, 52 } },
    { { 51, 51 } },

    { { 59, 55 } },
    { { 58, 56 } },
    { { 57, 57 } },

    { { 60, 64 } },
    { { 61, 63 } },
    { { 62, 62 } },

    { { 67, 65 } },
    { { 66, 66 } }
};

static cv::Mat getAffineLeastSquares(const cv::Vec6f& T1, const cv::Vec6f& T2)
{
    std::vector<cv::Point2f> pt1{ { T1[0], T1[1] }, { T1[2], T1[3] }, { T1[4], T1[5] } };
    std::vector<cv::Point2f> pt2{ { T2[0], T2[1] }, { T2[2], T2[3] }, { T2[4], T2[5] } };
    return cv::getAffineTransform(pt1, pt2);
}

DRISHTI_FACE_NAMESPACE_END
