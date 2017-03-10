// Copyright (c) 2016, David Hirvonen
// All rights reserved.

#include "FaceLandmarker.h"

#include <iostream>
#include <array>

FaceLandmarker::FaceLandmarker()
{
    
}

FaceLandmarker::FaceLandmarker(const std::string &sRegressor, const std::string &sDetector)
{
    if(!sDetector.empty())
    {
        m_detector.load(sDetector);
    }

    assert(!sRegressor.empty());
    m_tracker.load(sRegressor);
}
    
std::vector<cv::Point2f> FaceLandmarker::fitLandmarks(const cv::Mat &gray)
{
    dest::core::Rect r, ur = dest::core::unitRectangle();
    dest::util::toDest(cv::Rect({0,0}, gray.size()), r);
    dest::core::ShapeTransform shapeToImage;
    shapeToImage = dest::core::estimateSimilarityTransform(ur, r);
    dest::core::MappedImage mappedGray = dest::util::toDestHeaderOnly(gray);
    dest::core::Shape s = m_tracker.predict(mappedGray, shapeToImage);
     
    std::vector<cv::Point2f> landmarks(s.cols());
    for(int i = 0; i < s.cols(); i++)
    {
        landmarks[i] = { s(0,i), s(1,i) };
    }

    return landmarks;
}
    
std::vector<cv::Point2f>& FaceLandmarker::operator()(const cv::Mat1b &gray, const cv::Rect &roi)
{
    m_landmarks.clear();
    
    cv::Rect crop = roi;
    if(!m_detector.empty())
    {
        std::vector<cv::Rect> faces;
        cv::Size mini(gray.cols*1/3, gray.cols*1/3);
        cv::Size maxi(gray.cols*3/4, gray.cols*3/4);
        m_detector.detectMultiScale(gray, faces, 1.1, 1, 0, mini, maxi);
        
        if(faces.size() == 0)
        {
            return m_landmarks;
        }

        std::sort(faces.begin(), faces.end(), [](const cv::Rect &a, const cv::Rect &b) { return a.area() < b.area(); });
        crop = faces.front();
    }
    m_roi = crop;
    
    // =========
    auto & landmarks = (*this)(gray(crop));
    for(auto &p : landmarks)
    {
        p += cv::Point2f(crop.x, crop.y);
    }
    
    delaunay(gray.size());
    
    return landmarks;
}
    
std::vector<cv::Point2f>& FaceLandmarker::operator()(const cv::Mat1b &gray)
{
    m_landmarks = fitLandmarks(gray);
    return m_landmarks;
}

cv::Point2f clip(const cv::Point2f &p, const cv::Rect &roi)
{
    cv::Point2f q
    (
     std::max(std::min(float(roi.br().x-1), p.x), float(roi.tl().x)),
     std::max(std::min(float(roi.br().y-1), p.y), float(roi.tl().y))
    );
    return q;
}

void FaceLandmarker::delaunay(const cv::Size &size)
{
    auto landmarks = m_landmarks;
    auto mirrorMap = kMirrorMap;
    
    { // Synthesize some new points, making sure to update the mirror map:
        cv::Point2f p27 = m_landmarks[27];
        cv::Point2f p30 = m_landmarks[30];
        cv::Point2f v = p27 - p30;
        cv::Point2f vn = cv::normalize(cv::Vec2f(v));

        const float scale = 1.0;

        //cv::Mat drawing = canvas.clone();
        
        for(int i = 9; i < 13; i++)
        {
            std::array<int, 2> indices;
            for(int j = 0; j < 2; j++)
            {
                const cv::Point2f fh0 = m_landmarks[mirrorMap[i][j]];
                const cv::Point2f v0 = cv::normalize(cv::Vec2f(fh0 - p30));
                const float w0 = (1.5 + std::abs(v0.dot(vn))) * cv::norm(v) * scale;
                const int index0 = landmarks.size();
                indices[j] = index0;
                cv::Point2f l0 = clip(p30 + (v0 * w0), cv::Rect({0,0}, size));
                landmarks.push_back(l0);

                //cv::circle(drawing, fh0, 2, {255,255,0}, -1, 8);
                //cv::circle(drawing, l0, 2, {0,255,0}, -1, 8);
                //cv::imshow("c", drawing); cv::waitKey(0);
            }
            mirrorMap.push_back(indices);
        }
        
        { // Add the center ray:
            cv::Point2f c = (m_landmarks[mirrorMap[11][0]] + m_landmarks[mirrorMap[11][1]]) * 0.5f;
            const cv::Point2f v0 = cv::normalize(cv::Vec2f(c - p30));
            const float w0 = (1.5 + std::abs(v0.dot(vn))) * cv::norm(v) * scale;
            cv::Point2f l = clip(p30 + (v0 * w0), cv::Rect({0,0}, size));
            
            const int index = landmarks.size();
            landmarks.push_back(l);
            mirrorMap.push_back( {{ index, index }} );
        }

    }
    
    if(m_indices.size())
    {
        // Create triangles from indices
        m_triangles[0].resize(m_indices.size());
        m_triangles[1].resize(m_indices.size());
        for(int i = 0; i < m_indices.size(); i++)
        {
            cv::Point2f p1 = landmarks[ m_indices[i][0] ];
            cv::Point2f p2 = landmarks[ m_indices[i][1] ];
            cv::Point2f p3 = landmarks[ m_indices[i][2] ];
            m_triangles[0][i] = cv::Vec6f(p1.x,p1.y,p2.x,p2.y,p3.x,p3.y);
        }
        mirrorTriangulation(landmarks, mirrorMap);
    }
    else
    {
        // Perform the delaunay triangulation
        
        // Use a delaunay subdivision and balance mirror triangles:
        cv::Rect roi({0,0}, size);
        cv::Subdiv2D subdiv;
        subdiv.initDelaunay(roi);
        for(int i = 0; i < mirrorMap.size(); i++)
        {
            auto p = landmarks[ mirrorMap[i][0] ];
            subdiv.insert(p);
        }
        std::vector<cv::Vec3i> indices;
        subdiv.getTriangleList(m_triangles[0]);
        
        auto pruner = [&](const cv::Vec6f &triangle)
        {
            cv::Point2f p1(triangle[0], triangle[1]);
            cv::Point2f p2(triangle[2], triangle[3]);
            cv::Point2f p3(triangle[4], triangle[5]);
            return (! (roi.contains(p1) && roi.contains(p2) && roi.contains(p3)) );
        };
        m_triangles[0].erase(std::remove_if(m_triangles[0].begin(), m_triangles[0].end(), pruner), m_triangles[0].end());
        m_indices.resize(m_triangles[0].size());
        m_triangles[1].resize(m_triangles[0].size());
        
        for(int i = 0; i < m_triangles[0].size(); i++)
        {
            const auto &triangle = m_triangles[0][i];
            cv::Point2f p1(triangle[0], triangle[1]);
            cv::Point2f p2(triangle[2], triangle[3]);
            cv::Point2f p3(triangle[4], triangle[5]);
            
            int k1 = 0, k2 = 0, k3 = 0;
            for(k1 = 0; k1 < mirrorMap.size(); k1++) { if(landmarks[mirrorMap[k1][0]] == p1) break; }
            for(k2 = 0; k2 < mirrorMap.size(); k2++) { if(landmarks[mirrorMap[k2][0]] == p2) break; }
            for(k3 = 0; k3 < mirrorMap.size(); k3++) { if(landmarks[mirrorMap[k3][0]] == p3) break; }
            
            m_indices[i] = cv::Vec3i(k1,k2,k3);
        }
        
        mirrorTriangulation(landmarks, mirrorMap);
    }
}

// Input:
//  1) m_indices : for triangles on the left side
//  2) mirrorMap : mapping from vertices of triangles on left side to corresponding vertices on right
//  3) landmarks : the landmark vector
void FaceLandmarker::mirrorTriangulation(const std::vector<cv::Point2f> &landmarks, const std::vector<std::array<int,2>> &mirrorMap)
{
    for(int i = 0; i < m_triangles[0].size(); i++)
    {
        cv::Point2f q1( landmarks[mirrorMap[m_indices[i][0]][1]] );
        cv::Point2f q2( landmarks[mirrorMap[m_indices[i][1]][1]] );
        cv::Point2f q3( landmarks[mirrorMap[m_indices[i][2]][1]] );
        m_triangles[1][i] = cv::Vec6f(q1.x,q1.y,q2.x,q2.y,q3.x,q3.y);
    }
}

// Load in the left side triangles
int FaceLandmarker::readTriangulation(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(fs.isOpened())
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

int FaceLandmarker::writeTriangulation(const std::string &filename) const
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(fs.isOpened())
    {
        fs << "triangles" << "[";
        for(int i = 0; i < m_indices.size(); i++)
        {
            fs << m_indices[i];
        }
        fs << "]";
    }
    
    return 0;
}

void FaceLandmarker::draw(cv::Mat &canvas, const cv::Point2f &tl)
{
    for(int i = 0; i < 2; i++)
    {
        for(auto &t : m_triangles[i])
        {
            cv::Point2f p1(t[0], t[1]);
            cv::Point2f p2(t[2], t[3]);
            cv::Point2f p3(t[4], t[5]);
            cv::line(canvas, p1, p2, {0,255,0}, 1, 8);
            cv::line(canvas, p2, p3, {0,255,0}, 1, 8);
            cv::line(canvas, p3, p1, {0,255,0}, 1, 8);
        }
    }
    
    for(const auto &p : m_landmarks)
    {
        cv::circle(canvas, p, 1, {0,255,0}, -1, 8);
    }
}

const std::vector<cv::Range> FaceLandmarker::kContours
{
    {36, 41+1},
    {42, 47+1},
    {27, 35+1},
    {48, 59+1}
};

const std::vector<cv::Range> FaceLandmarker::kCurves
{
    {17, 21+1},
    {22, 26+1}
};

const std::vector<std::array<int,2>> FaceLandmarker::kMirrorMap
{
    // http://ibug.doc.ic.ac.uk/resources/facial-point-annotations/
    
    // Contour
    {{0,16}}, // 0
    {{1,15}},
    {{2,14}},
    {{3,13}},
    {{4,12}},
    {{5,11}},
    {{6,10}},
    {{7,9}},
    {{8,8}},
    
    // Eyebrow
    {{17,26}}, // 9
    {{18,25}},
    {{19,24}}, // 11
    {{20,23}},
    {{21,22}}, // 13
    
    // Nose
    {{27,27}}, // 14
    {{28,28}},
    {{29,29}},
    {{30,30}},
    
    {{31,35}},
    {{32,34}},
    {{33,33}},
    
    // Eye
    {{39,42}},
    {{38,43}},
    {{37,44}},
    {{36,45}},
    {{40,47}},
    {{41,46}},
    
    // Mouth
    {{48,54}},
    {{49,53}},
    {{50,52}},
    {{51,51}},
    
    {{59,55}},
    {{58,56}},
    {{57,57}},
    
    {{60,64}},
    {{61,63}},
    {{62,62}},
    
    {{67,65}},
    {{66,66}}
};
