// Copyright (c) 2016, David Hirvonen
// All rights reserved.

#ifndef FACELANDMARKER_H
#define FACELANDMARKER_H 1

#include <dest/dest.h>

// Use local convert definitions to avoid DEST_WITH_OPENCV which is failing on some platforms
#include "convert.h"
//#include <dest/util/convert.h>

#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class FaceLandmarker
{
public:

    FaceLandmarker();
    
    FaceLandmarker(const std::string &sRegressor, const std::string &sDetector);
    
    std::vector<cv::Point2f> fitLandmarks(const cv::Mat &gray);
    
    std::vector<cv::Point2f>& operator()(const cv::Mat1b &gray, const cv::Rect &roi);
    
    std::vector<cv::Point2f>& operator()(const cv::Mat1b &gray);
    
    void draw(cv::Mat &canvas, const cv::Point2f &tl = {});
    int writeTriangulation(const std::string &filename) const;
    int readTriangulation(const std::string &filename);
    
    const cv::Rect & getRoi() const { return m_roi; }

    // DEBUG:
    const cv::Mat1b &getLabels() const { return m_labels; }
    cv::Mat canvas;
    
protected:
    
    void delaunay(const cv::Size &size);
    void mirrorTriangulation(const std::vector<cv::Point2f> &landmarks, const std::vector<std::array<int,2>> &mirrorMap);
    
    
    cv::Mat1b m_labels;
    
    cv::Rect m_roi;
    
    std::vector<cv::Point2f> m_landmarks;
    
    std::vector<cv::Vec6f> m_triangles[2];
    
    std::vector<cv::Vec3i> m_indices;
    
    static const std::vector<cv::Range> kContours;
    static const std::vector<cv::Range> kCurves;
    static const std::vector<std::array<int,2>> kMirrorMap;
    
    cv::CascadeClassifier m_detector;
    dest::core::Tracker m_tracker;
};

#endif // FACE_LANDMARKER_H
