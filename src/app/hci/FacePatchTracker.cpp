/*! -*-c++-*-
  @file   FacePatchTracker.cpp
  @author David Hirvonen
  @brief  Face and eye tracking, optical flow, corner detection, etc.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FacePatchTracker.h"

#include <opencv2/highgui.hpp>

bool FacePatchTracker::empty() const
{
    return m_trackers.empty();
}
    
void FacePatchTracker::init(const drishti::face::FaceModel &face, const cv::Mat4b &image)
{
    std::array<cv::Rect2f, 2> eyes, eyesUpper, eyesLower;
    face.getEyeRegions(eyes[0], eyes[1], 0.33);
    
    cv::Point2f nose = (*face.noseTip);
    std::array<std::array<cv::Point2f, 2>, 2> corners
    {{
        {{ face.eyeFullR->getOuterCorner(), face.eyeFullR->getInnerCorner() }},
        {{ face.eyeFullL->getInnerCorner(), face.eyeFullL->getOuterCorner() }}
    }};
    
    // Create new line indexed templates (somewhat pose invariant):
    const cv::Point2f vro = corners[0][0] - nose;
    const cv::Point2f vri = corners[0][1] - nose;
    const cv::Point2f vli = corners[1][0] - nose;
    const cv::Point2f vlo = corners[1][1] - nose;
    
    m_trackers =
    {
        { "right-upper", makeRect(nose + vri * 1.666f, eyes[0].size()), PatchTracker::create() },
        { "right-lower", makeRect(nose + vro * 0.666f, eyes[0].size()), PatchTracker::create() },
        { "left-lower", makeRect(nose + vlo * 0.666f, eyes[1].size()), PatchTracker::create() },
        { "left-upper", makeRect(nose + vli * 1.666f, eyes[1].size()), PatchTracker::create() },
        { "dorsal-bridge", getDorsalBridgeRoi(face, 0.33f), PatchTracker::create() }
    };
    
    for(auto &t : m_trackers)
    {
        auto valid = t.roi & cv::Rect2d({0.0,0.0}, image.size());

        if(t.roi.area() != valid.area())
        {
            // To simplify edge cases (initially) we simply abort if
            // any face patch can't be propertly initialized
            return;
        }
        
        t.roi = valid;
    }
    
    // Skip if any patch isn't visible:
    cv::parallel_for_({0, static_cast<int>(m_trackers.size())}, [&](const cv::Range &r) {
        for(int i = r.start; i < r.end; i++) {
            m_trackers[i].tracker->init(image, m_trackers[i].roi);
        }
    });
}

void FacePatchTracker::update(const drishti::face::FaceModel &frace, const cv::Mat4b &image)
{
    // Update the trackers:
    cv::parallel_for_({0, static_cast<int>(m_trackers.size())}, [&](const cv::Range &r) {
        for(int i = r.start; i < r.end; i++) {
            m_trackers[i].tracker->update(image, m_trackers[i].roi);
        }
    });
}

void FacePatchTracker::fill(drishti::face::FaceModel &face)
{
    face.userFeatures.emplace_back("iris-center-right", face.eyeFullR->irisEllipse.center);
    face.userFeatures.emplace_back("iris-center-left", face.eyeFullL->irisEllipse.center);
    for(const auto &t : m_trackers)
    {
        face.userFeatures.emplace_back(t.name, (t.roi.tl() + t.roi.br()) * 0.5);
    }
}

std::array<cv::Point2f, 2> FacePatchTracker::getFrontalizedEyePair(const drishti::face::FaceModel &faceIn)
{
    // Get a mean reference point:
    auto face = faceIn;
    fill(face); // get gaze features
    
    int count = 0;
    cv::Point2f mu;
    for(const auto &f : face.userFeatures)
    {
        if(f.first.find("iris") == std::string::npos)
        {
            count ++;
            mu += f.second;
        }
    }
    mu *= (1.f / static_cast<float>(count));
    
    // Here we can consider applying a generic 3D face model to achieve a somewhat more pose invariant
    // gaze estimation -- even perspective model alignment with 4 coplanar points could be useful.
    const cv::Point2f irisR = faceIn.eyeFullR->irisEllipse.center;
    const cv::Point2f irisL = faceIn.eyeFullL->irisEllipse.center;
    const float iod = cv::norm(irisR - irisL);
    const std::array<cv::Point2f, 2> eyes {{ (irisR - mu) * (1.f / iod), (irisL - mu) * (1.f / iod) }};
    
    return eyes;
}

cv::Mat FacePatchTracker::draw(const drishti::face::FaceModel &faceIn, const cv::Mat4b &image)
{
    const auto eyes = getFrontalizedEyePair(faceIn);

    cv::Mat canvas = image.clone();
    {
        // Drawing part:
        cv::Point2f center(image.cols / 2, image.rows / 8);
        cv::line(canvas, center - cv::Point2f(100, 0), center + cv::Point2f(100, 0), {255,0,255}, 2, 8);
        cv::line(canvas, center - cv::Point2f(0, 100), center + cv::Point2f(0, 100), {255,0,255}, 2, 8);
        const float scale = 100.f;
        for(const auto &e : eyes)
        {
            cv::Point2f p = (e * scale) + center;
            cv::circle(canvas, p, 16, {0,255,255}, -1, 8);
        }
    }
    
    faceIn.draw(canvas, 4, true, false);
    
    for(const auto &t : m_trackers)
    {
        cv::Point2f tl(t.roi.tl()), br(t.roi.br()), tr(br.x, tl.y), bl(tl.x, br.y);
        
        // Draw patch:
        cv::rectangle(canvas, t.roi, {0,255,0}, 2, 8);
        
        // Draw cross hair:
        cv::line(canvas, tl, br, {0,255,0}, 2, 8);
        cv::line(canvas, tr, bl, {0,255,0}, 2, 8);
        
        // Draw line from patch to iris center:
        cv::line(canvas, (tl+br)*0.5, faceIn.eyeFullL->irisEllipse.center, {0,255,0}, 2, 8);
        cv::line(canvas, (tl+br)*0.5, faceIn.eyeFullR->irisEllipse.center, {0,255,0}, 2, 8);
        
        // Draw iris center:
        cv::circle(canvas, (tl + br) * 0.5f, std::max(6, static_cast<int>(cv::norm(tl-br)*0.025f)), {255,0,255}, -1, 8);
    }

    return canvas;
}

cv::Rect2d FacePatchTracker::getDorsalBridgeRoi(const drishti::face::FaceModel &face, float scale) const
{
    const cv::Point2f l = face.eyeFullL->irisEllipse.center;
    const cv::Point2f r = face.eyeFullR->irisEllipse.center;
    const float iod = cv::norm(l - r);
    const cv::Point2f center = (l + r) * 0.5f, diag(iod * scale * 0.5, iod * scale * 0.5f);
    return { center - diag, center + diag };
}

cv::Rect FacePatchTracker::makeRect(const cv::Point &center, const cv::Size &size)
{
    return { center - cv::Point(size.width/2, size.height/2), size };
}
