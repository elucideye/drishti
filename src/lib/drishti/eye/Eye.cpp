/*! -*-c++-*-
  @file   Eye.cpp
  @author David Hirvonen
  @brief  Internal utility eye model class declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of the internal SDK eye model,
  which contains extended functionality compared to the public SDK eye model.

*/

#include "drishti/eye/Eye.h"
#include "drishti/core/drishti_stdlib_string.h" // FIRST
#include "drishti/core/Shape.h"
#include "drishti/geometry/Ellipse.h"

// clang-format off
#if !DRISHTI_BUILD_MIN_SIZE
#  include "drishti/geometry/EllipseSerializer.h"
#endif
// clang-format on

#include "drishti/geometry/Primitives.h"
#include "drishti/geometry/motion.h"

// clang-format off
#if !DRISHTI_BUILD_MIN_SIZE
#  include <opencv2/highgui.hpp>
#endif
// clang-format on

#define DO_DRAW_INITIAL_SHAPE 0

using namespace drishti;
using namespace drishti::core;

DRISHTI_EYE_NAMESPACE_BEGIN

typedef std::vector<cv::Point2f> PointVec;
static std::vector<PointVec> ellipseToContours(const cv::RotatedRect& ellipse, const PointVec& eyelids = {});
static void fillPolly(cv::Mat& image, const PointVec& polly, const cv::Scalar& value);

EyeModel::EyeModel() = default;
EyeModel::~EyeModel() = default;

std::vector<cv::Point2f> EyeModel::getUpperEyelid() const
{
    return std::vector<cv::Point2f>(eyelids.begin(), eyelids.begin() + cornerIndices[1]);
}

std::vector<cv::Point2f> EyeModel::getLowerEyelid() const
{
    return std::vector<cv::Point2f>(eyelids.begin() + cornerIndices[1], eyelids.end());
}

float EyeModel::openness() const
{
    float openness = -1.f;
    if (eyelids.size())
    {
        // Moments
        cv::Moments mom = cv::moments(eyelids);
        cv::Matx22f C(mom.mu20 / mom.m00, mom.mu11 / mom.m00, mom.mu11 / mom.m00, mom.mu02 / mom.m00);
        cv::Matx22f vecs;
        cv::Vec2f vals;
        cv::eigen(C, vals, vecs);

        openness = vals[1] / float(vals[0] + 1e-6f);
    }
    return openness;
}

void EyeModel::clear()
{
    eyelids.clear();
    crease.clear();
    pupil = {};
    iris = {};
    irisEllipse = {};
    pupilEllipse = {};
}

void EyeModel::upsample(int eyelidFactor, int creaseFactor)
{
    roi = cv::boundingRect(eyelids);
    if (eyelids.size() > 2)
    {
        std::vector<cv::Point2f> spline;
        core::upsample(eyelids, spline, eyelidFactor, true);
        std::swap(eyelids, spline);
        cornerIndices[0] *= eyelidFactor;
        cornerIndices[1] *= eyelidFactor;
    }

    if (crease.size() > 2)
    {
        std::vector<cv::Point2f> spline;
        core::upsample(crease, spline, creaseFactor, false);
        std::swap(crease, spline);
    }
}

// Check for numerical stability (errors at very low resolution, etc)
static bool isValid(const PointVec& contour, const PointVec& spline, float tolerance)
{
    PointVec contourHull;
    cv::convexHull(contour, contourHull);
    const cv::Rect roi = cv::boundingRect(contourHull);
    const cv::Point2f tl = roi.tl(), br = roi.br(), center = (tl + br) * 0.5f, diag = br - center;
    const cv::Rect2f zone(center - diag * tolerance, center + diag * tolerance);
    for (const auto& p : spline)
    {
        if (!zone.contains(p))
        {
            return false;
        }
    }
    return true;
}

void EyeModel::refine(int eyelidPoints, int creasePoints)
{
    roi = cv::boundingRect(eyelids);
    if (eyelids.size() > 2)
    {
        eyelidsSpline.clear();
        fitSpline(eyelids, eyelidsSpline, eyelidPoints);
        if (!isValid(eyelids, eyelidsSpline, 1.25))
        {
            eyelidsSpline = eyelids;
        }
        cornerIndices[0] = 0;
        cornerIndices[1] = int(eyelids.size()) / 2;
    }

    if (crease.size() > 2)
    {
        creaseSpline.clear();
        fitSpline(crease, creaseSpline, creasePoints, false);
        if (!isValid(crease, creaseSpline, 1.25f))
        {
            crease = creaseSpline;
        }
    }

    cv::Point2f irisCenter_, irisInner_, irisOuter_;
    estimateIrisLandmarks(irisCenter_, irisInner_, irisOuter_);
    irisCenter = irisCenter_;
    irisInner = irisInner_;
    irisOuter = irisOuter_;
}

void EyeModel::normalizeEllipse(cv::RotatedRect& ellipse)
{
    // Find closest angle to
    float theta = ellipse.angle * M_PI / 180.0;
    cv::Vec2f v(std::cos(theta), std::sin(theta));

    // First make sure it is oriented veritically:
    if (std::abs(v[1]) < std::abs(v[0]))
    {
        v = v[0] < 0.f ? cv::Vec2f(v[1], -v[0]) : cv::Vec2f(-v[1], v[0]);
        std::swap(ellipse.size.width, ellipse.size.height);
    }

    ellipse.angle = std::atan2(v[1], v[0]) * 180.0 / M_PI;
    if (ellipse.angle < 0.0f)
    {
        ellipse.angle += 360.0f;
    }

    CV_Assert(ellipse.angle >= 0.f);
}
void EyeModel::normalize()
{
    if (irisEllipse.size.area())
    {
        normalizeEllipse(irisEllipse);
    }
    if (pupilEllipse.size.area())
    {
        normalizeEllipse(pupilEllipse);
    }
}

cv::Point2f EyeModel::estimateGaze(bool isRight) const
{
    // Get rotation that maps eye corners to axis aligned:
    int right(isRight);
    std::array<cv::Point2f, 2> corners{ { eyelids[cornerIndices[1 - right]], eyelids[cornerIndices[right]] } };
    cv::Matx33f H = transformation::estimateSimilarity(corners, { { { -1.0f, 0.0f }, { +1.0f, 0.0f } } });
    // NOTE: {{{25.0,50.0}, {75.0,50.0}}});
    cv::Point3f q = H * cv::Point3f(irisEllipse.center.x, irisEllipse.center.y, 1.f);
    return cv::Point2f(q.x / q.z, q.y / q.z);
}

void EyeModel::estimateIrisLandmarks(cv::Point2f& irisCenter, cv::Point2f& irisInner, cv::Point2f& irisOuter) const
{
    if (irisEllipse.size.width)
    {
        std::vector<PointVec> points = ellipseToContours(irisEllipse);
        irisCenter = irisEllipse.center;

        std::pair<float, int> innerBest(std::numeric_limits<float>::max(), -1);
        std::pair<float, int> outerBest(std::numeric_limits<float>::max(), -1);
        for (int i = 0; i < points[0].size(); i++)
        {
            const auto& p = points[0][i];
            float innerD = cv::norm(getInnerCorner() - p);
            float outerD = cv::norm(getOuterCorner() - p);
            if (innerD < innerBest.first)
            {
                innerBest = std::make_pair(innerD, i);
            }
            if (outerD < outerBest.first)
            {
                outerBest = std::make_pair(outerD, i);
            }
        }

        irisInner = points[0][innerBest.second];
        irisOuter = points[0][outerBest.second];
    }
}

cv::Mat EyeModel::irisMask(const cv::Size& size, bool removeEyelids) const
{
    cv::Mat1b mask(size, 0);
    try
    {
        cv::ellipse(mask, irisEllipse, 255, -1, 4);
    }
    catch (...)
    {
    }
    if (removeEyelids)
    {
        mask &= this->mask(size, false);
    }
    return mask;
}

cv::Mat EyeModel::labels(const cv::Size& size) const
{
    if (size.area() == 0)
    {
        return cv::Mat1b();
    }

    const auto& curve = eyelidsSpline.size() ? eyelidsSpline : eyelids;

    // Eye mask:
    cv::Mat1b eyeMask = cv::Mat1b::zeros(size);
    fillPolly(eyeMask, curve, 255);

    // Iris mask:
    cv::Mat1b irisMask = cv::Mat1b::zeros(size);
    cv::ellipse(irisMask, irisEllipse, 255, -1);

    eyeMask.setTo(127, (eyeMask & irisMask));

    return eyeMask;
}

cv::Mat EyeModel::mask(const cv::Size& size, bool sclera, float irisScale) const
{
    cv::Mat1b mask;

    if (size.area() == 0)
    {
        return mask;
    }

    const auto& curve = eyelidsSpline.size() ? eyelidsSpline : eyelids;
    if (curve.size())
    {
        mask = cv::Mat1b::zeros(size);
        fillPolly(mask, curve, 255);

        if (sclera && irisEllipse.size.width)
        {
            cv::RotatedRect ellipse = irisEllipse;
            ellipse.size = ellipse.size * irisScale;
            try
            {
                cv::ellipse(mask, ellipse, 0, -1);
            }
            catch (...)
            {
            }
        }
    }

    return mask;
}

std::vector<std::vector<cv::Point2f>> EyeModel::getContours(bool doPupil) const
{
    std::vector<std::vector<cv::Point2f>> contours;

    contours.push_back(eyelidsSpline);
    contours.back().push_back(eyelids.front()); // closed contour

    if (irisEllipse.size.width)
    {
        auto segments = ellipseToContours(irisEllipse, eyelidsSpline);
        for (auto& s : segments)
        {
            contours.push_back(s);
        }
    }

    if (pupilEllipse.size.width && doPupil)
    {
        auto segments = ellipseToContours(pupilEllipse, eyelidsSpline);
        for (auto& s : segments)
        {
            contours.push_back(s);
        }
    }

    if (creaseSpline.size())
    {
        contours.push_back(creaseSpline);
    }

    if (irisCenter.has && doPupil)
    {
        std::vector<cv::Point2f> contour{ irisInner, irisCenter, irisOuter };
        contours.push_back(contour);

        // Add a cross hair:
        cv::Point2f v = (*irisInner) - (*irisOuter);
        cv::Point2f n(v.y, -v.x);
        float diameter = cv::norm(v);
        n *= (1.0 / diameter);
        contours.push_back({ (*irisCenter) - (n * diameter * 0.25f), (*irisCenter) + (n * diameter * 0.25f) });
    }

    return contours;
}

void EyeModel::flop(int width)
{
    if (angle.has)
    {
        angle = (M_PI - angle.value);
    }

    if (roi.has)
    {
        roi = cv::Rect(int(width) - (roi->x + roi->width), roi->y, roi->width, roi->height);
    }
    if (pupil[2])
    {
        flop(pupil[0], width);
    }
    if (iris[2])
    {
        flop(iris[0], width);
    }
    if (irisEllipse.size.width)
    {
        flop(irisEllipse.center.x, width);
        irisEllipse.angle = (180 - irisEllipse.angle);
    }
    if (pupilEllipse.size.width)
    {
        flop(pupilEllipse.center.x, width);
        pupilEllipse.angle = (180 - pupilEllipse.angle);
    }
    for (auto& p : eyelids)
    {
        flop(p.x, width);
    }
    for (auto& p : eyelidsSpline)
    {
        flop(p.x, width);
    }
    for (auto& p : crease)
    {
        flop(p.x, width);
    }
    for (auto& p : creaseSpline)
    {
        flop(p.x, width);
    }

    if (irisCenter.has)
    {
        flop(irisCenter->x, width);
        flop(irisInner->x, width);
        flop(irisOuter->x, width);
        std::swap(irisInner, irisOuter);
    }

    innerCorner = getInnerCorner();
    outerCorner = getOuterCorner();
}

void EyeModel::draw(cv::Mat& canvas, int level, bool doMask, const cv::Scalar& color, int width) const
{
    CV_Assert(canvas.type() == CV_8UC3 || canvas.type() == CV_8UC4);
    if (eyelids.size() < 3)
    {
        return;
    }

    {
        // Draw the ellipse contours:
        auto contours = getContours();
        for (int i = 0; i < contours.size(); i++)
        {
            std::vector<std::vector<cv::Point>> contour(1);
            std::copy(contours[i].begin(), contours[i].end(), std::back_inserter(contour[0]));
            cv::polylines(canvas, contour, false, color, width);
        }
    }

    // Draw the bounding box:
    if (roi.has)
    {
        cv::rectangle(canvas, roi, color, 1, 8);
    }

    std::vector<std::vector<cv::Point>> contours(2);
    if (eyelidsSpline.size())
    {
        std::copy(eyelidsSpline.begin(), eyelidsSpline.end(), std::back_inserter(contours[0]));
        contours[0].push_back(contours[0].front()); // closed contour
        cv::polylines(canvas, contours, true, color, width, 8);
    }

    if (creaseSpline.size())
    {
        std::copy(creaseSpline.begin(), creaseSpline.end(), std::back_inserter(contours[1]));
        cv::polylines(canvas, contours, false, color, width, 8);
    }

    if (level > 1) // model specific points, etc
    {
        if (iris[2] > 0.f)
        {
            cv::circle(canvas, cv::Point(iris[0], iris[1]), iris[2], { 0, 255, 0 }, 1, 8);
        }
        for (int i = 0; i < eyelids.size(); i++)
        {
            cv::circle(canvas, eyelids[i], 4, { 0, 255, 0 }, 1, 8);
        }
        for (const auto& i : cornerIndices)
        {
            cv::circle(canvas, eyelids[i], 8, { 0, 255, 255 }, 1, 8);
        }
    }

#if DO_DRAW_INITIAL_SHAPE
    // Draw the initial shape for visualization:
    const cv::Point2f* pI = reinterpret_cast<const cv::Point2f*>(&initial_shape_ref(0, 0));
    for (int i = 0; i < n; i++)
        cv::circle(canvas, cv::Point2f(pI[i].x * image.cols, pI[i].y * image.rows), 1, { 255, 0, 255 }, CV_AA);
#endif
}

void EyeModel::read(const std::string& filename)
{
#if !DRISHTI_BUILD_MIN_SIZE
    cv::FileStorage storage(filename, cv::FileStorage::READ);
    if (storage.isOpened())
    {
        auto node = storage["eye"];
        if (!node.empty())
        {
            read(node);
            refine();
        }
    }
#else
    CV_Assert(false);
#endif
};

void EyeModel::write(const std::string& filename) const
{
#if !DRISHTI_BUILD_MIN_SIZE
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);
    if (storage.isOpened())
    {
        storage << "eye" << (*this);
    }
#else
    CV_Assert(false);
#endif
};

void EyeModel::read(const cv::FileNode& node)
{
#if !DRISHTI_BUILD_MIN_SIZE
    {
        // Parse eyelids
        auto n = node["eyelids"];
        if (!n.empty())
        {
            drishti::core::Shape contour;
            n >> contour;
            eyelids = contour.getPoints();
        }
    }

    {
        // Parse crease
        auto n = node["crease"];
        if (!n.empty())
        {
            n >> crease;
        }
    }

    {
        auto n = node["iris"];
        if (!n.empty())
        {
            n >> iris;
        }
    }

    {
        auto n = node["pupil"];
        if (!n.empty())
        {
            n >> pupil;
        }
    }

    {
        // Parse optional irisEllipse
        auto n = node["irisEllipse"];
        if (!n.empty())
        {
            drishti::geometry::EllipseSerializer ellipse;
            ellipse.read(n);
            irisEllipse = ellipse;
        }
    }

    {
        // Parse optional pupilEllipse
        auto n = node["pupilEllipse"];
        if (!n.empty())
        {
            drishti::geometry::EllipseSerializer ellipse;
            ellipse.read(n);
            pupilEllipse = ellipse;
        }
    }

#else
    CV_Assert(false);
#endif
};

void DRISHTI_EYE::EyeModel::write(cv::FileStorage& fs) const
{
#if !DRISHTI_BUILD_MIN_SIZE
    fs << "{";

    if (eyelids.size())
    {
        fs << "eyelids" << drishti::core::Shape(roi, eyelids);
    }

    if (crease.size())
    {
        fs << "crease" << crease;
    }

    if (iris.dot(iris) > 0.0)
    {
        fs << "iris" << iris;
    }

    if (pupil.dot(pupil) > 0.0)
    {
        fs << "pupil" << pupil;
    }

    if (irisEllipse.size.area() > 0.0)
    {
        fs << "irisEllipse" << drishti::geometry::EllipseSerializer(irisEllipse);
    }

    if (pupilEllipse.size.area() > 0.0)
    {
        fs << "pupilEllipse" << drishti::geometry::EllipseSerializer(pupilEllipse);
    }

    fs << "}";
#else
    CV_Assert(false);
#endif
}

void write(cv::FileStorage& fs, const std::string&, const DRISHTI_EYE::EyeModel& x)
{
#if !DRISHTI_BUILD_MIN_SIZE
    x.write(fs);
#else
    CV_Assert(false);
#endif
}

void read(const cv::FileNode& node, DRISHTI_EYE::EyeModel& x, const DRISHTI_EYE::EyeModel& default_value)
{
#if !DRISHTI_BUILD_MIN_SIZE
    if (node.empty())
    {
        x = default_value;
    }
    else
    {
        x.read(node);
    }
#else
    CV_Assert(false);
#endif
}

// ==========

static void fillPolly(cv::Mat& image, const PointVec& polly, const cv::Scalar& value)
{
    std::vector<std::vector<cv::Point>> contours(1);
    contours[0].resize(polly.size());
    std::copy(polly.begin(), polly.end(), contours[0].begin());
    cv::fillPoly(image, contours, 255, 4);
}

// http://stackoverflow.com/a/23214219
static int mod(int k, int n)
{
    return ((k %= n) < 0) ? k + n : k;
}

static std::vector<PointVec> ellipseToContours(const cv::RotatedRect& ellipse, const PointVec& eyelids)
{
    std::vector<std::vector<cv::Point2f>> contours;

    // Convert ellipse to contour:
    std::vector<cv::Point2f> contour;
    drishti::geometry::ellipse2Poly(ellipse, 1.f, contour);

    bool doCopy = true;
    if (eyelids.size())
    {
        int validCount = 0;
        std::vector<uint8_t> mask(contour.size(), 0);
        for (int i = 0; i < contour.size(); i++)
        {
            mask[i] = uint8_t(cv::pointPolygonTest(eyelids, contour[i], false) > 0);
            validCount += int(mask[i]);
        }

        if (validCount > 0 && validCount < contour.size())
        {
            // Search for zero crossing
            doCopy = false;
            int n = int(mask.size());
            for (int i = 0; (i < mask.size()) && validCount; i++)
            {
                int j = mod(i - 1, n);
                // Found zero crossing, read the segment:
                if (mask[i] && !mask[j])
                {
                    std::vector<cv::Point2f> segment;
                    for (int k = i; (mask[k] && validCount); k = mod(k + 1, n), i = k)
                    {
                        validCount--;
                        segment.push_back(contour[k]);
                        mask[k] = 0;
                    }
                    if (segment.size() >= 2)
                    {
                        contours.push_back(segment);
                    }
                }
            }
        }
    }

    //std::cout << "CONTOURS:" << contours.size() << std::endl;

    if (doCopy)
    {
        contours.resize(1);
        std::copy(contour.begin(), contour.end(), std::back_inserter(contours[0]));
    }

    return contours;
}

DRISHTI_EYE_NAMESPACE_END
