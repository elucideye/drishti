/*! -*-c++-*-
  @file   FaceDetector.cpp
  @author David Hirvonen
  @brief  Internal implementation of a face trained object detector.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/drishti_core.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/timing.h"
#include "drishti/core/Parallel.h"
#include "drishti/face/FaceDetector.h"
#include "drishti/face/FaceIO.h"
#include "drishti/geometry/Primitives.h"
#include "drishti/ml/ShapeEstimator.h"
#include "drishti/ml/ObjectDetector.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/face/Face.h"
#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/geometry/Rectangle.h"
#include "drishti/geometry/motion.h"

#include <acf/ACF.h> // ACF detection

#include <stdio.h>

#define DRISHTI_FACE_DETECTOR_DO_SIMILARITY_MOTION 1

DRISHTI_FACE_NAMESPACE_BEGIN

using drishti::geometry::operator*;

// Map from normalized coordinate system to input ROI
static cv::Matx33f denormalize(const cv::Rect& roi);
static void chooseBest(std::vector<cv::Rect>& objects, std::vector<double>& scores);

// ((((((((((((((( Impl )))))))))))))))
class FaceDetector::Impl
{
public:
    typedef FaceDetector::MatLoggerType MatLoggerType;
    typedef FaceDetector::TimeLoggerType TimeLoggerType;
    typedef FaceDetector::EyeCropper EyeCropper;

    Impl(FaceDetectorFactory& resources)
    {
        create(resources);
    }

    ~Impl() = default;

    void create(FaceDetectorFactory& resources)
    {
        m_detector = resources.getFaceDetector();
        m_regressor = resources.getFaceEstimator();
        m_eyeRegressor.resize(2);
        for (int i = 0; i < 2; i++)
        {
            m_eyeRegressor[i] = resources.getEyeEstimator();
        }
    }

    void setLandmarkFormat(FaceSpecification::Format format)
    {
        m_landmarkFormat = format;
    }

    /*
     * I    : input detectio image (typically planar format RGB)
     * Ib   : input image for regressor (graysacle uint8_t), often higer resolution
     * doMs : perform non max suppression on output detections
     * H    : homography expressing motion from detection image to regressor image
     * init : # of initializations to perform during regression
     */

    template <typename ImageType>
    void detect(const ImageType& I, std::vector<dsdkc::Shape>& shapes)
    {
        // clang-format off
        drishti::core::ScopeTimeLogger scopeTimeLogger = [this](double elapsed)
        {
            if (m_detectionTimeLogger)
            {
                m_detectionTimeLogger(elapsed);
            }
        };
        // clang-format on

        // Detect objects:
        std::vector<double> scores;
        std::vector<cv::Rect> objects;
        (*m_detector)(I, objects, &scores);

        if (m_doNMSGlobal)
        {
            chooseBest(objects, scores);
        }

        for (int i = 0; i < objects.size(); i++)
        {
            shapes.emplace_back(objects[i], scores[i]);
        }
    }

    void refineFace(const PaddedImage& Ib, std::vector<FaceModel>& faces, const cv::Matx33f& H, bool isDetection)
    {
        // Find the landmarks:
        if (m_regressor)
        {
            std::vector<dsdkc::Shape> shapes(faces.size());
            std::transform(faces.begin(), faces.end(), shapes.begin(), [](const FaceModel& face) {
                return dsdkc::Shape(face.roi);
            });
            findLandmarks(Ib, shapes, H, isDetection);
            shapesToFaces(shapes, faces);
        }

        if (m_eyeRegressor.size() && m_eyeRegressor[0] && m_eyeRegressor[1] && m_doEyeRefinement && faces.size())
        {
            for (auto& f : faces)
            {
                DRISHTI_EYE::EyeModel eyeR, eyeL;
                segmentEyes(Ib.Ib, f, eyeR, eyeL);
                if (eyeR.eyelids.size())
                {
                    f.eyeFullR = eyeR;
                    f.eyeRightCenter = core::centroid(eyeR.eyelids);
                }
                if (eyeL.eyelids.size())
                {
                    f.eyeFullL = eyeL;
                    f.eyeLeftCenter = core::centroid(eyeL.eyelids);
                }
            }
        }
    }

    using RectPair = std::array<cv::Rect, 2>;
    using MatPair = std::array<cv::Mat, 2>;
    static void extractCrops(const cv::Mat& Ib, const RectPair& eyes, const cv::Rect& bounds, MatPair& crops)
    {
        for (int i = 0; i < 2; i++)
        {
            cv::Rect roi = eyes[i] & bounds;
            if (roi == eyes[i])
            {
                crops[i] = Ib(roi); // shallow copy
            }
            else
            {
                // We use a crop preserving deep copy in rare case of clipping
                crops[i].create(eyes[i].size(), CV_8UC1);
                crops[i].setTo(0);
                Ib(roi).copyTo(crops[i](roi - eyes[i].tl()));
            }
        }
    }

    void segmentEyes(const cv::Mat1b& Ib, FaceModel& face, DRISHTI_EYE::EyeModel& eyeR, DRISHTI_EYE::EyeModel& eyeL)
    {
        cv::Rect2f roiR, roiL;
        bool hasEyes = face.getEyeRegions(roiR, roiL, 0.666);
        if (hasEyes && roiR.area() && roiL.area())
        {
            // clang-format off
            drishti::core::ScopeTimeLogger scopeTimeLogger = [this](double elapsed)
            {
                if (m_eyeRegressionTimeLogger)
                {
                    m_eyeRegressionTimeLogger(elapsed);
                }
            };
            // clang-format on

            MatPair crops;
            RectPair eyes = { { roiR, roiL } };
            extractCrops(Ib, eyes, { { 0, 0 }, Ib.size() }, crops);

            cv::Mat flipped;
            cv::flip(crops[1], flipped, 1); // Flip left eye to right eye cs
            crops[1] = flipped;

            cv::Point2f v = geometry::centroid<float, float>(roiR) - geometry::centroid<float, float>(roiL);
            float theta = std::atan2(v.y, v.x);
            eyeR.angle = theta;
            eyeL.angle = (-theta);

            std::array<DRISHTI_EYE::EyeModel*, 2> results{ { &eyeR, &eyeL } };
            for (int i = 0; i < 2; i++)
            {
                m_eyeRegressor[i]->setDoIndependentIrisAndPupil(m_doIrisRefinement);
                m_eyeRegressor[i]->setEyelidInits(1);
                m_eyeRegressor[i]->setIrisInits(1);
            }

            drishti::core::ParallelHomogeneousLambda harness = [&](int i) {
                (*m_eyeRegressor[i])(crops[i], *results[i]);
            };

            //harness({0, 2});
            cv::parallel_for_({ 0, 2 }, harness, 2);

            eyeL.flop(crops[1].cols);
            eyeL += eyes[1].tl(); // shift features to image coordinate system
            eyeR += eyes[0].tl();
            eyeR.roi = eyes[0];
            eyeL.roi = eyes[1];
        }
    }

    void findLandmarks(const PaddedImage& Ib, std::vector<dsdkc::Shape>& shapes, const cv::Matx33f& Hdr_, bool isDetection)
    {
        // Scope based eye segmentation timer:

        // clang-format off
        drishti::core::ScopeTimeLogger scopeTimeLogger = [this](double elapsed)
        {
            if (m_regressionTimeLogger)
            {
                m_regressionTimeLogger(elapsed);
            }
        };
        // clang-format on

        const cv::Mat gray = Ib.Ib;
        CV_Assert(gray.type() == CV_8UC1);

        const cv::Rect fullBounds({ 0, 0 }, Ib.Ib.size());
        const cv::Rect bounds = Ib.roi.area() ? Ib.roi : fullBounds;

        for (int i = 0; i < shapes.size(); i++)
        {
            // Detection rectangles may have a geometry (w.r.t. face features) that is incompatible with the
            // ROI geometry used for training the face landmark regressor.  In cases where we aim to refine
            // such raw detection rectangles, we must map them onto faces in the landmark regression image
            // with a geometry that is similar to that used during training.  We do this by using a known
            // approximate homography mapping our mean detection face features (eyes, nose, moiuth) to the
            // same face mean face features from the casecaded pose regressor training.  This transformation
            // must also be composed with the input Hdr_ homography that provides a transformation from the
            // detection image coordinate system to the landmark regression coordinate system, which is most
            // likely a scale and translation (detection typically happens at lower resolution).
            const cv::Rect roi = isDetection ? mapDetectionToRegressor(shapes[i].roi, m_Hrd, Hdr_) : (Hdr_ * shapes[i].roi);

            // Perform an addition (optional) scaling that can be tuned easily by the user as some detection
            // scales will perform better than the mean mapping used above (experimentally).
            shapes[i].roi = scaleRoi(roi, m_scaling);

            // Crop the image such that the ROI to pixel geometry is preserved.  For most cases this is
            // a simple shallow copy/view, but in cases where the border is clipped, then we will effectively
            // perform border padding to achieve this goal.  This make our prediction ROI closest to the ROI
            // used during training and ensures our cascaded pose regression has the best chance of success.
            cv::Mat crop = geometryPreservingCrop(shapes[i].roi, gray);

            std::vector<bool> mask;
            std::vector<cv::Point2f> points;
            (*m_regressor)(crop, points, mask);
            for (const auto& p : points)
            {
                const cv::Point q = p + cv::Point2f(shapes[i].roi.tl());
                shapes[i].contour.emplace_back(q.x, q.y, 0);
            }
        }
    }

    static cv::Rect scaleRoi(const cv::Rect& roi, float scale)
    {
        cv::Point2f tl(roi.tl()), br(roi.br()), center((tl + br) * 0.5f), diag(br - center);
        return cv::Rect(center - (diag * scale), center + (diag * scale));
    }

    // Return requested light weight copy if roi is contained in frame bounds, else perform
    // a deep copy that preseves the crop geometry via border padding.  This ensures that
    // landmark regression has the best chance of success.
    static cv::Mat geometryPreservingCrop(const cv::Rect& roi, const cv::Mat& gray)
    {
        const cv::Rect bounds({ 0, 0 }, gray.size());
        const cv::Rect clipped = roi & bounds;
        cv::Mat crop = gray(clipped);
        if (clipped.size() != roi.size())
        {
            cv::Mat padded(roi.size(), gray.type(), cv::Scalar::all(0));
            crop.copyTo(padded(clipped - roi.tl()));
            cv::swap(crop, padded);
        }
        return crop;
    }

    // Notes on motion and coordinate systems:
    //
    // FaceDetector::m_Hrd : map normalized regressor mean face to normalized detector mean face
    // denormalize(s.roi)  : denormalize points
    // Hdr                 : detector image to regressor
    static cv::Rect mapDetectionToRegressor(const cv::Rect& roi, const cv::Matx33f& Hrd, const cv::Matx33f& Hdr_)
    {
        // 1) map the unit square from regressor to the detector;
        // 2) denormalize coordinates in the detector image;
        // 3) transform from the detector image to the regressor image;
        cv::Matx33f H = Hdr_ * denormalize(roi) * Hrd;
        return H * cv::Rect_<float>(0, 0, 1, 1);
    }

    void mapDetectionsToRegressor(std::vector<dsdkc::Shape>& shapes, const cv::Matx33f& Hrd, const cv::Matx33f& Hdr_)
    {
        for (auto& s : shapes)
        {
            s.roi = mapDetectionToRegressor(s.roi, Hrd, Hdr_);
        }
    }

    void shapesToFaces(std::vector<dsdkc::Shape>& shapes, std::vector<FaceModel>& faces)
    {
        faces.clear();
        for (auto& s : shapes)
        {
            FaceModel f;
            faces.push_back(shapeToFace(s, m_landmarkFormat));
        }
    }

    FaceModel getMeanShape(const drishti::ml::ShapeEstimator& regressor, const cv::Rect2f& roi) const
    {
        auto mu = regressor.getMeanShape();
        drishti::core::Shape shape;
        for (auto& p : mu)
        {
            shape.contour.emplace_back(roi.x + p.x * roi.width, roi.y + p.y * roi.height, 0);
        }

        DRISHTI_FACE::FaceModel face = shapeToFace(shape, m_landmarkFormat);

        return face;
    }

    FaceModel getMeanShape(const cv::Size2f& size) const
    {
        return getMeanShape(*m_regressor, cv::Rect2f({ 0.f, 0.f }, size));
    }

    FaceModel getMeanShape(const cv::Rect2f& roi) const
    {
        return getMeanShape(*m_regressor, roi);
    }

    cv::Matx33f getAffineMotionFromRegressorToDetector(const ml::ShapeEstimator& regressor)
    {
        FaceModel faceRegressorMean = getMeanShape(regressor, { 0.f, 0.f, 1.f, 1.f });

#if DRISHTI_FACE_DETECTOR_DO_SIMILARITY_MOTION
        cv::Mat M = estimateMotionLeastSquares(faceRegressorMean, m_faceDetectorMean);
#else
        cv::Mat M = getAffineMotion(faceRegressorMean, m_faceDetectorMean);
#endif
        cv::Matx33f Hrd = cv::Matx33f::eye();
        for (int y = 0; y < 2; y++)
        {
            for (int x = 0; x < 3; x++)
            {
                Hrd(y, x) = M.at<double>(y, x);
            }
        }
        return Hrd;
    }

    void setFaceDetectorMean(const FaceModel& mu)
    {
        m_faceDetectorMean = mu;
        if (m_regressor)
        {
            m_Hrd = getAffineMotionFromRegressorToDetector(*m_regressor);
        }
    }

    const FaceModel& getFaceDetectorMean()
    {
        return m_faceDetectorMean;
    }

    void setScaling(float scale)
    {
        m_scaling = scale;
    }
    void setDoIrisRefinement(bool flag)
    {
        m_doIrisRefinement = flag;
    }
    void setDoEyeRefinement(bool flag)
    {
        m_doEyeRefinement = flag;
    }
    void setInits(int inits)
    {
        m_inits = inits;
    }
    void setDoNMS(bool doNMS)
    {
        m_detector->setDoNonMaximaSuppression(doNMS);
    }
    void setDoNMSGlobal(bool doNMS)
    {
        m_doNMSGlobal = doNMS;
    }
    void setLogger(MatLoggerType logger)
    {
        if (m_detector)
        {
            std::cerr << "TODO: must add logger method" << std::endl;
            //m_detector->setLogger(logger);
        }
    }
    void setDetectionTimeLogger(TimeLoggerType logger)
    {
        m_detectionTimeLogger = logger;
    }
    void setRegressionTimeLogger(TimeLoggerType logger)
    {
        m_regressionTimeLogger = logger;
    }
    void setEyeRegressionTimeLogger(TimeLoggerType logger)
    {
        m_eyeRegressionTimeLogger = logger;
    }
    void setEyeCropper(EyeCropper& cropper)
    {
        m_eyeCropper = cropper;
    }

    void setUprightImage(const cv::Mat& Ib)
    {
        m_Ib = Ib;
    }
    const cv::Mat& getUprightImage() const
    {
        return m_Ib;
    }

    void setFaceStagesHint(int stages)
    {
        if (m_regressor)
        {
            m_regressor->setStagesHint(stages);
        }
    }
    void setEyelidStagesHint(int stages)
    {
        for (auto& regressor : m_eyeRegressor)
        {
            regressor->setEyelidStagesHint(stages);
        }
    }
    void setIrisStagesHint(int stages)
    {
        for (auto& regressor : m_eyeRegressor)
        {
            regressor->setIrisStagesHint(stages);
        }
    }

    drishti::ml::ObjectDetector* getDetector()
    {
        return m_detector.get();
    }

protected:
    FaceSpecification::Format m_landmarkFormat = FaceSpecification::kibug68;

    cv::Mat m_Ib;
    bool m_doIrisRefinement = true;
    bool m_doEyeRefinement = true;
    bool m_doNMSGlobal = false;
    int m_inits = 1;
    float m_scaling = 1.0;

    FaceModel m_faceDetectorMean;
    cv::Matx33f m_Hrd = cv::Matx33f::eye();

    TimeLoggerType m_detectionTimeLogger;
    TimeLoggerType m_regressionTimeLogger;
    TimeLoggerType m_eyeRegressionTimeLogger;
    std::unique_ptr<drishti::ml::ObjectDetector> m_detector;
    std::unique_ptr<drishti::ml::ShapeEstimator> m_regressor;
    std::vector<std::unique_ptr<DRISHTI_EYE::EyeModelEstimator>> m_eyeRegressor;

    EyeCropper m_eyeCropper;
};

// ((((((((((((( API )))))))))))))

FaceDetector::FaceDetector(FaceDetectorFactory& resources)
    : m_impl(drishti::core::make_unique<Impl>(resources))
{
}

FaceDetector::~FaceDetector() = default;

std::vector<cv::Point2f> FaceDetector::getFeatures() const
{
    // noop
    std::vector<cv::Point2f> features;
    return features;
}

void FaceDetector::setDoIrisRefinement(bool flag)
{
    m_impl->setDoIrisRefinement(flag);
}
void FaceDetector::setDoEyeRefinement(bool flag)
{
    m_impl->setDoEyeRefinement(flag);
}
void FaceDetector::setInits(int inits)
{
    m_impl->setInits(inits);
}
void FaceDetector::setDoNMS(bool doNMS)
{
    m_impl->setDoNMS(doNMS);
}
void FaceDetector::setDoNMSGlobal(bool doNMS)
{
    m_impl->setDoNMSGlobal(doNMS);
}
void FaceDetector::setFaceDetectorMean(const FaceModel& mu)
{
    m_impl->setFaceDetectorMean(mu);
}
void FaceDetector::setLogger(MatLoggerType logger)
{
    m_impl->setLogger(logger);
}
drishti::ml::ObjectDetector* FaceDetector::getDetector()
{
    return m_impl->getDetector();
}

void FaceDetector::setLandmarkFormat(FaceSpecification::Format format)
{
    m_impl->setLandmarkFormat(format);
}

cv::Mat FaceDetector::getUprightImage()
{
    return m_impl->getUprightImage();
}

void FaceDetector::detect(const MatP& I, std::vector<FaceModel>& faces)
{
    // Run detections
    std::vector<dsdkc::Shape> shapes;
    m_impl->detect(I, shapes);

    faces.resize(shapes.size());
    for (int i = 0; i < faces.size(); i++)
    {
        faces[i].roi = shapes[i].roi;

        // project default detection points into the roi
        //FaceModel &mu = m_impl->getFaceDetectorMean();
    }

    std::transform(shapes.begin(), shapes.end(), faces.begin(), [](const dsdkc::Shape& shape) {
        return FaceModel(shape.roi);
    });
}

void FaceDetector::refine(const PaddedImage& Ib, std::vector<FaceModel>& faces, const cv::Matx33f& H, bool isDetection)
{
    if (faces.size() && m_impl)
    {
        m_impl->refineFace(Ib, faces, H, isDetection);
    }
}

// face.area() > 0 indicates detection
void FaceDetector::operator()(const MatP& I, const PaddedImage& Ib, std::vector<FaceModel>& faces, const cv::Matx33f& H)
{
    detect(I, faces);
    refine(Ib, faces, H, true);
}

// Legacy (doesn't use virtual detect):
void FaceDetector::operator()(const MatP& I, const PaddedImage& Ib, std::vector<dsdkc::Shape>& shapes, const cv::Matx33f& H)
{
}

cv::Size FaceDetector::getWindowSize() const
{
    return m_impl->getDetector()->getWindowSize();
}
void FaceDetector::setDetectionTimeLogger(TimeLoggerType logger)
{
    m_impl->setDetectionTimeLogger(logger);
}
void FaceDetector::setRegressionTimeLogger(TimeLoggerType logger)
{
    m_impl->setRegressionTimeLogger(logger);
}
void FaceDetector::setEyeRegressionTimeLogger(TimeLoggerType logger)
{
    m_impl->setEyeRegressionTimeLogger(logger);
}
void FaceDetector::setEyeCropper(EyeCropper& cropper)
{
    m_impl->setEyeCropper(cropper);
}
void FaceDetector::setScaling(float scale)
{
    m_impl->setScaling(scale);
}

FaceModel FaceDetector::getMeanShape(const cv::Size2f& size) const
{
    return m_impl->getMeanShape(size);
}
FaceModel FaceDetector::getMeanShape(const cv::Rect2f& roi) const
{
    return m_impl->getMeanShape(roi);
}

const FaceModel& FaceDetector::getFaceDetectorMean() const
{
    return m_impl->getFaceDetectorMean();
}

void FaceDetector::setFaceStagesHint(int stages)
{
    m_impl->setFaceStagesHint(stages);
}

void FaceDetector::setEyelidStagesHint(int stages)
{
    m_impl->setEyelidStagesHint(stages);
}

// utility

// Map from normalized coordinate system to input ROI
static cv::Matx33f denormalize(const cv::Rect& roi)
{
    const cv::Matx33f C1 = transformation::translate(-0.5f, -0.5f);
    const cv::Matx33f C2 = transformation::translate(transformation::center(roi));
    const cv::Matx33f S = cv::Matx33f::diag({ static_cast<float>(roi.width), static_cast<float>(roi.height), 1.f });
    return (C2 * S * C1);
}

static void chooseBest(std::vector<cv::Rect>& objects, std::vector<double>& scores)
{
    if (objects.size() > 1)
    {
        int best = 0;
        for (int i = 1; i < objects.size(); i++)
        {
            if (scores[i] > scores[best])
            {
                best = i;
            }
        }
        objects = { objects[best] };
        scores = { scores[best] };
    }
}

DRISHTI_FACE_NAMESPACE_END
