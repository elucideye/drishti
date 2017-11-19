/*! -*-c++-*-
  @file   FaceDetector.h
  @author David Hirvonen
  @brief  Internal declaration of a face trained object detector.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceDetector_h__
#define __drishti_face_FaceDetector_h__

#include "drishti/face/drishti_face.h"
#include "drishti/face/FaceDetectorFactory.h"
#include "drishti/core/Shape.h"
#include "drishti/face/Face.h"
#include "drishti/face/FaceIO.h"

#include "acf/MatP.h"

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
#include <iterator>
#include <memory>
#include <iomanip>
#include <functional>

#define DO_CV_FACE 1

namespace dsdkc = drishti::core;

DRISHTI_FACE_NAMESPACE_BEGIN

#define EYE cv::Matx33f::eye()

class FaceDetector
{
public:
    struct PaddedImage
    {
        PaddedImage() {}
        PaddedImage(const cv::Mat& Ib, const cv::Rect& roi = {})
            : Ib(Ib)
            , roi(roi)
        {
        }
        PaddedImage& operator=(const PaddedImage& I)
        {
            Ib = I.Ib;
            roi = I.roi;
            return (*this);
        }
        operator cv::Mat()
        {
            return Ib;
        }
        cv::Mat Ib;
        cv::Rect roi;
    };

    typedef std::function<std::array<cv::Mat, 2>(const cv::Point2f& L, const cv::Point2f& R)> EyeCropper;
    typedef std::function<int(const cv::Mat&, const std::string& tag)> MatLoggerType;
    typedef std::function<void(double seconds)> TimeLoggerType;

    class Impl;
    typedef std::vector<cv::Point2f> Landmarks;

    FaceDetector(FaceDetectorFactory& Resources);
    ~FaceDetector();

    void setLandmarkFormat(FaceSpecification::Format format);

    virtual void operator()(const MatP& I, const PaddedImage& Ib, std::vector<FaceModel>& faces, const cv::Matx33f& H = EYE);
    virtual void setFaceDetectorMean(const FaceModel& mu);
    virtual const FaceModel& getFaceDetectorMean() const;
    virtual cv::Mat getUprightImage();

    // Legacy (doesn't use virtual detect):
    virtual void operator()(const MatP& I, const PaddedImage& Ib, std::vector<dsdkc::Shape>& shapes, const cv::Matx33f& H = EYE);

    // TODO: Add this for detector modification (cascThr, etc), but eventually make limited public API
    drishti::ml::ObjectDetector* getDetector();

    virtual std::vector<cv::Point2f> getFeatures() const;

    FaceModel getMeanShape(const cv::Size2f& size) const;
    FaceModel getMeanShape(const cv::Rect2f& roi) const;

    void setScaling(float scale);
    cv::Size getWindowSize() const;

    void setFaceStagesHint(int stages);
    void setFace2StagesHint(int stages);
    void setEyelidStagesHint(int stages);
    void setIrisStagesHint(int stages);

    void setDoIrisRefinement(bool flag);
    void setDoEyeRefinement(bool flag);
    void setInits(int inits);
    void setDoNMS(bool doNMS);
    void setDoNMSGlobal(bool flag);
    void setDetectionTimeLogger(TimeLoggerType logger);
    void setRegressionTimeLogger(TimeLoggerType logger);
    void setEyeRegressionTimeLogger(TimeLoggerType logger);
    void setLogger(MatLoggerType logger);
    void setHrd(const cv::Matx33f& Hrd); // regression face => detection face
    void setEyeCropper(EyeCropper& cropper);
    void paint(cv::Mat& frame);

    virtual void detect(const MatP& I, std::vector<FaceModel>& faces);
    virtual void refine(const PaddedImage& Ib, std::vector<FaceModel>& faces, const cv::Matx33f& H, bool isDetection);

protected:
    std::unique_ptr<Impl> m_impl;
};

void splitContour(const std::vector<cv::Point2f>& points, std::vector<std::vector<cv::Point2f>>& contours);

DRISHTI_FACE_NAMESPACE_END

#endif /* defined(__drishti_face_FaceDetector_h__) */
