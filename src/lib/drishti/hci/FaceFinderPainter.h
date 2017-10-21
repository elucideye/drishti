/*! -*-c++-*-
  @file   drishti/hci/FaceFinderPainter.h
  @author David Hirvonen
  @brief  Face tracking class with OpenGL texture drawing

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_FaceFinderPainter_h__
#define __drishti_hci_FaceFinderPainter_h__

#include "drishti/hci/drishti_hci.h"
#include "drishti/hci/FaceFinder.h"

#include <memory>

// clang-format off
namespace ogles_gpgpu
{
    class FacePainter;
    class TransformProc;
    class CircleProc;
}
// clang-format on

#define DRISHTI_HCI_FACE_FINDER_PAINTER_SHOW_CIRCLE 0

DRISHTI_HCI_NAMESPACE_BEGIN

class FaceFinderPainter : public FaceFinder
{
public:
    enum EffectKind
    {
        kWireframes,
        kStabilize
    };

    class Impl;
    using FaceFinderPtr = std::unique_ptr<FaceFinderPainter>;
    using FrameDelegate = std::function<void(const cv::Mat& ref)>;

    FaceFinderPainter(FaceDetectorFactoryPtr& factory, Settings& settings, void* glContext = nullptr);
    ~FaceFinderPainter();
    virtual void getOutputPixels(FrameDelegate& delegate);
    virtual void init(const cv::Size& inputSize);
    void drawIris(bool flag) { m_drawIris = flag; }
    void setLetterboxHeight(float height);
    static FaceFinderPtr create(FaceDetectorFactoryPtr& factory, Settings& settings, void* glContext);

    void setShowMotionAxes(bool value);
    bool getShowMotionAxes() const;

    void setShowDetectionScales(bool value);
    bool getShowDetectionScales() const;

    void setEffectKind(EffectKind kind);
    EffectKind getEffectKind() const;

protected:
    virtual void initPainter(const cv::Size& inputSizeUp);
    virtual GLuint paint(const ScenePrimitives& scene, GLuint inputTexture);

    GLuint filter(const ScenePrimitives& scene, GLuint inputTexture);

    ogles_gpgpu::RenderOrientation m_outputOrientation = ogles_gpgpu::RenderOrientationStd;
    EffectKind m_effect = kWireframes;
    cv::Size m_inputSize;
    cv::Size m_inputSizeUp;
    bool m_drawIris = false;

#if DRISHTI_HCI_FACE_FINDER_PAINTER_SHOW_CIRCLE
    std::shared_ptr<ogles_gpgpu::CircleProc> m_circle;
#endif
    std::shared_ptr<ogles_gpgpu::TransformProc> m_rotater;
    std::shared_ptr<ogles_gpgpu::FacePainter> m_painter;

    std::unique_ptr<Impl> m_pImpl;
};

DRISHTI_HCI_NAMESPACE_END

#endif // __drishti_hci_FaceFinderPainter_h__
