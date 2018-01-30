/*! -*-c++-*-
  @file   finder/FacePainter.h
  @author David Hirvonen
  @brief Simple interface to draw faces and other primitives from the scene.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_gpu_FacePainter_h__
#define __drishti_hci_gpu_FacePainter_h__

/**
 * GPGPU transform processor.
 */

#include "drishti/face/gpu/EyeFilter.h"
#include "drishti/hci/gpu/LineDrawing.hpp"
#include "drishti/hci/Scene.hpp"
#include "drishti/face/Face.h" // face model
#include "drishti/geometry/Mesh3D.h"

#include "ogles_gpgpu/common/proc/transform.h"
#include "ogles_gpgpu/common/common_includes.h"

#include <opencv2/core/core.hpp>

#include <memory>
#include <array>

// clang-format off
namespace spdlog { class logger; }
// clang-format on

BEGIN_OGLES_GPGPU

class GLPrinterShader;

class FacePainter : public ogles_gpgpu::TransformProc
{
public:
    using FlowField = std::vector<cv::Vec4f>;
    using FeaturePoint = drishti::hci::FeaturePoint;
    using FeaturePoints = std::vector<FeaturePoint>;
    using PointSet = std::vector<cv::Point2f>;
    using Axes3D = std::array<drishti::geometry::Mesh3D<float>, 3>;
    using EyeWarpPair = std::array<drishti::eye::EyeWarp, 2>;

    struct DisplayTexture
    {
        DisplayTexture() {}
        DisplayTexture(GLint texId, const Size2d& size)
            : texId(texId)
            , size(size)
        {
        }
        DisplayTexture(GLint texId, const Size2d& size, const Rect2d& roi)
            : texId(texId)
            , size(size)
            , roi(roi)
        {
        }

        GLint texId = -1;
        ogles_gpgpu::Size2d size;
        ogles_gpgpu::Rect2d roi; // destination in FBO pixel coordinates

        void annotate()
        {
            if (m_delegate)
            {
                m_delegate();
            }
        }

        std::function<void()> m_delegate;
    };

    struct EyeAttributes
    {
        EyeAttributes() {}
        EyeAttributes(const FeaturePoints* points, float scale = 1.f, const cv::Vec3f& color = { 0.f, 1.f, 0.f })
            : points(points)
            , scale(scale)
            , color(color)
        {
        }

        const FeaturePoints* points = nullptr;
        float scale = 10.f;
        cv::Vec3f color = { 0.f, 1.f, 0.f };
        const FlowField* flow = nullptr;
    };

    struct EyePairInfo
    {
        DisplayTexture m_eyesInfo;
        EyeWarpPair m_eyes;
        cv::Rect m_eyesRoi;
    };

    struct Object3D
    {
        cv::Point3f position;
        cv::Vec3f velocity;
    };

    /**
     * Constructor
     */
    FacePainter(int orientation);

    /**
     * Destructor
     */
    ~FacePainter();

    /**
     * Render a face image.
     */
    int render(int position = 0);

    /**
     * Return the processors name.
     */
    virtual const char* getProcName()
    {
        return "FacePainter";
    }

    /**
     * Basic init.
     */
    virtual int init(int inW, int inH, unsigned int order, bool prepareForExternalInput);

    /**
     * Get the fragment shader source.
     */
    virtual const char* getFragmentShaderSource()
    {
        return fshaderLetterBoxSrc;
    }

    virtual void getUniforms();
    virtual void setUniforms();

    virtual void clear();

    //===========================
    //========= FACE ============
    //===========================

    // Get input face model list:
    std::vector<drishti::face::FaceModel>& getFaces()
    {
        return m_faces;
    }
    const std::vector<drishti::face::FaceModel>& getFaces() const
    {
        return m_faces;
    }

    // Add input face model
    void addFace(const drishti::face::FaceModel& face)
    {
        m_faces.push_back(face);
    }

    // Get input line drawing list:
    std::vector<LineDrawing>& getPermanentLineDrawings()
    {
        return m_permanentDrawings;
    }
    const std::vector<LineDrawing>& getPermanentLineDrawings() const
    {
        return m_permanentDrawings;
    }

    // Get input line drawing list:
    std::vector<LineDrawing>& getLineDrawings()
    {
        return m_drawings;
    }
    const std::vector<LineDrawing>& getLineDrawings() const
    {
        return m_drawings;
    }

    // Add input line drawings:
    void addLineDrawing(const LineDrawing& drawing)
    {
        m_drawings.push_back(drawing);
    }

    void setAxes(const cv::Point3f& axes);

    void setObjectState(const Object3D& object)
    {
        m_object = object;
    }

    void setFlowTexture(GLint texIdx, const ogles_gpgpu::Size2d& size);
    
    void setBlobTexture(GLint texIdx, const ogles_gpgpu::Size2d& size);
    
    void setIrisTexture(int index, GLint texIdx, const ogles_gpgpu::Size2d& size);
    
    void setEyeTexture(GLint texIdx, const ogles_gpgpu::Size2d& size, const EyeWarpPair& eyes);
    
    void setEyesWidthRatio(float ratio)
    {
        m_eyesWidthRatio = ratio;
    }

    void setShowDetectionScales(bool value)
    {
        m_showDetectionScales = value;
    }

    bool getShowDetectionScales()
    {
        return m_showDetectionScales;
    }

    void setEyeMotion(const cv::Point2f& motion)
    {
        m_eyeMotion = motion;
    }

    // points: normalized points wrt eye textures
    void setEyePoints(const FeaturePoints& points)
    {
        m_eyePoints = points;
    }

    void setEyeFlow(const FlowField& flow)
    {
        m_eyeFlow = flow;
    }

    void setBrightness(float value)
    {
        m_brightness = value;
    }

    void setLetterboxHeight(float value)
    {
        m_colorLetterboxHeight = value;
    }

    void setGazePoint(const FeaturePoints& points)
    {
        m_gazePoints = points;
    }

    void setLogger(std::shared_ptr<spdlog::logger>& logger)
    {
        m_logger = logger;
    }

    void copyEyeTex();
    void annotateEyes(const EyePairInfo& eyes, const EyeAttributes& attributes);

    // Implement all utilty texture drawing in terms of these:
    void renderTex(DisplayTexture& texInfo);
    void filterRenderPrepareTex(DisplayTexture& texInfo);
    void filterRenderSetCoordsTex(DisplayTexture& texInfo);

private:
    cv::Matx33f uprightImageToTexture();

    void renderFaces();
    EyeWarpPair renderEyes(const drishti::face::FaceModel& face);
    void renderEye(const cv::Rect& roi, const cv::Matx33f& H, const DRISHTI_EYE::EyeModel& eye);
    void annotateEye(const drishti::eye::EyeWarp& eyeWarp, const cv::Size& size, const EyeAttributes& attributes);

    virtual void renderDrawings();
    virtual void renderAxes();
    virtual void renderGaze();

    // Make sure these aren't called
    virtual void setOutputRenderOrientation(RenderOrientation o);
    virtual void setOutputSize(float scaleFactor);

    //static const char *fshaderTransformSrc;  // fragment shader source

    static const char* fshaderColorSrc; // fragment shader source
    static const char* vshaderColorSrc;

    static const char* fshaderColorVaryingSrc; // fragment shader source
    static const char* vshaderColorVaryingSrc;

    static const char* fshaderLetterBoxSrc;

    FeaturePoints m_gazePoints;

    float m_brightness = 1.f;

    cv::Point2f m_eyeMotion;
    cv::Point3f m_motion;

    Axes3D m_axes;
    Axes3D m_axesColors;

    drishti::core::Field<Object3D> m_object;

    int m_outputOrientation = 0;

    uint64_t m_frameIndex = 0;

    bool m_showDetectionScales = false;

    // Generic line drawings (debugging) and colors:
    std::vector<cv::Vec3f> m_colorBuf;
    std::vector<LineDrawing> m_drawings;
    std::vector<LineDrawing> m_permanentDrawings; // not erased

    // #### Face list
    std::vector<drishti::face::FaceModel> m_faces;

    // #### Draw eye texture
    float m_eyesWidthRatio = 0.25f;
    EyePairInfo m_eyes;
    EyeAttributes m_eyeAttributes;

    FlowField m_eyeFlow;
    FeaturePoints m_eyePoints;

    std::unique_ptr<GLPrinterShader> m_printer;

    // #### Draw shader ####
    std::shared_ptr<Shader> m_draw;
    GLint m_drawShParamAColor;
    GLint m_drawShParamAPosition;
    GLint m_drawShParamULineColor;
    GLint m_drawShParamUMVP;

    // #### color stuff ###
    Vec3f m_colorRGB;
    GLint m_colorShParamRGB;

    GLfloat m_colorLetterboxHeight;
    GLint m_colorShLetterboxHeight;

    // #### Draw flow texture
    DisplayTexture m_flowInfo;

    // #### Flash texture ###
    DisplayTexture m_blobInfo;

    // #### Iris textures ####
    DisplayTexture m_irisInfo[2];

    // ### Stream logging
    std::shared_ptr<spdlog::logger> m_logger;
};

END_OGLES_GPGPU

#endif // __drishti_hci_gpu_FacePainter_h__
