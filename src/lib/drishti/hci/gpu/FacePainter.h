/*!
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
#include "drishti/acf/drishti_acf.h"
#include "ogles_gpgpu/common/proc/transform.h"
#include "ogles_gpgpu/common/common_includes.h"

#include "drishti/hci/gpu/LineDrawing.hpp"

#include "drishti/face/Face.h" // face model

#include <opencv2/core/core.hpp>

#include <memory>
#include <array>

BEGIN_OGLES_GPGPU

class FlashFilter;

class FacePainter : public ogles_gpgpu::TransformProc
{
public:

    struct DisplayTexture
    {
        DisplayTexture() {}
        DisplayTexture(GLint texId, const Size2d &size) : texId(texId), size(size) {}
        DisplayTexture(GLint texId, const Size2d &size, const Rect2d &roi) : texId(texId), size(size), roi(roi) {}

        GLint texId = -1;
        ogles_gpgpu::Size2d size;
        ogles_gpgpu::Rect2d roi; // destination in FBO pixel coordinates

        void annotate()
        {
            if(m_delegate)
            {
                //m_delegate();
            }
        }

        std::function<void()> m_delegate;
    };

    /**
     * Constructor
     */
    FacePainter(int orientation);

    /**
     * Render a face image.
     */
    int render(int position = 0);

    /**
     * Return the processors name.
     */
    virtual const char *getProcName()
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
    virtual const char *getFragmentShaderSource()
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
    void addFace(const drishti::face::FaceModel &face)
    {
        m_faces.push_back(face);
    }

    //===========================
    //========= LINES ===========
    //===========================
    
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
    void addLineDrawing(const LineDrawing &drawing)
    {
        m_drawings.push_back(drawing);
    }

    //===========================
    //========= FLOW -===========
    //===========================

    void setFlowTexture(GLint texIdx, const ogles_gpgpu::Size2d &size)
    {
        Size2d flowSize(outFrameW, outFrameW * size.height / size.width);
        Rect2d flowRoi(0, outFrameH - flowSize.height - 1, flowSize.width, flowSize.height);
        m_flowInfo = { texIdx, size, flowRoi };
    }

    //===========================
    //========= FLASH ===========
    //===========================

    void setFlashTexture(GLint texIdx, const ogles_gpgpu::Size2d &size)
    {
        const int flashWidth = outFrameW/4;
        Size2d flashSize(flashWidth, flashWidth * size.height / size.width);
        Rect2d flashRoi(0, (outFrameH/2)-(flashSize.height/2), flashSize.width, flashSize.height);
        m_flashInfo = { texIdx, size, flashRoi };
    }

    //===========================
    //========= IRIS ============
    //===========================

    void setIrisTexture(int index, GLint texIdx, const ogles_gpgpu::Size2d &size)
    {
        int shrink = 3; // vertical shrink for small displays
        assert(index >= 0 && index <= 1);
        Size2d irisSize(outFrameW, (outFrameW * m_eyesInfo.size.height / m_eyesInfo.size.width) / shrink);
        Rect2d irisRoi(0, index * (outFrameH - irisSize.height - 1), irisSize.width, irisSize.height);
        m_irisInfo[index] = { texIdx, size, irisRoi };
    }

    //===========================
    //========= EYES ============
    //===========================

    void setEyeTexture(GLint texIdx, const ogles_gpgpu::Size2d &size, const std::array<EyeWarp, 2> &eyes)
    {
        // A) This stretches and preserves the aspect ratio across the top of the frame:
        //Rect2d eyesRoi(0, 0, outFrameW, outFrameW * size.height/size.width);

        // B) This stretches and centers the roi on top
        const int maxWidth = 480;
        const int width = std::min(maxWidth, outFrameW);
        Rect2d eyesRoi(0, 0, width, width * size.height/size.width);
        eyesRoi.x = (outFrameW / 2) - (size.width / 2);
        eyesRoi.y = eyesRoi.height / 4;

        m_eyesInfo = { texIdx, size, eyesRoi };

        m_eyesInfo.m_delegate = [&]()
        {
            annotateEyes();
        };
        m_eyes = eyes;
    }

    void copyEyeTex();
    void annotateEyes();

    // Implement all utilty texture drawing in terms of these:
    void renderTex( DisplayTexture &texInfo);
    void filterRenderPrepareTex( DisplayTexture &texInfo);
    void filterRenderSetCoordsTex( DisplayTexture &texInfo);
    
private:

    cv::Matx33f uprightImageToTexture();

    void renderFaces();
    std::array<EyeWarp, 2> renderEyes(const drishti::face::FaceModel &face); // return transformations
    void renderEye(const cv::Rect &roi, const cv::Matx33f &H, const DRISHTI_EYE::EyeModel &eye);
    void annotateEye(const cv::Rect &dstRoiPix, const cv::Matx33f &Heye, const DRISHTI_EYE::EyeModel &eye);

    virtual void renderDrawings();

    // Make sure these aren't called
    virtual void setOutputRenderOrientation(RenderOrientation o);
    virtual void setOutputSize(float scaleFactor);

    //static const char *fshaderTransformSrc;  // fragment shader source

    static const char *fshaderColorSrc; // fragment shader source
    static const char *vshaderColorSrc;

    static const char *fshaderColorVaryingSrc; // fragment shader source
    static const char *vshaderColorVaryingSrc;

    static const char *fshaderLetterBoxSrc;

    int m_outputOrientation = 0;

    uint64_t m_frameIndex = 0;
    
    // Generic line drawings (debugging) and colors:
    std::vector<cv::Vec3f> m_colorBuf;
    std::vector<LineDrawing> m_drawings;
    std::vector<LineDrawing> m_permanentDrawings; // not erased

    // #### Face list
    std::vector<drishti::face::FaceModel> m_faces;

    // #### Draw eye texture
    DisplayTexture m_eyesInfo;

    std::array<EyeWarp, 2> m_eyes;
    cv::Rect m_eyesRoi;

    // #### Draw shader ####
    std::shared_ptr<Shader> m_draw;
    GLint m_drawShParamAColor;
    GLint m_drawShParamAPosition;
    GLint m_drawShParamULineColor;
    GLint m_drawShParamUMVP;

    // #### color stuff ###
    Vec3f m_colorRGB;
    GLfloat m_colorWeight;
    GLint m_colorShParamRGB;
    GLint m_colorShParamUWeight;

    // #### Draw flow texture
    DisplayTexture m_flowInfo;

    // #### Flash texture ###
    DisplayTexture m_flashInfo;

    // #### Iris textures ####
    DisplayTexture m_irisInfo[2];
};

END_OGLES_GPGPU

#endif // __drishti_hci_gpu_FacePainter_h__
