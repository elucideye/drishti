/*!
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

// *INDENT-OFF*
namespace ogles_gpgpu
{
    class FacePainter;
    class TransformProc;
}
// *INDENT-ON*

DRISHTI_HCI_NAMESPACE_BEGIN

class FaceFinderPainter : public FaceFinder
{
public:

    FaceFinderPainter(FaceDetectorFactoryPtr &factory, Config &config, void *glContext = nullptr);
    virtual void init(const FrameInput &frame);

protected:
    
    virtual void initPainter(const cv::Size &inputSizeUp);    
    virtual GLuint paint(const ScenePrimitives &scene, GLuint inputTexture);    

    std::shared_ptr<ogles_gpgpu::TransformProc> m_rotater;
    std::shared_ptr<ogles_gpgpu::FacePainter> m_painter;
};

DRISHTI_HCI_NAMESPACE_END

#endif // __drishti_hci_FaceFinderPainter_h__
