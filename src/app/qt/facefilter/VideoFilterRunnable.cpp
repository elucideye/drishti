/****************************************************************************
 **
 ** Copyright (C) 2015 The Qt Company Ltd.
 ** Copyright (C) 2015 Ruslan Baratov
 ** Copyright (C) 2016 David Hirvonen
 ** Contact: http://www.qt.io/licensing/
 **
 ** This file is part of the examples of the Qt Multimedia module.
 **
 ** $QT_BEGIN_LICENSE:LGPL21$
 ** Commercial License Usage
 ** Licensees holding valid commercial Qt licenses may use this file in
 ** accordance with the commercial license agreement provided with the
 ** Software or, alternatively, in accordance with the terms contained in
 ** a written agreement between you and The Qt Company. For licensing terms
 ** and conditions see http://www.qt.io/terms-conditions. For further
 ** information use the contact form at http://www.qt.io/contact-us.
 **
 ** GNU Lesser General Public License Usage
 ** Alternatively, this file may be used under the terms of the GNU Lesser
 ** General Public License version 2.1 or version 3 as published by the Free
 ** Software Foundation and appearing in the file LICENSE.LGPLv21 and
 ** LICENSE.LGPLv3 included in the packaging of this file. Please review the
 ** following information to ensure the GNU Lesser General Public License
 ** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
 ** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
 **
 ** As a special exception, The Qt Company gives you certain additional
 ** rights. These rights are described in The Qt Company LGPL Exception
 ** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
 **
 ** $QT_END_LICENSE$
 **
 ****************************************************************************/

#include "VideoFilterRunnable.hpp"

#include <cassert> // assert

#include "graphics/drishti_graphics.h"

#include "FaceFinder.h"
#include "FrameHandler.h"
#include "VideoFilter.hpp"
#include "TextureBuffer.hpp"
#include "QVideoFrameScopeMap.h"
#include "QtFaceDetectorFactory.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <QDateTime>
#include <QFile>

struct VideoFilterRunnable::Impl
{
    using FrameInput = ogles_gpgpu::FrameInput;

    Impl(void *glContext, int orientation)
    {
        // Retrieve sensor intrinsic calibration:
        auto manager = FrameHandlerManager::get();
        
        // Allocate the face detector:
        std::shared_ptr<drishti::face::FaceDetectorFactory> resources = std::make_shared<QtFaceDetectorFactory>();
        
        FaceFinder::Config config;
        config.sensor = manager->getSensor();
        config.logger = manager->getLogger();
        config.threads = manager->getThreadPool();
        config.outputOrientation = orientation;
        config.delay = 1;
        
        m_detector = std::make_shared<FaceFinder>(resources, config, glContext);
    }

    GLuint operator()(const ogles_gpgpu::FrameInput &frame)
    {
        assert(m_detector);
        return (*m_detector)(frame);
    }

    std::shared_ptr<FaceFinder> m_detector;
};


VideoFilterRunnable::VideoFilterRunnable(VideoFilter *filter) :
    m_filter(filter),
    m_outTexture(0)
{
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    const char *vendor = (const char *) f->glGetString(GL_VENDOR);
    qDebug("GL_VENDOR: %s", vendor);
}

VideoFilterRunnable::~VideoFilterRunnable()
{

}

QVideoFrame VideoFilterRunnable::run(QVideoFrame *input, const QVideoSurfaceFormat &surfaceFormat, RunFlags flags)
{
    Q_UNUSED(surfaceFormat);
    Q_UNUSED(flags);

    QOpenGLContext * qContext = QOpenGLContext::currentContext();
    QOpenGLFunctions glFuncs(qContext);

    void* glContext = 0;
#if defined(Q_OS_IOS)
    glContext = ogles_gpgpu::Core::getCurrentEAGLContext();
#else
    glContext = qContext;
#endif

    if(!m_pImpl)
    {
        int orientation = FrameHandlerManager::get()->getOrientation();
        m_pImpl = std::make_shared<Impl>(glContext, orientation);
    }

    // This example supports RGB data only, either in system memory (typical with
    // cameras on all platforms) or as an OpenGL texture (e.g. video playback on
    // OS X).  The latter is the fast path where everything happens on GPU. The
    // former involves a texture upload.

    if (!isFrameValid(*input))
    {
        qWarning("Invalid input format");
        return *input;
    }

    if (isFrameFormatYUV(*input))
    {
        qWarning("YUV data is not supported");
        return *input;
    }

    m_outTexture = createTextureForFrame(input);
    return TextureBuffer::createVideoFrame(m_outTexture, input->size());
}

bool VideoFilterRunnable::isFrameValid(const QVideoFrame& frame)
{
    if (!frame.isValid())
    {
        return false;
    }
    if (frame.handleType() == QAbstractVideoBuffer::NoHandle)
    {
        return true;
    }
    if (frame.handleType() == QAbstractVideoBuffer::GLTextureHandle)
    {
        return true;
    }

    return false;
}

bool VideoFilterRunnable::isFrameFormatYUV(const QVideoFrame& frame)
{
    if (frame.pixelFormat() == QVideoFrame::Format_YUV420P)
    {
        return true;
    }
    if (frame.pixelFormat() == QVideoFrame::Format_YV12)
    {
        return true;
    }
    return false;
}

// Create a texture from the image data.
GLuint VideoFilterRunnable::createTextureForFrame(QVideoFrame* input)
{
    GLuint outTexture = detectFaces(input);
    return outTexture;
}

int VideoFilterRunnable::detectFaces(QVideoFrame *input)
{
    using FrameInput = ogles_gpgpu::FrameInput;

    QOpenGLContext* openglContext = QOpenGLContext::currentContext();
    if (!openglContext)
    {
        qWarning("Can't get context!");
        return 0;
    }
    assert(openglContext->isValid());

    QOpenGLFunctions *f = openglContext->functions();
    assert(f != 0);

    GLint inputTexture = 0, outputTexture = 0;
    GLenum textureFormat = 0;
    const ogles_gpgpu::Size2d size(input->width(), input->height());
    void* pixelBuffer = nullptr; //  we are using texture
    bool useRawPixels = false; //  - // -

    // Already an OpenGL texture.
    if (input->handleType() == QAbstractVideoBuffer::GLTextureHandle)
    {
        assert(input->pixelFormat() == TextureBuffer::qtTextureFormat());
        inputTexture = outputTexture = input->handle().toUInt();
        assert(inputTexture != 0);

        FrameInput frame(size, pixelBuffer, useRawPixels, inputTexture, GL_RGBA);
        m_outTexture = (*m_pImpl)(frame);
    }
    else
    {
        // Scope based pixel buffer lock for non ios platforms
        QVideoFrameScopeMap scopeMap(DRISHTI_IOS ? nullptr : input, QAbstractVideoBuffer::ReadOnly);
        if((DRISHTI_IOS && !scopeMap) || !DRISHTI_IOS) // for non ios platforms
        {
            assert((input->pixelFormat() == QVideoFrame::Format_ARGB32) || (DRISHTI_IOS && input->pixelFormat() == QVideoFrame::Format_NV12));

#if defined(Q_OS_IOS) || defined(Q_OS_OSX)
            const GLenum rgbaFormat = GL_BGRA;
#else
            const GLenum rgbaFormat = GL_RGBA;
#endif

#if defined(Q_OS_IOS)
            pixelBuffer = input->pixelBufferRef();
#else
            pixelBuffer = input->bits();
#endif

            // 0 indicates YUV
            textureFormat = (input->pixelFormat() == QVideoFrame::Format_ARGB32) ? rgbaFormat : 0;
            useRawPixels = !(DRISHTI_IOS); // ios uses texture cache / pixel buffer
            inputTexture = 0;
            assert(pixelBuffer != nullptr);

            FrameInput frame(size, pixelBuffer, useRawPixels, inputTexture, textureFormat);

            // Note: Pixel buffer access must occur within the protected scope  (see QVideoFrameScopeMap above)
            m_outTexture = (*m_pImpl)(frame);
        }
    }

    // Be sure to active GL_TEXTURE0 for Qt
    glActiveTexture(GL_TEXTURE0);

    return m_outTexture;
}

