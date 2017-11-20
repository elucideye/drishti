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

#ifndef VIDEO_FILTER_RUNNABLE_HPP_
#define VIDEO_FILTER_RUNNABLE_HPP_

#include <QVideoFilterRunnable>

// Includes 'glew.h' {
#include <acf/GPUACF.h>
// }

// Includes 'gl2.h', after 'glew.h' {
#include <QOpenGLFunctions> // introduce GLuint in cross-platform fashion
// }

#include <memory> // std::shared_ptr

// clang-format off
namespace acf { class Detector; };
namespace ogles_gpgpu { struct ACF; };
// clang-format on

class OEGLGPGPUTest;
class VideoFilter;

class VideoFilterRunnable : public QVideoFilterRunnable
{
public:
    VideoFilterRunnable(VideoFilter* filter);
    ~VideoFilterRunnable();

    struct Impl;

    QVideoFrame run(
        QVideoFrame* input,
        const QVideoSurfaceFormat& surfaceFormat,
        RunFlags flags) Q_DECL_OVERRIDE;

private:
    static bool isFrameValid(const QVideoFrame& frame);
    static bool isFrameFormatYUV(const QVideoFrame& frame);

    GLuint detectFaces(const cv::Size& size, void* pixelBuffer, bool useRawPixels, GLuint inputTexture, GLenum textureFormat);

    GLuint createTextureForFrame(QVideoFrame* input);

    int processFrame(QVideoFrame* input);
    int detectFaces(QVideoFrame* input);

    VideoFilter* m_filter = nullptr;

    uint m_outTexture;

    std::shared_ptr<Impl> m_pImpl;
};

#endif // VIDEO_FILTER_RUNNABLE_HPP_
