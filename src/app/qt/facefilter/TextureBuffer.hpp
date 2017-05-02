/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2015 Ruslan Baratov
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

#ifndef TEXTURE_BUFFER_HPP_
#define TEXTURE_BUFFER_HPP_

#include <QAbstractVideoBuffer>

class TextureBuffer : public QAbstractVideoBuffer
{
public:
    TextureBuffer(uint id)
        : QAbstractVideoBuffer(GLTextureHandle)
        , id_(id)
    {
        assert(id != 0);
    }

    static QVideoFrame::PixelFormat qtTextureFormat()
    {
#if defined(Q_OS_IOS)
        return QVideoFrame::Format_BGRA32;
#else
        return QVideoFrame::Format_BGR32;
#endif
    }

    static GLenum openglTextureFormat()
    {
#if defined(Q_OS_IOS)
        return GL_BGRA;
#else
        return GL_RGBA;
#endif
    }

    MapMode mapMode() const
    {
        return NotMapped;
    }

    uchar* map(MapMode, int*, int*)
    {
        return 0;
    }

    void unmap()
    {
    }

    QVariant handle() const
    {
        return QVariant::fromValue<uint>(id_);
    }

    /*
      Creates and returns a new video frame wrapping the OpenGL texture
      textureId. The size must be passed in size, together with the format of the
      underlying image data in format. When the texture originates from a QImage,
      use QVideoFrame::imageFormatFromPixelFormat() to get a suitable format.
      Ownership is not altered, the new QVideoFrame will not destroy the texture.
    */
    static QVideoFrame createVideoFrame(
        uint textureId,
        const QSize& size)
    {
        return QVideoFrame(new TextureBuffer(textureId), size, qtTextureFormat());
    }

private:
    GLuint id_;
};

#endif // TEXTURE_BUFFER_HPP_
