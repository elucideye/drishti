/*!
  @file   QGLContext.cpp
  @author David Hirvonen
  @brief  Implementation of a Qt based cross platform offscreen OpenGL context.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/gltest/QGLContext.h"
#include <QDebug>
#include <iostream>

DRISHTI_GLTEST_BEGIN

// https://forum.qt.io/topic/52688/console-application-with-opengl/2

QGLContext::QGLContext()
{
    QSurfaceFormat format;

#if defined(Q_OS_OSX)
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setVersion(2, 1);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
#endif

    m_context = std::make_shared<QOpenGLContext>();
    m_context->setFormat(format);

    // Note: If you crash here, it is likely that you have not instantiated:
    // QGuiApplication a(argc, argv) or ...
    // QApplication a(argc, argv)
    if (!m_context->create() || !m_context->isValid())
    {
        qDebug() << Q_FUNC_INFO << "QOpenGLContext create";
        return;
    }

    m_surface = std::make_shared<QOffscreenSurface>();
    m_surface->setFormat(format);
    m_surface->create();
    if (!m_surface->isValid())
    {
        qDebug() << Q_FUNC_INFO << "QOffscreenSurface create";
    }

    if (!m_context->makeCurrent(m_surface.get()))
    {
        qDebug() << Q_FUNC_INFO << "Shit happens with QOpenGLContext makeCurrent";
    }
}

QGLContext::~QGLContext()
{
    m_context->makeCurrent(m_surface.get());
    m_context->doneCurrent();
    m_surface->deleteLater();
}

QGLContext::operator bool() const
{
    return (m_context != nullptr) && (m_surface != nullptr);
}

DRISHTI_GLTEST_END
