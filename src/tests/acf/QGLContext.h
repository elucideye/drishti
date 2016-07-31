/*!
  @file   QGLContext.h
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Declaration of a Qt based cross platform offscreen OpenGL context.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <QApplication>
#include <QDesktopWidget>
#include <QSurfaceFormat>
#include <QtGlobal> // Q_OS_IOS

#include <QOffscreenSurface>
#include <QOpenGLContext>

#include <memory>

class QGLContext
{
public:
    QGLContext();
protected:
    std::shared_ptr<QOffscreenSurface> m_surface;
    std::shared_ptr<QOpenGLContext> m_context;
};

