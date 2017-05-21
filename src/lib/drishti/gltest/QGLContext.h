/*!
  @file   QGLContext.h
  @author David Hirvonen
  @brief  Declaration of a Qt based cross platform offscreen OpenGL context.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_gltest_QGLContext_h__
#define __drishti_gltest_QGLContext_h__

#include "drishti/gltest/GLContext.h"

#include <QApplication>
#include <QDesktopWidget>
#include <QSurfaceFormat>
#include <QtGlobal> // Q_OS_IOS

#include <QOffscreenSurface>
#include <QOpenGLContext>

#include <memory>

DRISHTI_GLTEST_BEGIN

class QGLContext : public GLContext
{
public:

    QGLContext();
    ~QGLContext();
    operator bool() const;

protected:
    
    std::shared_ptr<QOffscreenSurface> m_surface;
    std::shared_ptr<QOpenGLContext> m_context;
};

DRISHTI_GLTEST_END

#endif // __drishti_gltest_QGLContext_h__
