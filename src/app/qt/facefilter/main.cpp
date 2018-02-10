/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2015 Ruslan Baratov
** Coypright (C) 2016 David Hirvonen
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

#include <cassert> // assert
#include <QGuiApplication>
#include <QQuickItem>
#include <QQmlExtensionPlugin>
#include <QFile>
#include <QTextStream>
#include <QDirIterator>
#include <QCameraExposure>

// Includes 'glew.h' {
#include "VideoFilterRunnable.hpp"
// }

// Includes 'gl2.h', after 'glew.h' {
#include <QQuickView>
#include <QtOpenGL/QGLFormat>

#if QT_VERSION >= QT_VERSION_CHECK(5, 6, 0)
#include <QOpenGLExtraFunctions>
#endif
// }

#include "GLVersion.h"
#include "QMLCameraManager.h"
#include "VideoFilter.hpp"
#include "InfoFilter.hpp"
#include "FrameHandler.h"
#include "QtStream.h"
#include "QtFaceMonitor.h"
#include "CameraListener.h"

#include "drishti/core/Logger.h"
#include "drishti/core/drishti_core.h"

#include <iostream>

// Utilities
static void printResources();

std::shared_ptr<CameraListener>
createCameraListener(QQuickItem* root, std::shared_ptr<spdlog::logger>& logger, const GLVersion& glVersion)
{
    // Manage the camera:
    QObject* qmlCamera = root->findChild<QObject*>("CameraObject");
    assert(qmlCamera != nullptr);
    CV_Assert(qmlCamera != nullptr);

    QCamera* camera = qvariant_cast<QCamera*>(qmlCamera->property("mediaObject"));
    assert(camera != nullptr);
    CV_Assert(camera != nullptr);

    return std::make_shared<CameraListener>(camera, logger, glVersion);
}

static QSurfaceFormat createGLESSurfaceFormat(const GLVersion& glVersion)
{
    QSurfaceFormat format;
    format.setSamples(4);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(glVersion.major, glVersion.minor);
    return format;
}

int facefilter_main(int argc, char** argv, std::shared_ptr<spdlog::logger>& logger)
{
#ifdef Q_OS_WIN // avoid ANGLE on Windows
    QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
#elif defined(Q_OS_IOS) || defined(Q_OS_ANDROID)
    QGuiApplication::setAttribute(Qt::AA_UseOpenGLES);
#endif

    // https://stackoverflow.com/questions/40385482/why-cant-i-use-opengl-es-3-0-in-qt
    GLVersion glVersion{ 2, 0 };
    bool usePBO = false;

#if defined(QT_OPENGL_ES_3) && defined(DRISHTI_OPENGL_ES3)
    glVersion.major = 3;
    usePBO = true;
#endif

    QSurfaceFormat::setDefaultFormat(createGLESSurfaceFormat(glVersion));

    // ###### Instantiate logger ########
    logger->info("Start");

    printResources();

    QGuiApplication app(argc, argv);

    qmlRegisterType<VideoFilter>("facefilter.test", 1, 0, "VideoFilter");
    qmlRegisterType<InfoFilter>("facefilter.test", 1, 0, "InfoFilter");

    QQuickView view;
    view.setSource(QUrl("qrc:///main.qml"));
    view.setResizeMode(QQuickView::SizeRootObjectToView);

#if defined(Q_OS_OSX)
    // This had been tested with GLFW + ogles_gpgpu before
    //OpenGL version: 2.1 NVIDIA-10.4.2 310.41.35f01
    //GLSL version: 1.20
    QSurfaceFormat format;
    format.setVersion(2, 1);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    view.setFormat(format);

    logger->info("OpenGL Versions Supported: {}", QGLFormat::openGLVersionFlags());
#endif

    // Default camera on iOS is not setting good parameters by default
    QQuickItem* root = view.rootObject();
    CV_Assert(root);

    // Create a CameraListener in order to trigger instantiation of classes
    // when the camera is in state QCamera::LoadedStatus.  The QML Android
    // camera does not response to manual camera->load() calls.
    // * QMLCameraManager : for camera customizations
    // * FrameHandlerManager : for application specific settings (depends on camera "name")
    auto listener = createCameraListener(root, logger, glVersion);
    listener->setUsePBO(usePBO);

#if defined(Q_OS_IOS)
    // On iOS we do not receive the
    // * iOS does not receive QCamera::LoadedStatus, but we can configure the camera manually
    // * Android does receive  QCamera::LoadedStatus,so we wait fot the signal
    listener->configureCamera();
#endif

    view.showFullScreen();
    return app.exec();
}

// QT uses a shared library for Android builds, so we must
// explicitly export the main function for it to be loadable
// with standard dlopen/dlsym pairs.

// clang-format off
#if defined(Q_OS_ANDROID)
#  include "facefilter_export.h"
#  define FACEFILTER_QT_EXPORT FACEFILTER_EXPORT
#else
#  define FACEFILTER_QT_EXPORT
#endif
// clang-format on

extern "C" FACEFILTER_QT_EXPORT int main(int argc, char** argv)
{
    auto logger = drishti::core::Logger::create("facefilter");
    try
    {
        return facefilter_main(argc, argv, logger);
    }
    catch (std::exception& exc)
    {
        logger->error("Exception caught: {}", exc.what());
        return EXIT_FAILURE;
    }
    catch (...)
    {
        logger->error("Unknown exception caught");
        return EXIT_FAILURE;
    }
    return 0;
}

static void printResources()
{
    QDirIterator it(":", QDirIterator::Subdirectories);
    while (it.hasNext())
    {
        qDebug() << it.next();
    }
}
