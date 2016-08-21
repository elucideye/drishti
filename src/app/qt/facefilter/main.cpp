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
#include <QQuickView>
#include <QQuickItem>
#include <QCamera>
#include <QQuickWindow>
#include <QCameraInfo>
#include <QCameraImageCapture>
#include <QMediaRecorder>
#include <QtPlugin> // Q_IMPORT_PLUGIN
#include <QQmlExtensionPlugin>
#include <QtOpenGL/QGLFormat>
#include <QFile>
#include <QTextStream>
#include <QDirIterator>

#include "VideoFilterRunnable.hpp"
#include "VideoFilter.hpp"
#include "InfoFilter.hpp"
#include "QTRenderGL.hpp"
#include "FrameHandler.h"
#include "QtStream.h"

#include "core/Logger.h"

#include "nlohmann/json.hpp" // nlohman-json

#include <iostream>

#if defined(Q_OS_OSX)
Q_IMPORT_PLUGIN(QtQuick2Plugin);
Q_IMPORT_PLUGIN(QMultimediaDeclarativeModule);
#endif

static void printResources()
{
    QDirIterator it(":", QDirIterator::Subdirectories);
    while (it.hasNext())
    {
        qDebug() << it.next();
    }
}

static nlohmann::json loadJSON(spdlog::logger &logger)
{
    nlohmann::json json;

    QFile inputFile(":/facefilter.json");
    if (!inputFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        logger.error() << "Can't open file";
        return EXIT_FAILURE;
    }
    
    QTextStream in(&inputFile);
    std::stringstream stream;
    stream <<  in.readAll().toStdString();
    stream >> json;
    
    return json;
}

#if defined(Q_OS_IOS)
extern "C" int qtmn(int argc, char** argv)
{
#else
int main(int argc, char **argv)
{
#endif
#ifdef Q_OS_WIN // avoid ANGLE on Windows
    QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
#endif
    
    // ###### Instantiate logger ########
    auto logger = drishti::core::Logger::create("facefilter");
    logger->info() << "Start";
    
    printResources();

    // JSON configuration
    auto json = loadJSON(*logger);
    
    //std::cout << json["pi"].get<float>() << std::endl;
    //std::cout << json["name"].get<std::string>() << std::endl;
    
    QGuiApplication app(argc, argv);

    qmlRegisterType<VideoFilter>("facefilter.test", 1, 0, "VideoFilter");
    qmlRegisterType<InfoFilter>("facefilter.test", 1, 0, "InfoFilter");
    qmlRegisterType<QTRenderGL>("OpenGLUnderQML", 1, 0, "QTRenderGL");

#if defined(Q_OS_OSX)
    qobject_cast<QQmlExtensionPlugin*>(qt_static_plugin_QtQuick2Plugin().instance())->registerTypes("QtQuick");
    qobject_cast<QQmlExtensionPlugin*>(qt_static_plugin_QMultimediaDeclarativeModule().instance())->registerTypes("QtMultimedia");
#endif

    QQuickView view;
    view.setSource(QUrl("qrc:///main.qml"));
    view.setResizeMode( QQuickView::SizeRootObjectToView );

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

    logger->info() << "OpenGL Versions Supported: " << QGLFormat::openGLVersionFlags();
#endif

    // Default camera on iOS is not setting good parameters by default
    QQuickItem* root = view.rootObject();

    QObject* qmlCamera = root->findChild<QObject*>("CameraObject");
    assert(qmlCamera != nullptr);

    QCamera* camera = qvariant_cast<QCamera*>(qmlCamera->property("mediaObject"));
    assert(camera != nullptr);

    QObject * qmlVideoOutput = root->findChild<QObject*>("VideoOutput");
    assert(qmlVideoOutput);

#if defined(Q_OS_ANDROID)
    {
        // viewfinderSettings doesn't work for Android:
        // * https://bugreports.qt.io/browse/QTBUG-50422
        // Experiments show that QCameraImageCapture can be used for this:
        // * https://github.com/headupinclouds/gatherer/issues/109
        // (probably not quite correctly)

        std::pair<QSize, int> best;
        QCameraImageCapture *imageCapture = new QCameraImageCapture(camera);
        QList<QVideoFrame::PixelFormat> formats = imageCapture->supportedBufferFormats();

        QList<QSize> resolutions = imageCapture->supportedResolutions();

        if(resolutions.size())
        {
            // This seems to work on Android, but not for iOS
            for(auto &i : resolutions)
            {
                int area = i.width() * i.height();
                if(area > best.second)
                {
                    best = {i, area};
                }
                logger->info() << "video: " << i.width() << " " << i.height();
            }

            logger->info() << "best: " << best.first.width() << " " << best.first.height();

            QImageEncoderSettings imageSettings;
            imageSettings.setResolution(best.first);
            imageCapture->setEncodingSettings(imageSettings);
        }
    }
#endif // Q_OS_ANDROID

#if defined(Q_OS_IOS) || defined(Q_OS_OSX)
    {
        // Not available in Android:
        // https://bugreports.qt.io/browse/QTBUG-46470

        // Try the highest resolution NV{12,21} format format:
        // This should work for both Android and iOS
        std::vector<QVideoFrame::PixelFormat> desiredFormats;

#if defined(Q_OS_IOS)
        desiredFormats = { QVideoFrame::Format_NV12, QVideoFrame::Format_NV21 };
#else
        desiredFormats = { QVideoFrame::Format_ARGB32 };
#endif
        auto viewfinderSettings = camera->supportedViewfinderSettings();

        logger->info() << "# of settings: " << viewfinderSettings.size();

        std::pair<int, QCameraViewfinderSettings> best;
        for (auto i: viewfinderSettings)
        {
            logger->info() << "settings: " << i.resolution().width() << "x" << i.resolution().height() << " : " << int(i.pixelFormat());
            if(std::find(desiredFormats.begin(), desiredFormats.end(), i.pixelFormat()) != desiredFormats.end())
            {
                int area = (i.resolution().height() * i.resolution().width());
                if(area > best.first)
                {
                    best = { area, i };
                }
            }
        }
        assert(!best.second.isNull());
        camera->setViewfinderSettings(best.second);
    }
#endif // Q_OS_IOS

    auto frameHandlers = FrameHandlerManager::get();
    if(frameHandlers)
    {
        QCameraInfo cameraInfo( *camera );
        frameHandlers->setOrientation(cameraInfo.orientation());
    }

    view.showFullScreen();

    return app.exec();
}
