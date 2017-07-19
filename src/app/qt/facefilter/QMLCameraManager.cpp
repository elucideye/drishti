/*!
  @file   finder/QMLCameraManager.cpp
  @author David Hirvonen, Ruslan Baratov
  @brief  Platform abstraction for QML camera configuration

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.

  \license{This project is released under the 3 Clause BSD License.}

  Note: Refactoring of original code.
 
*/

#include "QMLCameraManager.h"

#include "drishti/core/drishti_core.h"
#include "drishti/core/make_unique.h"

#include <QQuickItem>
#include <QCamera>
#include <QCameraInfo>
#include <QCameraImageCapture>
#include <QMediaRecorder>

#include <spdlog/spdlog.h>

#include <memory>

// ###################################################################################
// ############################### QMLCameraManager ##################################
// ###################################################################################

std::string QMLCameraManager::getDeviceName() const
{
    return QCameraInfo(*m_camera).deviceName().toStdString();
}

std::string QMLCameraManager::getDescription() const
{
    return QCameraInfo(*m_camera).description().toStdString();
}

int QMLCameraManager::getOrientation() const
{
    return QCameraInfo(*m_camera).orientation();
}

cv::Size QMLCameraManager::getSize() const
{
    return m_size;
}

cv::Size QMLCameraManager::configure()
{
    return m_size = configureCamera();
}

std::unique_ptr<QMLCameraManager>
QMLCameraManager::create(QQuickItem* root, std::shared_ptr<spdlog::logger>& logger)
{
    // Manage the camera:
    QObject* qmlCamera = root->findChild<QObject*>("CameraObject");
    assert(qmlCamera != nullptr);

    QCamera* camera = qvariant_cast<QCamera*>(qmlCamera->property("mediaObject"));
    assert(camera != nullptr);

    QCameraImageProcessing* processing = camera->imageProcessing();
    if (processing->isAvailable())
    {
        processing->setWhiteBalanceMode(QCameraImageProcessing::WhiteBalanceAuto);
    }

    std::unique_ptr<QMLCameraManager> cameraManager;
#if defined(Q_OS_ANDROID)
    cameraManager = drishti::core::make_unique<QMLCameraManagerAndroid>(camera, logger);
#elif defined(Q_OS_IOS) || defined(Q_OS_OSX)
    cameraManager = drishti::core::make_unique<QMLCameraManagerApple>(camera, logger);
#elif defined(Q_OS_WIN)
    // TODO: Specialize
    cameraManager = drishti::core::make_unique<QMLCameraManage>(camera, logger);
#elif defined(Q_OS_UNIX)
    // TODO: Specialize
    cameraManager = drishti::core::make_unique<QMLCameraManage>(camera, logger);
#endif

    return cameraManager;
}

// ###################################################################################
// ################################## Apple ##########################################
// ###################################################################################

// Note: See example settings
// https://github.com/RSATom/Qt/blob/master/qtmultimedia/tests/auto/unit/qcamera/tst_qcamera.cpp

cv::Size QMLCameraManagerApple::configureCamera()
{
    cv::Size bestSize;

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
    auto viewfinderSettings = m_camera->supportedViewfinderSettings();

    auto pExposure = m_camera->exposure();
    if (pExposure)
    {
        m_logger->info("Exposure avaialble: {}", int(pExposure->isAvailable()));
    }

    m_logger->info("# of settings: {}", viewfinderSettings.size());

    m_logger->warn("Limiting video resolution to <= 2048");

    std::pair<int, QCameraViewfinderSettings> best;
    for (auto i : viewfinderSettings)
    {
        m_logger->info("settings: {} x {} : {}", i.resolution().width(), i.resolution().height(), int(i.pixelFormat()));
        if (std::find(desiredFormats.begin(), desiredFormats.end(), i.pixelFormat()) != desiredFormats.end())
        {
            int area = (i.resolution().height() * i.resolution().width());
            if ((area > best.first) && (i.resolution().width() <= 2048))
            {
                m_logger->info("Best camera resolution {}", best.first);
                best = { area, i };
            }
        }
    }

    best.second.setMinimumFrameRate(60.0);
    best.second.setMinimumFrameRate(120.0);

    assert(!best.second.isNull());
    m_camera->setViewfinderSettings(best.second);

    bestSize = { best.second.resolution().width(), best.second.resolution().height() };

    return bestSize;
}

// ###################################################################################
// ################################## Android ########################################
// ###################################################################################

cv::Size QMLCameraManagerAndroid::configureCamera()
{
    cv::Size bestSize;

    // viewfinderSettings doesn't work for Android:
    // * https://bugreports.qt.io/browse/QTBUG-50422
    // Experiments show that QCameraImageCapture can be used for this:
    // * https://github.com/headupinclouds/gatherer/issues/109
    // (probably not quite correctly)

    std::pair<QSize, int> best;
    QCameraImageCapture* imageCapture = new QCameraImageCapture(m_camera);
    QList<QVideoFrame::PixelFormat> formats = imageCapture->supportedBufferFormats();

    QList<QSize> resolutions = imageCapture->supportedResolutions();

    if (resolutions.size())
    {
        // This seems to work on Android, but not for iOS
        for (auto& i : resolutions)
        {
            int area = i.width() * i.height();
            if (area > best.second)
            {
                best = { i, area };
            }
            m_logger->info("video: {} {}", i.width(), i.height());
        }

        m_logger->info("best: {} {}", best.first.width(), best.first.height());

        bestSize = { best.first.width(), best.first.height() };

        QImageEncoderSettings imageSettings;
        imageSettings.setResolution(best.first);
        imageCapture->setEncodingSettings(imageSettings);
    }

    return bestSize;
}
