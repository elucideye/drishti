/*! -*-c++-*-
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
#include "drishti/graphics/drishti_graphics.h"

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

QMLCameraManager::QMLCameraManager(QCamera* camera, std::shared_ptr<spdlog::logger>& logger)
    : m_camera(camera)
    , m_logger(logger)
{
}

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

// Note: See example settings
// https://github.com/RSATom/Qt/blob/master/qtmultimedia/tests/auto/unit/qcamera/tst_qcamera.cpp

cv::Size QMLCameraManager::configure()
{
    return m_size = configureCamera();
}

cv::Size QMLCameraManager::configureCamera()
{
    cv::Size bestSize;

    // Try the highest resolution NV{12,21} format format:
    // This should work for both Android and iOS
    std::vector<QVideoFrame::PixelFormat> desiredFormats;

#if defined(Q_OS_IOS) || defined(Q_OS_ANDROID)
    desiredFormats = { QVideoFrame::Format_NV12, QVideoFrame::Format_NV21 };
#else
    desiredFormats = { QVideoFrame::Format_ARGB32 };
#endif

    auto viewfinderSettings = m_camera->supportedViewfinderSettings();

    QCameraImageProcessing* processing = m_camera->imageProcessing();
    if (processing->isAvailable())
    {
        processing->setWhiteBalanceMode(QCameraImageProcessing::WhiteBalanceAuto);
    }

    auto pExposure = m_camera->exposure();
    if (pExposure)
    {
        std::vector<QCameraExposure::ExposureMode> exposureModes = {
            QCameraExposure::ExposureAuto,
            QCameraExposure::ExposurePortrait,
            QCameraExposure::ExposureBacklight
        };
        for (const auto& mode : exposureModes)
        {
            if (pExposure->isExposureModeSupported(mode))
            {
                pExposure->setExposureMode(mode);
                break;
            }
        }

        std::vector<QCameraExposure::MeteringMode> meteringModes = {
            QCameraExposure::MeteringSpot,
            QCameraExposure::MeteringMatrix,
            QCameraExposure::MeteringAverage
        };
        for (const auto& mode : meteringModes)
        {
            if (pExposure->isMeteringModeSupported(mode))
            {
                m_logger->info("Exposure metering mode supported: {}", int(mode));
                pExposure->setMeteringMode(mode);
                if (mode == QCameraExposure::MeteringSpot)
                {
                    pExposure->setSpotMeteringPoint({ 0.5, 0.5 });
                }
                break;
            }
        }
    }

    auto pImageProcessing = m_camera->imageProcessing();
    if (pImageProcessing)
    {
        m_logger->info("Image processing: {}", int(pImageProcessing->isAvailable()));
        pImageProcessing->setWhiteBalanceMode(QCameraImageProcessing::WhiteBalanceAuto);
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

    best.second.setMinimumFrameRate(30.0);

    assert(!best.second.isNull());
    m_camera->setViewfinderSettings(best.second);

    bestSize = { best.second.resolution().width(), best.second.resolution().height() };

    return bestSize;
}
