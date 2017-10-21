#include "CameraListener.h"
#include "QMLCameraManager.h"
#include "FrameHandler.h"

#include "drishti/core/make_unique.h"

CameraListener::CameraListener(QCamera* camera, LoggerPtr& logger, const GLVersion& glVersion)
    : m_camera(camera)
    , m_logger(logger)
    , m_glVersion(glVersion)
{
    connect(camera, SIGNAL(statusChanged(QCamera::Status)), this, SLOT(updateCameraStatus(QCamera::Status)));
}

void CameraListener::setUsePBO(bool flag)
{
    m_usePBO = flag;
}

void CameraListener::updateCameraStatus(QCamera::Status status)
{
    if (status == QCamera::LoadedStatus)
    {
        configureCamera();
    }
}

void CameraListener::configureCamera()
{
    m_logger->info("Configuring camera");

    // Create a QMLCameraManager and configure camera
    auto qmlCameraManager = drishti::core::make_unique<QMLCameraManager>(m_camera, m_logger);
    auto size = qmlCameraManager->configure();

    if (!size.area())
    {
        m_logger->error("Unable to configure valid camera resolution");
        throw std::runtime_error("Unable to configure valid camera resolution");
    }

    m_logger->info("device: {}", qmlCameraManager->getDeviceName());
    m_logger->info("description: {}", qmlCameraManager->getDescription());
    m_logger->info("resolution: {} {}", qmlCameraManager->getSize().width, qmlCameraManager->getSize().height);

    auto frameHandler = FrameHandlerManager::get(qmlCameraManager->getDeviceName(), qmlCameraManager->getDescription(), m_glVersion);
    if (!frameHandler || !frameHandler->good())
    {
        m_logger->error("Failed to instantiate FrameHandlerManager");
        throw std::runtime_error("Failed to instantiate FrameHandlerManager");
    }

    if (frameHandler)
    {
        frameHandler->setUsePBO(m_usePBO);
        if (qmlCameraManager)
        {
            frameHandler->setOrientation(qmlCameraManager->getOrientation());
            frameHandler->setSize(qmlCameraManager->getSize());
        }
    }
}
