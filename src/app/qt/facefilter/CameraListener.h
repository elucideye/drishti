#ifndef _facefilter_CameraListener_h_
#define _facefilter_CameraListener_h_

#include "GLVersion.h"

#include <QCamera>

#include <spdlog/spdlog.h>

#include <memory>

class CameraListener : public QObject
{
    Q_OBJECT

public:
    using LoggerPtr = std::shared_ptr<spdlog::logger>;

    CameraListener(QCamera* camera, LoggerPtr& logger, const GLVersion& version);

    void setUsePBO(bool flag);

    void configureCamera();

protected:
    QCamera* m_camera = nullptr;
    std::shared_ptr<spdlog::logger> m_logger;
    GLVersion m_glVersion;
    bool m_usePBO = false;

public slots:

    void updateCameraStatus(QCamera::Status status);
};

#endif // _facefilter_CameraListener_h_
