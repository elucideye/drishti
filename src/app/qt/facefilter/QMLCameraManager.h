/*! -*-c++-*-
  @file   finder/QMLCameraManager.h
  @author David Hirvonen, Ruslan Baratov
  @brief  Platform abstraction for QML camera configuration

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Note: Refactoring of original code.
 
*/

#ifndef __drishti_qt_facefilter_QMLCameraManager_h__
#define __drishti_qt_facefilter_QMLCameraManager_h__ 1

#include <memory>           // std::shared_ptr
#include <opencv2/core.hpp> // for cv::Size

// clang-format off
namespace spdlog { class logger; }
// clang-format on
class QCamera;
class QQuickItem;

class QMLCameraManager
{
public:
    QMLCameraManager(QCamera* camera, std::shared_ptr<spdlog::logger>& logger);
    virtual std::string getDeviceName() const;
    virtual std::string getDescription() const;
    virtual int getOrientation() const;
    virtual cv::Size getSize() const;
    virtual cv::Size configure();

protected:
    virtual cv::Size configureCamera();

    cv::Size m_size;
    QCamera* m_camera = nullptr;
    std::shared_ptr<spdlog::logger> m_logger;
};

#endif // __drishti_qt_facefilter_QMLCameraManager_h__
