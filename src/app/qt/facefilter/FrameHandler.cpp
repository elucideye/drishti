/*! -*-c++-*-
  @file   finder/FrameHandler.cpp
  @author David Hirvonen
  @brief Common state and stored frame handlers.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FrameHandler.h"
#include "QtFaceMonitor.h"

#include "drishti/sensor/Sensor.h"
#include "drishti/core/Logger.h"
#include "drishti/core/make_unique.h"

#include "nlohmann_json.hpp" // nlohman-json + ANDROID stdlib patch

// clang-format off
#if DRISHTI_USE_BEAST
#  include "ImageLogger.h"
#endif
// clang-format on

#include <QTextStream>
#include <QFile>

// Sample:
//
//"sensor": {
//    "intrinsic": {
//        "size": {
//            "width": 640,
//            "height": 852
//        }
//        "principal": {
//            "x": 320.0,
//            "y": 426.0
//        }
//        "focal_length_x": 768.0
//     }
// }

#define DRISHTI_STACK_LOGGING_DEMO 0

std::unique_ptr<nlohmann::json> loadJSON(spdlog::logger& logger);

FrameHandlerManager* FrameHandlerManager::m_instance = nullptr;

FrameHandlerManager::FrameHandlerManager(const std::string& name, const std::string& description, const GLVersion& glVersion)
    : m_glVersion(glVersion)
{
    m_logger = drishti::core::Logger::create("drishti");
    m_logger->info("FrameHandlerManager #################################################################");

    m_settings = loadJSON(*m_logger);

    const auto& device = (*m_settings)[name];
    if (device.empty())
    {
        m_logger->error("Failure to parse settings for device: {}", name);
        return;
    }

#if DRISHTI_USE_BEAST
    const auto& address = (*m_settings)["ipAddress"];
    if (!address.empty())
    {
        bool active = address["active"].get<bool>();
        if (active)
        {
            std::string host = address["host"];
            std::string port = address["port"];

            float frequency = address["frequency"];
            m_imageLogger = std::make_shared<drishti::core::ImageLogger>(host, port);
            m_imageLogger->setMaxFramesPerSecond(frequency); // throttle network traffic
        }
    }
#endif

    // Parse detection parameters (minDepth, maxDepth)
    const auto& detectionParams = (*m_settings)["detection"];
    if (!detectionParams.empty())
    {
        m_detectionParams.m_minDepth = detectionParams["minDepth"];
        m_detectionParams.m_maxDepth = detectionParams["maxDepth"];
        m_detectionParams.m_interval = detectionParams["interval"];
        m_detectionParams.m_singleFace = detectionParams["singleFace"];
        m_detectionParams.m_minTrackHits = detectionParams["minTrackHits"];
        m_detectionParams.m_maxTrackMisses = detectionParams["maxTrackMisses"];
        m_detectionParams.m_minSeparation = detectionParams["minSeparation"];
    }

    const auto& sensor = device["sensor"];
    const auto& intrinsic = sensor["intrinsic"];

    cv::Size size;
    const auto& sizeParams = intrinsic["size"];
    if (!sizeParams.empty())
    {
        size.width = sizeParams["width"].get<int>();
        size.height = sizeParams["height"].get<int>();
    }

    cv::Point2f p;
    const auto& principalParams = intrinsic["principal"];
    if (!principalParams.empty())
    {
        p.x = principalParams["x"].get<float>();
        p.y = principalParams["y"].get<float>();
    }

    const auto fx = intrinsic["focal_length_x"].get<float>();

    drishti::sensor::SensorModel::Intrinsic params(p, fx, size);
    m_sensor = std::make_shared<drishti::sensor::SensorModel>(params);

    m_threads = std::unique_ptr<tp::ThreadPool<>>(new tp::ThreadPool<>);

#if DRISHTI_STACK_LOGGING_DEMO
    m_faceMonitor = drishti::core::make_unique<QtFaceMonitor>(cv::Vec2d(0.12, 0.16), m_threads);
#endif
}

FrameHandlerManager::~FrameHandlerManager()
{
    if (m_instance)
    {
        delete m_instance;
    }
    m_instance = 0;
}

bool FrameHandlerManager::good() const
{
    return (m_sensor.get() != nullptr && m_threads.get() != nullptr);
}

auto FrameHandlerManager::createAsynchronousImageLogger() -> FrameHandler
{
#if DRISHTI_USE_BEAST
    if (!m_imageLogger)
    {
        return nullptr;
    }

    // clang-format off
    std::function<void(const cv::Mat&)> logger = [this](const cv::Mat& image)
    {
        if (m_imageLogger && !image.empty())
        {
            cv::Mat payload = image; // need to make a handle in this lambda
            std::function<void()> worker = [this,payload]()
            {
                try
                {
                    m_logger->info("Logging: {} : {} [{}x{}]", m_imageLogger->host(), m_imageLogger->port(), payload.cols, payload.rows);
                    (*m_imageLogger)(payload);
                }
                catch(std::exception &e)
                {
                    m_logger->error("facefilter: network error {}", e.what());
                }
            };
            m_threads->post(worker);
        }
    };
    // clang-format on

    return logger;
#else
    return nullptr;
#endif
}

void FrameHandlerManager::setSize(const cv::Size& size)
{
    if (m_sensor)
    {
        m_sensor->intrinsic().setSize(size);
    }
}

cv::Size FrameHandlerManager::getSize() const
{
    return m_sensor ? m_sensor->intrinsic().getSize() : cv::Size(0, 0);
}

FrameHandlerManager*
FrameHandlerManager::get(const std::string& name, const std::string& description, const GLVersion& glVersion)
{
    if (!m_instance)
    {
        m_instance = new FrameHandlerManager(name, description, glVersion);
    }
    return m_instance;
}

//////

std::unique_ptr<nlohmann::json> loadJSON(spdlog::logger& logger)
{
    std::unique_ptr<nlohmann::json> json;

    QString inputFilename(":/facefilter.json");
    QFile inputFile(inputFilename);
    if (!inputFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        logger.error("Can't open file");
        throw std::runtime_error("loadJSON: Can't open file");
    }

    QTextStream in(&inputFile);
    if (in.status() == QTextStream::Ok)
    {
        std::stringstream stream;
        stream << in.readAll().toStdString();

        json = drishti::core::make_unique<nlohmann::json>();
        stream >> (*json);
    }
    else
    {
        throw std::runtime_error("loadJSON: Can't read file");
    }

    return json;
}
