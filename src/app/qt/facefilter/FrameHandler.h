/*!
  @file   finder/FrameHandler.h
  @author David Hirvonen
  @brief Common state and stored frame handlers.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef _frame_handler_h_
#define _frame_handler_h_

#include "drishti/hci/FaceMonitor.h"

#include <opencv2/core/core.hpp>

#include "thread_pool/thread_pool.hpp"

//#include "nlohmann/json.hpp" // nlohman-json
#include "nlohmann_json.hpp" // nlohman-json

#include <functional>
#include <vector>
#include <memory>

// *INDENT-OFF*
namespace drishti
{
    namespace sensor
    {
        class SensorModel;
    };
};

namespace spdlog { class logger; }
// *INDENT-ON*

class FrameHandlerManager
{
public:
    
    struct DetectionParams
    {
        float m_minDepth; // meters
        float m_maxDepth; // meters
    };
    
    using Settings=nlohmann::json;
    typedef std::function<void(const cv::Mat &)> FrameHandler;
    
    FrameHandlerManager(Settings *settings, const std::string &name, const std::string &description);
    
    ~FrameHandlerManager();
    
    bool good() const;
    
    static FrameHandlerManager *get(Settings *settings=nullptr, const std::string &name={}, const std::string &description={});

    int getOrientation() const
    {
        return m_orientation;
    }

    void setOrientation(int orientation)
    {
        m_orientation = orientation;
    }
    
    void setDeviceInfo(const std::string &name, const std::string &description)
    {
        m_deviceName = name;
        m_deviceDescription = description;
    }
    
    void setSize(const cv::Size &size);
    
    cv::Size getSize() const;

    void add(FrameHandler &handler)
    {
        m_handlers.push_back(handler);
    }

    std::vector<FrameHandler> &getHandlers()
    {
        return m_handlers;
    }
    
    std::shared_ptr<drishti::sensor::SensorModel> & getSensor()
    {
        return m_sensor;
    }
    
    std::shared_ptr<spdlog::logger> & getLogger()
    {
        return m_logger;
    }
    
    std::shared_ptr<ThreadPool<128>> & getThreadPool()
    {
        return m_threads;
    }
    
    const DetectionParams & getDetectionParameters()
    {
        return m_detectionParams;
    }
    
    drishti::hci::FaceMonitor* getFaceMonitor()
    {
        return m_faceMonitor.get();
    }
    
    Settings * getSettings() { return m_settings; }
    const Settings * getSettings() const { return m_settings; }

protected:

    Settings *m_settings = nullptr;
    
    DetectionParams m_detectionParams;
    
    std::string m_deviceName;
    
    std::string m_deviceDescription;
    
    int m_orientation = 0;
    
    std::shared_ptr<spdlog::logger> m_logger;

    std::shared_ptr<ThreadPool<128>> m_threads;

    std::shared_ptr<drishti::sensor::SensorModel> m_sensor;
    
    std::vector<FrameHandler> m_handlers;
    
    std::unique_ptr<drishti::hci::FaceMonitor> m_faceMonitor;
    
    static FrameHandlerManager * m_instance;
};

#endif // _frame_handler_h_
