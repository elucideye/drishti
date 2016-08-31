/*!
  @file   finder/FrameHandler.h
  @author David Hirvonen
  @brief Common state and stored frame handlers.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef _frame_handler_h_
#define _frame_handler_h_

#include <opencv2/core/core.hpp>

#if USE_LOCAL_THREAD_POOL
#  include <thread_pool.hpp>
#else
#  include <thread-pool-cpp/thread_pool.hpp>
#endif

#include "nlohmann/json.hpp" // nlohman-json

#include <functional>
#include <vector>

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

    using Settings=nlohmann::json;
    typedef std::function<void(const cv::Mat &)> FrameHandler;
    
    FrameHandlerManager(Settings *settings=nullptr);
    
    ~FrameHandlerManager();
    
    static FrameHandlerManager *get(Settings *settings=nullptr);

    int getOrientation() const
    {
        return m_orientation;
    }

    void setOrientation(int orientation)
    {
        m_orientation = orientation;
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

protected:

    Settings *m_settings = nullptr;
    
    int m_orientation = 0;
    
    std::shared_ptr<spdlog::logger> m_logger;

    std::shared_ptr<ThreadPool<128>> m_threads;

    std::shared_ptr<drishti::sensor::SensorModel> m_sensor;
    
    static FrameHandlerManager * m_instance;

    std::vector< FrameHandler>  m_handlers;
};

#endif // _frame_handler_h_
