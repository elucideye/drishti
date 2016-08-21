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

#include <functional>
#include <vector>

class FrameHandlerManager
{
public:

    typedef std::function<void(const cv::Mat &)> FrameHandler;
    FrameHandlerManager();
    ~FrameHandlerManager();
    static FrameHandlerManager *get();

    int getOrientation() const
    {
        return m_orientation;
    }

    void setOrientation(int orientation)
    {
        m_orientation = orientation;
    }

    void add(FrameHandler &handler)
    {
        m_handlers.push_back(handler);
    }

    std::vector<FrameHandler> &getHandlers()
    {
        return m_handlers;
    }

protected:

    int m_orientation = 0;

    static FrameHandlerManager * m_instance;

    std::vector< FrameHandler>  m_handlers;
};

#endif // _frame_handler_h_
