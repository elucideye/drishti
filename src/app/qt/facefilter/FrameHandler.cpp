/*!
  @file   finder/FrameHandler.cpp
  @author David Hirvonen
  @brief Common state and stored frame handlers.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FrameHandler.h"

FrameHandlerManager * FrameHandlerManager::m_instance = nullptr;

FrameHandlerManager::FrameHandlerManager()
{

}

FrameHandlerManager::~FrameHandlerManager()
{
    if(m_instance)
    {
        delete m_instance;
    }
    m_instance = 0;
}

FrameHandlerManager * FrameHandlerManager::get()
{
    if(!m_instance)
    {
        m_instance = new FrameHandlerManager;
    }
    return m_instance;
}
