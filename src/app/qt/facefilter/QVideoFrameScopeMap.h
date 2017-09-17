/*! -*-c++-*-
  @file   QVideoFrameScopeMap.h
  @author David Hirvonen
  @brief  Declaration of a "scope guard" utility for QVideoFrame management.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_qt_facefilter_QVideoFrameScopeMap_h__
#define __drishti_qt_facefilter_QVideoFrameScopeMap_h__

#include <QVideoFilterRunnable>

struct QVideoFrameScopeMap
{
    QVideoFrameScopeMap(QVideoFrame* frame, QAbstractVideoBuffer::MapMode mode)
        : frame(frame)
    {
        if (frame)
        {
            status = frame->map(mode);
            if (!status)
            {
                qWarning("Can't map!");
            }
        }
    }
    ~QVideoFrameScopeMap()
    {
        if (frame)
        {
            frame->unmap();
        }
    }
    operator bool() const
    {
        return status;
    }
    QVideoFrame* frame = nullptr;
    bool status = false;
};

#endif /* __drishti_qt_facefilter_QVideoFrameScopeMap_h__ */
