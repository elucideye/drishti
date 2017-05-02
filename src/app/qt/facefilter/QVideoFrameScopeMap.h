//
//  QVideoFrameScopeMap.h
//  drishtisdk
//
//  Created by David Hirvonen on 2/26/16.
//
//

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
