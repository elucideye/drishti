//
//  QVideoFrameScopeMap.h
//  drishtisdk
//
//  Created by David Hirvonen on 2/26/16.
//
//

#ifndef QVideoFrameScopeMap_h
#define QVideoFrameScopeMap_h

#include <QVideoFilterRunnable>

struct QVideoFrameScopeMap
{
    QVideoFrameScopeMap(QVideoFrame *frame, QAbstractVideoBuffer::MapMode mode) : frame(frame)
    {
        if(frame)
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
        if(frame)
        {
            frame->unmap();
        }
    }
    operator bool() const
    {
        return status;
    }
    QVideoFrame *frame = nullptr;
    bool status = false;
};

#endif /* QVideoFrameScopeMap_h */
