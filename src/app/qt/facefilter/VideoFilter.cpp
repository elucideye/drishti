/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Copyright (C) 2015 Ruslan Baratov
** Contact: http://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Multimedia module.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** As a special exception, The Qt Company gives you certain additional
** rights. These rights are described in The Qt Company LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "VideoFilter.hpp"
#include "VideoFilterRunnable.hpp"
#include "FrameHandler.h"

QVideoFilterRunnable* VideoFilter::createFilterRunnable()
{
    auto runnable = new VideoFilterRunnable(this);
    return runnable;
}

VideoFilter::VideoFilter()
    : m_factor(1)
    , m_outputString("Filter output")
{
    connect(this, SIGNAL(updateOutputString(QString)), this, SLOT(setOutputString(QString)));
    connect(this, SIGNAL(updateRectangle(QPoint, QSize, bool)), this, SLOT(setRectangle(QPoint, QSize, bool)));
}

void VideoFilter::setFactor(qreal v)
{
    if (m_factor != v)
    {
        m_factor = v;
        emit factorChanged();
    }
}

void VideoFilter::setOutputString(QString newOutput)
{
    if (m_outputString == newOutput)
    {
        return;
    }
    m_outputString = newOutput;
    emit outputStringChanged();
}

void VideoFilter::setLandmarkState(bool state)
{
    std::cout << "STATE: " << state << std::endl;
}

void VideoFilter::setRectangle(QPoint position, QSize size, bool visible)
{
    bool changed = false;

    if (m_rectanglePosition != position)
    {
        changed = true;
        m_rectanglePosition = position;
    }

    if (m_rectangleSize != size)
    {
        changed = true;
        m_rectangleSize = size;
    }

    if (m_rectangleVisible != visible)
    {
        changed = true;
        m_rectangleVisible = visible;
    }

    if (!changed)
    {
        return;
    }

    emit rectangleChanged();
}
