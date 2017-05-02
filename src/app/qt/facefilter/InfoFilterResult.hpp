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

#ifndef INFO_FILTER_RESULT_HPP_
#define INFO_FILTER_RESULT_HPP_

#include <QString>

class InfoFilterResult : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QSize frameResolution READ frameResolution)
    Q_PROPERTY(QString handleType READ handleType)
    Q_PROPERTY(QString pixelFormat READ pixelFormat)
    Q_PROPERTY(int fps READ fps)

public:
    QSize frameResolution() const
    {
        return m_frameResolution;
    }
    QString handleType() const
    {
        return m_handleType;
    }
    QString pixelFormat() const
    {
        return m_pixelFormat;
    }
    int fps() const
    {
        return m_fps;
    }

private:
    QSize m_frameResolution;
    QString m_handleType;
    QString m_pixelFormat;
    int m_fps;
    friend class InfoFilterRunnable;
};

#endif // INFO_FILTER_RESULT_HPP_
