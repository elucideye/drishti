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

#ifndef VIDEO_FILTER_HPP_
#define VIDEO_FILTER_HPP_

#include <QAbstractVideoFilter>

class VideoFilter : public QAbstractVideoFilter
{
    Q_OBJECT
    Q_PROPERTY(qreal factor READ factor WRITE setFactor NOTIFY factorChanged)
    Q_PROPERTY(QString outputString READ outputString NOTIFY outputStringChanged)
    Q_PROPERTY(QPoint rectanglePosition READ rectanglePosition NOTIFY rectangleChanged)
    Q_PROPERTY(QSize rectangleSize READ rectangleSize NOTIFY rectangleChanged)
    Q_PROPERTY(bool rectangleVisible READ rectangleVisible NOTIFY rectangleChanged)

public:
    VideoFilter();

    qreal factor() const
    {
        return m_factor;
    }
    void setFactor(qreal v);

    QString outputString() const
    {
        return m_outputString;
    }

    QPoint rectanglePosition() const
    {
        return m_rectanglePosition;
    }
    QSize rectangleSize() const
    {
        return m_rectangleSize;
    }
    bool rectangleVisible() const
    {
        return m_rectangleVisible;
    }

    QVideoFilterRunnable* createFilterRunnable() Q_DECL_OVERRIDE;

signals:
    void factorChanged();
    void outputStringChanged();
    void rectangleChanged();

    void updateOutputString(QString newOutput);
    void updateRectangle(QPoint position, QSize size, bool visible);

public slots:
    void setOutputString(QString newOutput);
    void setRectangle(QPoint position, QSize size, bool visible);
    void setLandmarkState(bool state);

private:
    qreal m_factor;
    QString m_outputString;

    QPoint m_rectanglePosition;
    QSize m_rectangleSize;
    bool m_rectangleVisible;
};

#endif // VIDEO_FILTER_HPP_
