/*! -*-c++-*-
  @file   QtFaceMonitor.cpp
  @author David Hirvonen
  @brief Qt sample facemonitor 

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "QtFaceMonitor.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // for cvtColor
#include <opencv2/highgui/highgui.hpp>

#include <QtGlobal> // for Q_OS_*

#include <vector>
#include <iostream>

// http://stackoverflow.com/a/26184296
//template <typename T, typename U>
//auto DurationCast(U const& u) -> decltype(std::chrono::duration_cast<T>(u))
//{
//    return std::chrono::duration_cast<T>(u);
//}

double QtFaceMonitor::PositionAndTime::estimateVelocity(const PositionAndTime& current)
{
    double translation = cv::norm(current.position - position);
    double interval = std::chrono::duration_cast<std::chrono::duration<double>>(current.time - time).count();
    return translation / (interval + std::numeric_limits<double>::epsilon());
}

QtFaceMonitor::QtFaceMonitor(const cv::Vec2d& range, std::shared_ptr<tp::ThreadPool<>>& threads)
    : m_range(range)
    , m_frameCounter(0)
    , m_stackCounter(0)
    , m_threads(threads)
{
}

bool QtFaceMonitor::isValid(const cv::Point3f& position, const TimePoint& now)
{
    bool okay = false;

    // Here we can apply various heuristics
    m_frameCounter++;
    PositionAndTime current(position, now);

    if (m_previousPosition.has)
    {
        const double velocity = m_previousPosition->estimateVelocity(current);

        if ((m_range[0] < position.z) && (position.z < m_range[1]))
        {
            if (velocity < m_velocityTreshold)
            {
                double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - m_previousStack->time).count();
                if (elapsed > m_stackSampleInterval)
                {
                    okay = true;
                    m_previousStack = current;
                }
            }
        }
    }

    m_previousPosition = current;

    return okay;
}

void QtFaceMonitor::grab(const std::vector<FaceImage>& frames, bool isInitialized)
{
    std::string tmp;
#if defined(Q_OS_IOS)
    tmp = std::string(getenv("HOME")) + "/Documents";
#elif defined(Q_OS_ANDROID)
    tmp = "/data/local/tmp";
#else
    tmp = "/tmp";
#endif

    auto counter = m_stackCounter++;
    std::function<void()> logger = [tmp, frames, counter, this]() {
        std::vector<cv::Mat> faces, eyes;

        for (int i = 0; i < frames.size(); i++)
        {
            faces.push_back(frames[i].image.image);
            if (!frames[i].eyes.image.empty())
            {
                eyes.push_back(frames[i].eyes.image);
            }
        }

        if (faces.size())
        {
            cv::Mat stack;
            hconcat(faces, stack);

            std::stringstream ss;
            ss << tmp << "/frame_" << counter << ".png";
            cv::imwrite(ss.str(), stack);
        }

        if (eyes.size())
        {
            cv::Mat stack;
            cv::vconcat(eyes, stack);

            std::stringstream ss;
            ss << tmp << "/eyes_" << counter << ".png";
            cv::imwrite(ss.str(), stack);
        }

        if (frames.size() && !frames[0].filtered.image.empty())
        {
            cv::Mat bgr;
            cv::cvtColor(frames[0].filtered.image, bgr, cv::COLOR_BGRA2BGR);

            std::stringstream ss;
            ss << tmp << "/eye_mean_" << counter << ".png";
            cv::imwrite(ss.str(), bgr);
        }

    };

    if (m_threads)
    {
        m_threads->process(logger);
    }
    else
    {
        logger();
    }
}
