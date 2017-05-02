#ifndef FaceMonitorHCITest_H
#define FaceMonitorHCITest_H 1

#include "drishti/hci/FaceMonitor.h"
#include <mutex>

/*
 * Demonstrate sample callback functionality
 */

class FaceMonitorHCITest : public drishti::hci::FaceMonitor
{
public:
    /*
     * API
     */

    virtual bool isValid(const cv::Point3f& position, const TimePoint& timeStamp)
    {
        std::cout << "Received detection at position: " << position << std::endl;
        return true;
    }

    virtual void grab(const std::vector<FaceImage>& frames, bool isInitialized)
    {
        m_faces = frames;
        m_isInitialized = isInitialized;
        notify();
    }

    /*
     * Custom
     */

    bool isInitialized() const { return m_isInitialized; }

    void wait()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [this] { return m_hasACK; });
    }

    const std::vector<FaceImage>& getFaces() const // synchronous
    {
        return m_faces;
    }

    std::vector<FaceImage>& getFaces() // synchronous
    {
        return m_faces;
    }

    void clear() // synchronous
    {
        m_faces.clear();
        m_hasACK = false;
    }

protected:
    void notify()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_hasACK = true;
        m_cv.notify_one();
    }

    std::mutex m_mutex;
    std::condition_variable m_cv;
    bool m_hasACK = false;
    bool m_isInitialized = false;

    std::vector<FaceImage> m_faces;
};

#endif // FaceMonitorHCITest_H
