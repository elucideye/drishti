/*!
  @file   AsyncWorker
  @author David Hirvonen
  @brief  Simple asynchronous worker (1 item job queue)

  \copyright Copyright 2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <mutex>
#include <condition_variable>

#ifndef __AsyncWorker_h__
#define __AsyncWorker_h__

template <typename Callable>
struct AsyncWorker
{
    void loop()
    {
        while (running)
        {
            // Wait until main() sends data:
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [this] { return this->ready; });

            // call the callback/lambda:
            action();
            ready = false;
            processed = true;

            // Manual unlocking is done before notifying, to avoid waking up
            // the waiting thread only to block again (see notify_one for details):
            lock.unlock();
            cv.notify_one();
        }
    }

    void start()
    {
        worker = std::thread([&] { loop(); });
    }

    void stop()
    {
        running = false;
        post([] {}); // post sentinel
    }

    void post(const Callable& callback)
    {
        action = callback;

        // send data to the worker thread:
        {
            std::lock_guard<std::mutex> lock(mutex);
            ready = true;
        }
        cv.notify_one();

        // wait for the worker:
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [this] { return this->processed; });
        }
    }

    // Synchronization {
    std::mutex mutex;
    std::condition_variable cv;
    bool ready = false;
    bool processed = false;
    std::thread worker;
    Callable action;

    bool running = true;
    //  }
};

#endif // __AsyncWorker_h__
