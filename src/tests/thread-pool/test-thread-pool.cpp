#include "thread_pool/thread_pool.hpp"

#include <iostream>
#include <algorithm> // std::sort

const int FunctionSize = 128;
using FixedThreadPool = tp::ThreadPool<FunctionSize>;

static const size_t CONCURRENCY = (1024);

int main(int argc, char** argv)
{
    FixedThreadPool thread_pool;

    std::cout << "***thread pool cpp***" << std::endl;

    std::vector<std::vector<int>> arrays(CONCURRENCY);
    for (auto& a : arrays)
    {
        a.resize(1024 * 1024);
    }

    std::vector<std::future<int>> waiters(CONCURRENCY);

    for (int i = 0; i < CONCURRENCY; i++)
    {
        waiters[i] = thread_pool.process([i, &arrays]() {
            std::sort(arrays[i].begin(), arrays[i].end(), [](int a, int b) { return a < b; });
            return 0;
        });
    }

    std::cout << "All jobs posted!" << std::endl;
    for (auto& waiter : waiters)
    {
        waiter.get();
        //waiter.get_future().wait();
    }
}
