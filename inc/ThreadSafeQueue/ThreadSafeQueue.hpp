#ifndef _THREAD_SAFE_QUEUE_HPP_
#define _THREAD_SAFE_QUEUE_HPP_

#include <queue>
#include <condition_variable>
#include <mutex>

template<typename T>
class ThreadSafeQueue{
public:
    ThreadSafeQueue() = default;
    ThreadSafeQueue(size_t capacity) : capacity_(capacity){};

    void Push(T& new_val);
    void WaitAndPop(T& val);

private:
    mutable std::mutex mut_;

    size_t capacity_;

    std::queue<T> data_queue_;
    std::condition_variable data_cond_;
};

#endif