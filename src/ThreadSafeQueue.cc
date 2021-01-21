#include "ThreadSafeQueue/ThreadSafeQueue.hpp"

using std::move;

using std::mutex;
using std::lock_guard;
using std::unique_lock;

using StereoCamera::ThreadSafe::ThreadSafeQueue;

template<typename T>
void ThreadSafeQueue<T>::Push(T& new_val){
    lock_guard<mutex> lock(this->mut_);
    while (this->data_queue_.size() >= this->capacity_){
        this->data_queue_.pop();
    }
    this->data_queue_.push(new_val);
    this->data_cond_.notify_one();
}

template<typename T>
void ThreadSafeQueue<T>::WaitAndPop(T& val){
    unique_lock<mutex> lock(this->mut_);
    data_cond_.wait(lock, [this]{
        return !this->data_queue_.empty();
    });
    val = move(this->data_queue_.front());
    this->data_queue_.pop();
}
