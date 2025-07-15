#pragma once

#include <thread>
#include <functional>
#include <future>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

class TaskWorker {
public:
    TaskWorker();
    ~TaskWorker();

    // Submits a task to be run on the worker thread.
    // Returns a future that will hold the task's result.
    template <class F, class... Args>
    auto enqueue(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type>;

    TaskWorker(const TaskWorker&) = delete;
    TaskWorker& operator=(const TaskWorker&) = delete;

private:
    void run();

    std::thread worker_thread_;
    std::queue<std::function<void()>> tasks_;
    
    std::mutex queue_mutex_;
    std::condition_variable condition_;
    std::atomic<bool> stop_;
};

inline TaskWorker::TaskWorker() : stop_(false) {
    worker_thread_ = std::thread(&TaskWorker::run, this);
}

inline TaskWorker::~TaskWorker() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        stop_ = true;
    }
    condition_.notify_one();

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

template <class F, class... Args>
auto TaskWorker::enqueue(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type> {
    
    using return_type = typename std::result_of<F(Args...)>::type;

    // Create a packaged_task to wrap the function and connect it to a future
    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (stop_) {
            throw std::runtime_error("enqueue on stopped TaskWorker");
        }
        // Push a lambda that will execute the packaged_task
        tasks_.emplace([task]() { (*task)(); });
    }
    condition_.notify_one();
    return res;
}

inline void TaskWorker::run() {
    while (true) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            // Wait until there's a task or the stop signal is received
            condition_.wait(lock, [this] {
                return stop_ || !tasks_.empty();
            });

            // If stopped and no more tasks, exit the loop
            if (stop_ && tasks_.empty()) {
                return;
            }

            task = std::move(tasks_.front());
            tasks_.pop();
        }
        task(); // Execute the task
    }
}
