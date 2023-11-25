#ifndef _REGISTRATION_TIMER_H_
#define _REGISTRATION_TIMER_H_

#include <chrono>
#include <iostream>

class Timer
{
public:
    using Clock = std::chrono::system_clock;

    Timer(std::string &&activity = "stopwatch")
        : activity_(std::move(activity)), start_(Clock::time_point::min()), elapsed_time_(0U) {}

    ~Timer()
    {
        std::cout << activity_ << " elapsed " << getMs() << " (ms)\n";
    }

    void clear()
    {
        elapsed_time_ = 0U;
    }

    void start()
    {
        start_ = Clock::now();
    }

    bool isStarted() const
    {
        return start_.time_since_epoch() != Clock::duration::zero();
    }

    void stop()
    {
        if (isStarted())
        {
            elapsed_time_ += getCurrentMs();
            start_ = Clock::time_point::min();
        }
    }

    unsigned int getMs()
    {
        bool is_started = isStarted();
        if(isStarted())
        {
            stop();
            start();
        }
        return elapsed_time_;
    }

private:
    std::string activity_;
    Clock::time_point start_;
    unsigned int elapsed_time_;

    unsigned int getCurrentMs()
    {
        unsigned int elapsed_ms = static_cast<unsigned int>(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start_).count());
        return elapsed_ms;
    }
};

#endif // _REGISTRATION_TIMER_H_