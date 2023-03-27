#pragma once

#include <chrono>
#include <string>

#include <ros/ros.h>

static double const NANOSECONDS_TO_MILLISECONDS = 1e-6;

// "Context manager" for timing and printing runtime to ROS debug logger.
class Stopwatch
{
public:
    Stopwatch(std::string const& routine_name);
    ~Stopwatch();

private:
    std::string const routine_name_;
    std::chrono::steady_clock::time_point const start_time_;
};