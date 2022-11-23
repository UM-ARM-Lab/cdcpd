#include "cdcpd/stopwatch.h"

Stopwatch::Stopwatch(std::string const& logger_name)
    : logger_name_(logger_name),
      start_time_(std::chrono::steady_clock::now())
{}

Stopwatch::~Stopwatch()
{
    auto const end_time = std::chrono::steady_clock::now();
    auto const runtime_duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time_);
    double const runtime_ns = static_cast<double>(runtime_duration.count());
    double const runtime_ms = runtime_ns * NANOSECONDS_TO_MILLISECONDS;
    ROS_DEBUG_STREAM_NAMED(logger_name_, "Took " << runtime_ms << " milliseconds to run.");
}