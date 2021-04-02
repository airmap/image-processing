#pragma once

#include <chrono>
#include <ostream>
#include <tuple>
#include <vector>

#include <iostream>

namespace airmap {
namespace stitcher {
namespace monitor {

class ElapsedTime {
public:
    using Hours = std::chrono::hours;
    using Minutes = std::chrono::minutes;
    using Seconds = std::chrono::seconds;
    using Milliseconds = std::chrono::milliseconds;
    using DurationType = Milliseconds;
    using ValuesTuple = std::tuple<ino64_t, ino64_t, ino64_t, ino64_t>;

    ElapsedTime();

    ElapsedTime(const DurationType elapsedTime);

    ElapsedTime(const int64_t elapsedTime);

    ElapsedTime(const std::string &elapsedTime);

    ElapsedTime &operator=(const ElapsedTime &other);

    ElapsedTime &operator*=(const double &multiplier);

    ElapsedTime &operator/=(const double &divisor);

    bool operator==(const ElapsedTime &other) const;

    bool operator!=(const ElapsedTime &other) const;

    const ElapsedTime operator+(const ElapsedTime &other) const;

    const ElapsedTime operator-(const ElapsedTime &other) const;

    const ElapsedTime operator*(const double &multiplier) const;

    double operator/(const ElapsedTime &other) const;

    const ElapsedTime operator/(const double &divisor) const;

    static const ElapsedTime fromHours(const int64_t &hours);

    static const ElapsedTime fromMinutes(const int64_t &minutes);

    static const ElapsedTime fromSeconds(const int64_t &seconds);

    static const ElapsedTime fromMilliseconds(const int64_t &milliseconds);

    static const ElapsedTime fromValues(const ValuesTuple &values);

    const DurationType get() const;

    int64_t hours(bool remainderOnly = true) const;

    int64_t minutes(bool remainderOnly = true) const;

    int64_t seconds(bool remainderOnly = true) const;

    int64_t milliseconds(bool remainderOnly = true) const;

    const std::string str(bool includeMilliseconds = true) const;

private:
    DurationType _elapsedTime;

    const ValuesTuple _parseString(const std::string &elapsedTime) const;
};

/**
 * @brief operator<<
 * Representation of ElapsedTime object as stream.
 * @param os ostream reference.
 * @param elapsedTime ElapsedTime reference
 */
std::ostream &operator<<(std::ostream &os, const ElapsedTime &elapsedTime);

/**
 * @brief Timer
 * A simple class to time operations.
 */
class Timer {
public:
    using steady_clock = std::chrono::steady_clock;
    using time_point = steady_clock::time_point;

    /**
     * @brief elapsed
     * Return the elapsed time.
     */
    const ElapsedTime elapsed() const;

    /**
     * @brief start
     * Start the timer.
     */
    void start();

    /**
     * @brief stop
     * Stop the timer.
     */
    void stop();

    /**
     * @brief str
     * Logs the elapsed time.
     * @param prefix The string prefix to prepend to the log statement.
     */
    std::string str() const;

private:
    time_point _start;
    time_point _stop;
};

/**
 * @brief operator<<
 * Representation of timer object as stream.
 * @param os ostream reference.
 * @param timer Timer reference.
 */
std::ostream &operator<<(std::ostream &os, const Timer &timer);

} // namespace monitor
} // namespace stitcher
} // namespace airmap
