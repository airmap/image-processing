#include "airmap/monitor/timer.h"

namespace airmap {
namespace stitcher {
namespace monitor {

//
//
// ElapsedTime
//
//
ElapsedTime::ElapsedTime(const ElapsedTime::DurationType elapsedTime)
    : _elapsedTime(elapsedTime)
{
}

ElapsedTime::ElapsedTime(const int64_t elapsedTime)
    : _elapsedTime(DurationType(elapsedTime))
{
}

ElapsedTime::ElapsedTime(const std::string &elapsedTime)
{
    // TODO(bkd): asserts?
    int hours = atoi(elapsedTime.substr(0, 2).c_str());
    int minutes = atoi(elapsedTime.substr(3, 2).c_str());
    int seconds = atoi(elapsedTime.substr(6, 2).c_str());
    int milliseconds = atoi(elapsedTime.substr(9, 3).c_str());

    _elapsedTime =
        (ElapsedTime::fromHours(hours) + ElapsedTime::fromMinutes(minutes) +
         ElapsedTime::fromSeconds(seconds) +
         ElapsedTime::fromMilliseconds(milliseconds))
            .get();
}

ElapsedTime &ElapsedTime::operator=(const ElapsedTime &other)
{
    _elapsedTime = other.get();
    return *this;
}

const ElapsedTime ElapsedTime::operator+(const ElapsedTime &other) const
{
    return {get() + other.get()};
}

const ElapsedTime ElapsedTime::fromHours(const int64_t &hours)
{
    return {hours * 24 * 60 * 1000};
}

const ElapsedTime ElapsedTime::fromMinutes(const int64_t &minutes)
{
    return {minutes * 60 * 1000};
}

const ElapsedTime ElapsedTime::fromSeconds(const int64_t &seconds)
{
    return {seconds * 1000};
}

const ElapsedTime ElapsedTime::fromMilliseconds(const int64_t &milliseconds)
{
    return {milliseconds};
}

const ElapsedTime::DurationType ElapsedTime::get() const
{
    return _elapsedTime;
}

const int64_t ElapsedTime::hours() const
{
    return std::chrono::duration_cast<Hours>(_elapsedTime).count();
}

const int64_t ElapsedTime::minutes() const
{
    return std::chrono::duration_cast<Minutes>(_elapsedTime).count() % 60;
}

const int64_t ElapsedTime::seconds() const
{
    return std::chrono::duration_cast<Seconds>(_elapsedTime).count() % 60;
}

const int64_t ElapsedTime::milliseconds() const
{
    return std::chrono::duration_cast<Milliseconds>(_elapsedTime).count() %
           1000;
}

const std::string ElapsedTime::str() const
{
    const char *format = "%02d:%02d:%02d.%02d";
    auto size = std::snprintf(nullptr, 0, format, hours(), minutes(), seconds(),
                              milliseconds());
    std::string message(size + 1, '\0');
    std::sprintf(&message[0], format, hours(), minutes(), seconds(),
                 milliseconds());
    return message;
}

std::ostream &operator<<(std::ostream &os, const ElapsedTime &elapsedTime)
{
    return os << elapsedTime.str();
}

//
//
// Timer
//
//
const ElapsedTime Timer::elapsed() const
{
    return {
        std::chrono::duration_cast<ElapsedTime::DurationType>(_stop - _start)};
}

std::string Timer::str() const { return elapsed().str(); }

void Timer::start() { _start = steady_clock::now(); }

void Timer::stop() { _stop = steady_clock::now(); }

std::ostream &operator<<(std::ostream &os, const Timer &timer)
{
    return os << timer;
}

} // namespace monitor
} // namespace stitcher
} // namespace airmap
