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
    int milliseconds = atoi(elapsedTime.substr(9, elapsedTime.length() - 9).c_str());

    _elapsedTime = (ElapsedTime::fromHours(hours) + ElapsedTime::fromMinutes(minutes)
                    + ElapsedTime::fromSeconds(seconds)
                    + ElapsedTime::fromMilliseconds(milliseconds))
                           .get();
}

ElapsedTime &ElapsedTime::operator=(const ElapsedTime &other)
{
    _elapsedTime = other.get();
    return *this;
}

const ElapsedTime ElapsedTime::operator+(const ElapsedTime &other) const
{
    return { get() + other.get() };
}

const ElapsedTime ElapsedTime::operator-(const ElapsedTime &other) const
{
    return { get() - other.get() };
    // _elapsedTime = other.get();
    // return *this;
}

const ElapsedTime ElapsedTime::operator*(const double &multiplier) const
{
    return { static_cast<int64_t>(static_cast<double>(milliseconds(false))
                                  * multiplier) };
}

double ElapsedTime::operator/(const ElapsedTime &other) const
{
    return static_cast<double>(milliseconds(false))
            / static_cast<double>(other.milliseconds(false));
}

const ElapsedTime ElapsedTime::operator/(const double &divisor) const
{
    return { static_cast<int64_t>(static_cast<double>(milliseconds(false) / divisor)) };
}

const ElapsedTime ElapsedTime::fromHours(const int64_t &hours)
{
    return { hours * 24 * 60 * 1000 };
}

const ElapsedTime ElapsedTime::fromMinutes(const int64_t &minutes)
{
    return { minutes * 60 * 1000 };
}

const ElapsedTime ElapsedTime::fromSeconds(const int64_t &seconds)
{
    return { seconds * 1000 };
}

const ElapsedTime ElapsedTime::fromMilliseconds(const int64_t &milliseconds)
{
    return { milliseconds };
}

const ElapsedTime::DurationType ElapsedTime::get() const
{
    return _elapsedTime;
}

int64_t ElapsedTime::hours() const
{
    return std::chrono::duration_cast<Hours>(_elapsedTime).count();
}

int64_t ElapsedTime::minutes(bool remainderOnly) const
{
    return remainderOnly ? std::chrono::duration_cast<Minutes>(_elapsedTime).count() % 60
                         : std::chrono::duration_cast<Minutes>(_elapsedTime).count();
}

int64_t ElapsedTime::seconds(bool remainderOnly) const
{
    return remainderOnly ? std::chrono::duration_cast<Seconds>(_elapsedTime).count() % 60
                         : std::chrono::duration_cast<Seconds>(_elapsedTime).count();
}

int64_t ElapsedTime::milliseconds(bool remainderOnly) const
{
    return remainderOnly
            ? std::chrono::duration_cast<Milliseconds>(_elapsedTime).count() % 1000
            : std::chrono::duration_cast<Milliseconds>(_elapsedTime).count();
}

const std::string ElapsedTime::str(bool includeMilliseconds) const
{
    std::string format = "%02d:%02d:%02d.%03d";
    auto size = std::snprintf(nullptr, 0, format.c_str(), hours(), minutes(), seconds(),
                              milliseconds());
    std::string message(size + 1, '\0');
    std::sprintf(&message[0], format.c_str(), hours(), minutes(), seconds(),
                 milliseconds());
    return includeMilliseconds ? message : message.substr(0, message.length() - 3);
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
    return { std::chrono::duration_cast<ElapsedTime::DurationType>(_stop - _start) };
}

std::string Timer::str() const
{
    return elapsed().str();
}

void Timer::start()
{
    _start = steady_clock::now();
}

void Timer::stop()
{
    _stop = steady_clock::now();
}

std::ostream &operator<<(std::ostream &os, const Timer &timer)
{
    return os << timer.str();
}

} // namespace monitor
} // namespace stitcher
} // namespace airmap
