#include "airmap/monitor/timer.h"

namespace airmap {
namespace stitcher {
namespace monitor {

//
//
// ElapsedTime
//
//
ElapsedTime::ElapsedTime()
    : _elapsedTime(DurationType{0})
{
}

ElapsedTime::ElapsedTime(const DurationType elapsedTime)
    : _elapsedTime(elapsedTime)
{
}

ElapsedTime::ElapsedTime(const int64_t elapsedTime)
    : _elapsedTime(DurationType(elapsedTime))
{
}

ElapsedTime::ElapsedTime(const std::string &elapsedTime)
{
    // Parse and throw if invalid.
    ValuesTuple values = _parseString(elapsedTime);

    _elapsedTime = (ElapsedTime::fromHours(std::get<0>(values)) +
                    ElapsedTime::fromMinutes(std::get<1>(values)) +
                    ElapsedTime::fromSeconds(std::get<2>(values)) +
                    ElapsedTime::fromMilliseconds(std::get<3>(values)))
                       .get();
}

ElapsedTime &ElapsedTime::operator=(const ElapsedTime &other)
{
    _elapsedTime = other.get();
    return *this;
}

ElapsedTime &ElapsedTime::operator*=(const double &divisor)
{
    _elapsedTime *= divisor;
    return *this;
}

ElapsedTime &ElapsedTime::operator/=(const double &multiplier)
{
    _elapsedTime /= multiplier;
    return *this;
}

bool ElapsedTime::operator==(const ElapsedTime &other) const
{
    return get() == other.get();
}

bool ElapsedTime::operator!=(const ElapsedTime &other) const
{
    return !(*this == other);
}

const ElapsedTime ElapsedTime::operator+(const ElapsedTime &other) const
{
    return { get() + other.get() };
}

const ElapsedTime ElapsedTime::operator-(const ElapsedTime &other) const
{
    return {get() - other.get()};
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
    return {hours * 60 * 60 * 1000};
}

const ElapsedTime ElapsedTime::fromMinutes(const int64_t &minutes)
{
    return {minutes * 60 * 1000};
}

const ElapsedTime ElapsedTime::fromSeconds(const int64_t &seconds)
{
    return { seconds * 1000 };
}

const ElapsedTime ElapsedTime::fromMilliseconds(const int64_t &milliseconds)
{
    return { milliseconds };
}

const ElapsedTime ElapsedTime::fromValues(const ValuesTuple &values)
{
    return ElapsedTime::fromHours(std::get<0>(values)) +
           ElapsedTime::fromMinutes(std::get<1>(values)) +
           ElapsedTime::fromSeconds(std::get<2>(values)) +
           ElapsedTime::fromMilliseconds(std::get<3>(values));
}

const ElapsedTime::DurationType ElapsedTime::get() const
{
    return _elapsedTime;
}

int64_t ElapsedTime::hours(bool remainderOnly) const
{
    return remainderOnly
               ? std::chrono::duration_cast<Hours>(_elapsedTime).count() % 24
               : std::chrono::duration_cast<Hours>(_elapsedTime).count();
}

int64_t ElapsedTime::minutes(bool remainderOnly) const
{
    return remainderOnly
               ? std::chrono::duration_cast<Minutes>(_elapsedTime).count() % 60
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
    std::string message(size, '\0');
    std::sprintf(&message[0], format.c_str(), hours(), minutes(), seconds(),
                 milliseconds());
    return includeMilliseconds ? message
                               : message.substr(0, message.length() - 4);
}

const ElapsedTime::ValuesTuple
ElapsedTime::_parseString(const std::string &elapsedTime) const
{
    std::invalid_argument invalid(
        "Invalid elapsed time string.  Format is 00:00:00.000");

    if (elapsedTime.length() != 12 || elapsedTime.substr(2, 1) != ":" ||
        elapsedTime.substr(5, 1) != ":" || elapsedTime.substr(8, 1) != ".") {
        throw(invalid);
    }

    try {
        int64_t hours = atoi(elapsedTime.substr(0, 2).c_str());
        int64_t minutes = atoi(elapsedTime.substr(3, 2).c_str());
        int64_t seconds = atoi(elapsedTime.substr(6, 2).c_str());
        int64_t milliseconds = atoi(elapsedTime.substr(9, 3).c_str());
        return std::make_tuple(hours, minutes, seconds, milliseconds);
    } catch (...) {
        throw(invalid);
    }
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
