#pragma once

#include "airmap/monitor/timer.h"

namespace airmap {
namespace stitcher {
namespace monitor {

class Operation {
public:
    enum class Enum {
        Start,
        UndistortImages,
        FindFeatures,
        MatchFeatures,
        EstimateCameraParameters,
        AdjustCameraParameters,
        PrepareExposureCompensation,
        FindSeams,
        Compose,
        Complete
    };

    Operation(const Enum value)
        : _value(value)
    {
    }

    Operation &operator=(const Operation &other)
    {
        _value = other.value();
        return *this;
    }

    bool operator==(const Operation &other) const
    {
        return _value == other.value();
    }

    bool operator!=(const Operation &other) const { return !(*this == other); }

    bool operator<(const Operation &other) const { return toInt() < other.toInt(); }

    bool operator>(const Operation &other) const { return other < *this; }

    bool operator<=(const Operation &other) const { return !(*this > other); }

    bool operator>=(const Operation &other) const { return !(*this < other); }

    static constexpr int count = 10;

    static const Operation Start() { return { Enum::Start }; }

    static const Operation UndistortImages() { return { Enum::UndistortImages }; }

    static const Operation FindFeatures() { return { Enum::FindFeatures }; }

    static const Operation MatchFeatures() { return { Enum::MatchFeatures }; }

    static const Operation EstimateCameraParameters()
    {
        return { Enum::EstimateCameraParameters };
    }

    static const Operation AdjustCameraParameters()
    {
        return {Enum::AdjustCameraParameters};
    }

    static const Operation PrepareExposureCompensation()
    {
        return {Enum::PrepareExposureCompensation};
    }

    static const Operation FindSeams() { return {Enum::FindSeams}; }

    static const Operation Compose() { return {Enum::Compose}; }

    static const Operation Complete() { return {Enum::Complete}; }

    const Operation previous() const
    {
        int value = toInt();
        value = value >= 0 ? value - 1 : count;
        return {static_cast<Operation::Enum>(value)};
    }

    const Operation next() const
    {
        return {static_cast<Operation::Enum>((toInt() + 1) % count)};
    }

    const std::string str() const
    {
        switch (_value) {
        case Enum::Start:
            return "Start";
        case Enum::UndistortImages:
            return "UndistortImages";
        case Enum::FindFeatures:
            return "FindFeatures";
        case Enum::MatchFeatures:
            return "MatchFeatures";
        case Enum::EstimateCameraParameters:
            return "EstimateCameraParameters";
        case Enum::AdjustCameraParameters:
            return "AdjustCameraParameters";
        case Enum::PrepareExposureCompensation:
            return "PrepareExposureCompensation";
        case Enum::FindSeams:
            return "FindSeams";
        case Enum::Compose:
            return "Compose";
        case Enum::Complete:
            return "Complete";
        default:
            return "Start";
        }
    }

    int toInt() const { return static_cast<int>(_value); }

    Enum value() const { return _value; }

private:
    Enum _value;
};

using OperationDoubleMap = std::map<Operation::Enum, const double>;
using OperationDoublePair = std::pair<Operation::Enum, const double>;
using OperationElapsedTimesMap = std::map<Operation::Enum, const ElapsedTime>;
using OperationElapsedTimesPair = std::pair<Operation::Enum, const ElapsedTime>;

} // namespace monitor
} // namespace stitcher
} // namespace airmap
