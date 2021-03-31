#include "airmap/monitor/monitor.h"

namespace airmap {
namespace stitcher {
namespace monitor {

Monitor::Monitor(Estimator &estimator, std::shared_ptr<Logger> logger,
                 bool enabled, bool logEnabled)
    : _estimator(estimator)
    , _logger(logger)
    , _enabled(enabled || logEnabled)
    , _logEnabled(logEnabled)
{
}

void Monitor::changeOperation(const Operation &operation)
{
    if (!_enabled) {
        return;
    }

    if (operation.toInt() > 0) {
        _timer.stop();
        _operationTimes.insert(
            std::make_pair(operation.previous().value(), _timer.elapsed()));

        logOperation(operation);
    }

    _estimator.changeOperation(operation, _operationTimes);

    if (operation == Operation::Complete()) {
        logComplete();
    } else {
        _timer.start();
    }
}

Operation Monitor::currentOperation()
{
    return std::prev(_operationTimes.end())->first;
}

void Monitor::logComplete() const
{
    if (!_logEnabled) {
        return;
    }

    ElapsedTime totalTime { std::accumulate(
            std::begin(_operationTimes), std::end(_operationTimes),
            ElapsedTime::fromSeconds(0),
            [](const ElapsedTime &previous,
               const std::pair<Operation::Enum, ElapsedTime> &current) {
                return previous + current.second;
            }) };

    _logger->log(Logger::Severity::info,
                 ("Stitch finished in " + totalTime.str()).c_str(), "stitcher");
}

void Monitor::logOperation(const Operation &operation) const
{
    if (!_logEnabled) {
        return;
    }

    _logger->log(Logger::Severity::info,
                 (operation.previous().str() + " finished in "
                  + std::prev(_operationTimes.end())->second.str())
                         .c_str(),
                 "stitcher");
}

const OperationTimes Monitor::operationTimes() const { return _operationTimes; }

void Monitor::updateCurrentOperation(double progressPercent)
{
    if (!_enabled) {
        return;
    }

    _estimator.updateCurrentOperation(progressPercent);
}

} // namespace monitor
} // namespace stitcher
} // namespace airmap
