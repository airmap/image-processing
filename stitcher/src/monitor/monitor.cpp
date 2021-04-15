#include "airmap/monitor/monitor.h"

namespace airmap {
namespace stitcher {
namespace monitor {

Monitor::Monitor(OperationsEstimator::SharedPtr estimator,
                 std::shared_ptr<airmap::logging::Logger> logger, bool enabled,
                 bool logEnabled)
    : _estimator(estimator)
    , _logger(logger)
    , _enabled(enabled || logEnabled)
    , _logEnabled(logEnabled)
{
}

Monitor::SharedPtr Monitor::create(OperationsEstimator::SharedPtr estimator,
                                   std::shared_ptr<airmap::logging::Logger> logger,
                                   bool enabled, bool logEnabled)
{
    return std::make_shared<Monitor>(estimator, logger, enabled, logEnabled);
}

Monitor::SharedPtr Monitor::create(Estimator::SharedPtr estimator,
                                   std::shared_ptr<airmap::logging::Logger> logger,
                                   bool enabled, bool logEnabled)
{
    OperationsEstimator::SharedPtr _estimator =
            std::dynamic_pointer_cast<OperationsEstimator>(estimator);
    return std::make_shared<Monitor>(_estimator, logger, enabled, logEnabled);
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

    _estimator->changeOperation(operation);

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

void Monitor::disable()
{
    _enabled = false;
    _logEnabled = false;
}

void Monitor::disableLog()
{
    _logEnabled = false;
}

void Monitor::enable()
{
    _enabled = true;
}

void Monitor::enableLog()
{
    _enabled = true;
    _logEnabled = true;
}

OperationsEstimator::SharedPtr Monitor::estimator()
{
    return _estimator;
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

    _logger->log(airmap::logging::Logger::Severity::info,
                 ("Stitch finished in " + totalTime.str()).c_str(), "stitcher");
}

void Monitor::logOperation(const Operation &operation) const
{
    if (!_logEnabled) {
        return;
    }

    _logger->log(airmap::logging::Logger::Severity::info,
                 (operation.previous().str() + " finished in "
                  + std::prev(_operationTimes.end())->second.str())
                         .c_str(),
                 "stitcher");
}

const OperationElapsedTimesMap Monitor::operationTimes() const
{
    return _operationTimes;
}

void Monitor::updateCurrentOperation(double progress)
{
    if (!_enabled) {
        return;
    }

    _estimator->updateCurrentOperation(progress);
}

} // namespace monitor
} // namespace stitcher
} // namespace airmap
