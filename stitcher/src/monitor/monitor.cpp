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

void Monitor::changeOperation(Operation operation)
{
    if (!_enabled) {
        return;
    }

    if (operation.toInt() > 0) {
        _timer.stop();
        _operationTimes.insert(
            std::make_pair(operation.previous().value(), _timer.elapsed()));

        if (_logEnabled) {
            _logger->log(Logger::Severity::info,
                         (operation.previous().str() + " finished in " +
                          std::prev(_operationTimes.end())->second.str())
                             .c_str(),
                         "stitcher");
        }
    }

    _estimator.changeOperation(operation, _operationTimes);
    _timer.start();
}

Operation Monitor::currentOperation()
{
    return std::prev(_operationTimes.end())->first;
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
