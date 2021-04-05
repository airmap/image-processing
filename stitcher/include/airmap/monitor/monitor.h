#pragma once

#include <chrono>
#include <map>
#include <string>

#include "airmap/logging.h"
#include "airmap/monitor/estimator.h"
#include "airmap/monitor/operation.h"
#include "airmap/monitor/timer.h"

using Logger = airmap::logging::Logger;

namespace airmap {
namespace stitcher {
namespace monitor {

/**
 * @brief Monitor
 * Manages operation timer and estimates.
 */
class Monitor {
public:
    using SharedPtr = std::shared_ptr<Monitor>;

    Monitor(Estimator &estimator, std::shared_ptr<Logger> logger,
            bool enabled = false, bool logEnabled = false);

    /**
     * @brief changeOperation
     * Change the current operation.
     * @param operation The current operation.
     */
    void changeOperation(const Operation &operation);

    /**
     * @brief currentOperation
     * Returns the current operation.
     */
    Operation currentOperation();

    void disable();

    void disableLog();

    void enable();

    void enableLog();

    /**
     * @brief operationTimes
     * Returns a map of elapsed times for each operation.
     */
    const OperationElapsedTimesMap operationTimes() const;

    void updateCurrentOperation(double progressPercent);

private:
    Estimator &_estimator;
    OperationElapsedTimesMap _operationTimes;
    std::shared_ptr<Logger> _logger;
    bool _enabled;
    bool _logEnabled;
    Timer _timer;

    void logComplete() const;

    void logOperation(const Operation &operation) const;
};

} // namespace monitor
} // namespace stitcher
} // namespace airmap
