#pragma once

#include <chrono>
#include <map>
#include <string>

#include "airmap/logging.h"
#include "airmap/monitor/estimator.h"
#include "airmap/monitor/operation.h"
#include "airmap/monitor/timer.h"

namespace airmap {
namespace stitcher {
namespace monitor {

/**
 * @brief Monitor
 * Manages operation timer, estimates, and progress.
 */
class Monitor {
public:
    using SharedPtr = std::shared_ptr<Monitor>;

    Monitor(OperationsEstimator::SharedPtr estimator,
            std::shared_ptr<airmap::logging::Logger> logger, bool enabled = false,
            bool logEnabled = false);

    static Monitor::SharedPtr create(OperationsEstimator::SharedPtr estimator,
                                     std::shared_ptr<airmap::logging::Logger> logger,
                                     bool enabled = false, bool logEnabled = false);

    static Monitor::SharedPtr create(Estimator::SharedPtr estimator,
                                     std::shared_ptr<airmap::logging::Logger> logger,
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

    /**
     * @brief disable
     * Disable monitoring of operation elapsed times.
     */
    void disable();

    /**
     * @brief disableLog
     * Disable logging of operation elapsed times.
     */
    void disableLog();

    /**
     * @brief enable
     * Enables monitoring of operation elapsed times.
     */
    void enable();

    /**
     * @brief enableLog
     * Enables logging of operation elapsed times.
     */
    void enableLog();

    /**
     * @brief estimator
     * Returns a pointer to the estimator.
     */
    OperationsEstimator::SharedPtr estimator();

    /**
     * @brief operationTimes
     * Returns a map of elapsed times for each operation.
     */
    const OperationElapsedTimesMap operationTimes() const;

    /**
     * @brief updateCurrentOperation
     * @param progress Progress of the current operation.  A number
     * between 0 and 1.
     */
    void updateCurrentOperation(double progress);

private:
    /**
     * @brief _estimator
     * A pointer to an instance of an estimator.
     */
    OperationsEstimator::SharedPtr _estimator;

    /**
     * @brief _operationTimes
     * The current elapsed times for each operation.
     */
    OperationElapsedTimesMap _operationTimes;

    /**
     * @brief _logger
     * A pointer to an instance of a logger.
     */
    std::shared_ptr<airmap::logging::Logger> _logger;

    /**
     * @brief _enabled
     * Whether to monitor elapsed times for operations.
     */
    bool _enabled;

    /**
     * @brief _logEnabled
     * Whether to log operation elapsed times.
     */
    bool _logEnabled;

    /**
     * @brief _timer
     * An instance of a timer.  Used to time each operation.
     */
    Timer _timer;

    /**
     * @brief logComplete
     * Logs the total elapsed time of the stitch when complete.
     */
    void logComplete() const;

    /**
     * @brief logOperation
     * Logs the elapsed time for an operation, when complete.
     */
    void logOperation(const Operation &operation) const;
};

} // namespace monitor
} // namespace stitcher
} // namespace airmap
