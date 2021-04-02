#pragma once

#include <boost/optional.hpp>

#include "airmap/camera.h"
#include "airmap/logging.h"
#include "airmap/monitor/operation.h"
#include "airmap/monitor/timer.h"

using Logger = airmap::logging::Logger;

namespace airmap {
namespace stitcher {
namespace monitor {

/**
 * @brief Estimator
 */
class Estimator {
public:
    using OperationTimesCb = std::function<OperationElapsedTimesMap()>;
    using UpdatedCb = std::function<void()>;

    Estimator(
        const boost::optional<Camera> camera,
        const std::shared_ptr<Logger> logger, UpdatedCb updatedCb = []() {},
        bool enabled = false, bool logEnabled = false);

    static const std::string estimateLogPrefix;

    static const std::string progressLogPrefix;

    void changeOperation(const Operation &operation);

    const ElapsedTime currentEstimate() const;

    double currentProgress() const;

    void disable();

    void disableLog();

    // Sum and average the ratios of completed operations.
    double elapsedToEstimateRatio() const;

    // Calculate the ratio of elapsed time to estimated time for each operation.
    const OperationDoubleMap elapsedToEstimateRatios() const;

    void enable();

    void enableLog();

    // Sum all remaining estimates and multiply by the elasped:estimate ratio of completed
    // operations.
    const ElapsedTime estimatedTimeRemaining() const;

    // Sum all estimated operation times.
    const ElapsedTime estimatedTimeTotal() const;

    const OperationElapsedTimesMap estimateOperations() const;

    const OperationElapsedTimesMap operationEstimateTimes() const;

    void setCurrentEstimate(const std::string &estimatedTimeRemaining);

    void setCurrentProgress(const std::string &progress);

    void setOperationTimesCb(const OperationTimesCb operationTimesCb);

    void updateCurrentOperation(double progress);

private:
    const boost::optional<Camera> _camera;
    ElapsedTime _currentEstimate;
    Operation _currentOperation;
    double _currentOperationProgress;
    double _currentProgress;
    bool _enabled;
    bool _logEnabled;
    const std::shared_ptr<Logger> _logger;
    OperationElapsedTimesMap _operationEstimates;
    OperationTimesCb _operationTimesCb;
    UpdatedCb _updatedCb;

    void log() const;

    void updated() const;
};

} // namespace monitor
} // namespace stitcher
} // namespace airmap
