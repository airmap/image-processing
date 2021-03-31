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
    using OperationTimesCb = std::function<OperationTimes()>;
    using UpdatedCb = std::function<void()>;

    Estimator(
            const boost::optional<Camera> &camera, const std::shared_ptr<Logger> logger,
            UpdatedCb updatedCb = []() {}, bool enabled = false, bool logEnabled = false);

    static const std::string logPrefix;

    void changeOperation(const Operation &operation,
                         const OperationTimes &operationTimes);

    const ElapsedTime currentEstimate() const;

    const OperationEstimateTimes operationEstimateTimes() const;

    void setCurrentEstimate(const std::string &estimatedTimeRemaining);

    void setOperationTimesCb(const OperationTimesCb operationTimesCb);

    void updateCurrentOperation(double progressPercent);

private:
    const boost::optional<Camera> &_camera;
    ElapsedTime _currentEstimate;
    Operation _currentOperation;
    double _currentOperationProgressPercent;
    bool _enabled;
    bool _logEnabled;
    const std::shared_ptr<Logger> _logger;
    OperationEstimateTimes _operationEstimates;
    OperationTimesCb _operationTimesCb;
    UpdatedCb _updatedCb;

    void estimateOperations(const OperationTimes &operationTimes);

    void log() const;

    void updated() const;
};

} // namespace monitor
} // namespace stitcher
} // namespace airmap