#include "airmap/monitor/estimator.h"

namespace airmap {
namespace stitcher {
namespace monitor {

Estimator::Estimator(const boost::optional<Camera> &camera,
                     const std::shared_ptr<Logger> logger, UpdatedCb updatedCb,
                     bool enabled, bool logEnabled)
    : _camera(camera)
    , _currentEstimate(0)
    , _currentOperation(Operation::Start())
    , _currentOperationProgressPercent(0.)
    , _enabled(enabled || logEnabled)
    , _logEnabled(logEnabled)
    , _logger(logger)
    , _updatedCb(updatedCb)
{
}

const std::string Estimator::logPrefix = "Estimated time remaining: ";

void Estimator::changeOperation(const Operation &operation,
                                const OperationTimes &operationTimes)
{
    if (!_enabled) {
        return;
    }

    _currentOperationProgressPercent = 0.;
    _currentOperation = operation;
    estimateOperations(operationTimes);
    log();
    updated();
}

const ElapsedTime Estimator::currentEstimate() const
{
    // If a parent process has set the estimate, return it directly.
    // For example, AirBoss monitors logs of the child process
    // for estimates and sets its wrapped stitcher's estimate.
    if (_currentEstimate.get() > ElapsedTime::DurationType { 0 }) {
        return _currentEstimate;
    }

    // No operations have completed.  Sum estimates for all operations.
    if (_operationEstimates.begin() == _operationEstimates.end()) {
        const ElapsedTime estimate { std::accumulate(
                std::begin(_operationEstimates), std::end(_operationEstimates),
                ElapsedTime::fromSeconds(0),
                [](const ElapsedTime previous,
                   const std::pair<Operation::Enum, ElapsedTime> &current) {
                    return previous + current.second;
                }) };
        return estimate;
    }

    // Calculate the ratio of elapsed time to estimated time for each operation.
    std::map<Operation::Enum, double> elapsedToEstimateRatios;
    if (_operationTimesCb) {
        OperationTimes operationTimes = _operationTimesCb();

        for (auto &operationTime : operationTimes) {
            ElapsedTime estimate = _operationEstimates.find(operationTime.first)
                            != _operationEstimates.end()
                    ? _operationEstimates.at(operationTime.first)
                    : operationTime.second;

            int64_t operationTimeMs = operationTime.second.milliseconds(false);
            int64_t estimateMs = estimate.milliseconds(false);
            double elapsedToEstimateRatio = (operationTimeMs > 0 && estimateMs > 0)
                    ? static_cast<double>(operationTimeMs)
                            / static_cast<double>(estimateMs)
                    : 1.;
            elapsedToEstimateRatio =
                    elapsedToEstimateRatio > 0 ? elapsedToEstimateRatio : 1.;
            elapsedToEstimateRatios.insert(
                    std::make_pair(operationTime.first, elapsedToEstimateRatio));
        }
    } else {
        for (auto &operationEstimate : _operationEstimates) {
            elapsedToEstimateRatios.insert(std::make_pair(operationEstimate.first, 1.0));
        }
    }

    // Sum and average the ratios.
    double elapsedToEstimateRatio { std::accumulate(
            std::begin(elapsedToEstimateRatios), std::end(elapsedToEstimateRatios), 0.,
            [this](const double previous,
                   const std::pair<Operation::Enum, double> &current) {
                if (static_cast<int>(current.first) < _currentOperation.toInt()) {
                    return previous + current.second;
                }
                return previous;
            }) };
    elapsedToEstimateRatio = elapsedToEstimateRatio / elapsedToEstimateRatios.size();
    elapsedToEstimateRatio = elapsedToEstimateRatio > 0. ? elapsedToEstimateRatio : 1.0;

    // Sum all remaining estimates and multiply by the elasped to estimate ratio.
    const ElapsedTime estimate {
        std::accumulate(
                std::begin(_operationEstimates), std::end(_operationEstimates),
                ElapsedTime::fromSeconds(0),
                [this](const ElapsedTime previous,
                       const std::pair<Operation::Enum, ElapsedTime> &current) {
                    if (static_cast<int>(current.first) >= _currentOperation.toInt()) {
                        // We might have a progress percent for the current operation.
                        // If so, return a fraction of the estimated time.
                        if (current.first == _currentOperation.value()) {
                            ElapsedTime currentElapsedTime {
                                std::chrono::duration_cast<ElapsedTime::DurationType>(
                                        current.second.get()
                                        - (current.second.get()
                                           * _currentOperationProgressPercent))
                                        .count()
                            };
                            return previous + currentElapsedTime;
                        };
                        return previous + current.second;
                    }
                    return previous;
                })
        * elapsedToEstimateRatio
    };

    return estimate;
}

void Estimator::estimateOperations(const OperationTimes &operationTimes)
{
    if (!_enabled) {
        return;
    }

    if (_camera.has_value()) {
        Camera camera = _camera.get();

        if (camera.distortion_model) {
            PinholeDistortionModel *pinholeDistortionModel =
                    dynamic_cast<PinholeDistortionModel *>(camera.distortion_model.get());
            if (pinholeDistortionModel) {
                _operationEstimates = {
                    { Operation::Start().value(), ElapsedTime::fromMilliseconds(2500) },
                    { Operation::UndistortImages().value(),
                      ElapsedTime::fromMilliseconds(400) },
                    { Operation::FindFeatures().value(),
                      ElapsedTime::fromMilliseconds(500) },
                    { Operation::MatchFeatures().value(),
                      ElapsedTime::fromMilliseconds(1500) },
                    { Operation::EstimateCameraParameters().value(),
                      ElapsedTime::fromSeconds(0) },
                    { Operation::AdjustCameraParameters().value(),
                      ElapsedTime::fromSeconds(3) },
                    { Operation::PrepareExposureCompensation().value(),
                      ElapsedTime::fromMilliseconds(2500) },
                    { Operation::FindSeams().value(), ElapsedTime::fromSeconds(70) },
                    { Operation::Compose().value(), ElapsedTime::fromSeconds(100) },
                    { Operation::Complete().value(), ElapsedTime::fromSeconds(10) }
                };
                return;
            }

            ScaramuzzaDistortionModel *scaramuzzaDistortionModel =
                    dynamic_cast<ScaramuzzaDistortionModel *>(
                            camera.distortion_model.get());
            if (scaramuzzaDistortionModel) {
                _operationEstimates = {
                    { Operation::Start().value(), ElapsedTime::fromMilliseconds(1500) },
                    { Operation::UndistortImages().value(),
                      ElapsedTime::fromSeconds(30) },
                    { Operation::FindFeatures().value(), ElapsedTime::fromSeconds(0) },
                    { Operation::MatchFeatures().value(), ElapsedTime::fromSeconds(0) },
                    { Operation::EstimateCameraParameters().value(),
                      ElapsedTime::fromSeconds(0) },
                    { Operation::AdjustCameraParameters().value(),
                      ElapsedTime::fromSeconds(30) },
                    { Operation::PrepareExposureCompensation().value(),
                      ElapsedTime::fromSeconds(10) },
                    { Operation::FindSeams().value(), ElapsedTime::fromSeconds(100) },
                    { Operation::Compose().value(), ElapsedTime::fromSeconds(100) },
                    { Operation::Complete().value(), ElapsedTime::fromSeconds(10) }
                };
                return;
            }
        }
    }

    _operationEstimates = {
        { Operation::Start().value(), ElapsedTime::fromMilliseconds(1500) },
        { Operation::UndistortImages().value(), ElapsedTime::fromMilliseconds(400) },
        { Operation::FindFeatures().value(), ElapsedTime::fromSeconds(0) },
        { Operation::MatchFeatures().value(), ElapsedTime::fromMilliseconds(500) },
        { Operation::EstimateCameraParameters().value(), ElapsedTime::fromSeconds(0) },
        { Operation::AdjustCameraParameters().value(), ElapsedTime::fromSeconds(3) },
        { Operation::PrepareExposureCompensation().value(),
          ElapsedTime::fromMilliseconds(2500) },
        { Operation::FindSeams().value(), ElapsedTime::fromSeconds(70) },
        { Operation::Compose().value(), ElapsedTime::fromSeconds(100) },
        { Operation::Complete().value(), ElapsedTime::fromSeconds(10) }
    };
    return;
}

void Estimator::log() const
{
    if (_logEnabled) {
        _logger->log(Logger::Severity::info,
                     (logPrefix + currentEstimate().str()).c_str(), "stitcher");
    }
}

const OperationEstimateTimes Estimator::operationEstimateTimes() const
{
    return _operationEstimates;
}

void Estimator::setCurrentEstimate(const std::string &estimatedTimeRemaining)
{
    _currentEstimate = { estimatedTimeRemaining };
    updated();
}

void Estimator::setOperationTimesCb(const OperationTimesCb operationTimesCb)
{
    _operationTimesCb = operationTimesCb;
}

void Estimator::updateCurrentOperation(double progressPercent)
{
    _currentOperationProgressPercent = progressPercent;
    log();
    updated();
}

void Estimator::updated() const
{
    _updatedCb();
}

} // namespace monitor
} // namespace stitcher
} // namespace airmap
