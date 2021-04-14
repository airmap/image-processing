#include "airmap/monitor/estimator.h"

namespace airmap {
namespace stitcher {
namespace monitor {

OperationsEstimator::OperationsEstimator(const std::shared_ptr<Camera> camera,
                             const std::shared_ptr<airmap::logging::Logger> logger,
                             UpdatedCb updatedCb, bool enabled, bool logEnabled)
    : Estimator(logger, updatedCb, enabled, logEnabled)
    , _camera(camera)
    , _currentOperation(Operation::Start())
    , _currentOperationProgress(0.)
{
}

OperationsEstimator::SharedPtr
OperationsEstimator::create(const std::shared_ptr<Camera> camera,
                      const std::shared_ptr<airmap::logging::Logger> logger,
                      UpdatedCb updatedCb, bool enabled, bool logEnabled)
{
    return std::make_shared<OperationsEstimator>(camera, logger, updatedCb, enabled,
                                           logEnabled);
}

void OperationsEstimator::changeOperation(const Operation &operation)
{
    if (!_enabled) {
        return;
    }

    _currentOperationProgress = 0.;
    _currentOperation = operation;

    OperationElapsedTimesMap operationEstimates = estimateOperations();
    _operationEstimates.insert(
            OperationElapsedTimesMoveIter(std::begin(operationEstimates)),
            OperationElapsedTimesMoveIter(std::end(operationEstimates)));

    log();
    updated();
}

const ElapsedTime OperationsEstimator::currentEstimate() const
{
    if (!_enabled) {
        return ElapsedTime::fromSeconds(0);
    }

    // If a parent process has set the estimate, return it directly.
    // For example, AirBoss monitors stdout of the child (stitcher)
    // process for estimates and sets its wrapped stitcher's estimate.
    if (_currentEstimate.get() > ElapsedTime::DurationType { 0 }) {
        return _currentEstimate;
    }

    // No operations have completed.
    if (_operationEstimates.begin() == _operationEstimates.end()) {
        return estimatedTimeTotal();
    }

    return estimatedTimeRemaining() * elapsedToEstimateRatio();
}

double OperationsEstimator::currentProgress() const
{
    if (!_enabled) {
        return 0.;
    }

    // If a parent process has set the progress, return it directly.
    // For example, AirBoss monitors stdout of the child (stitcher)
    // process for progress and sets its wrapped stitcher's progress.
    if (_currentProgress > -1.) {
        return _currentProgress;
    }

    // No operations have completed.
    if (_operationEstimates.begin() == _operationEstimates.end()) {
        return 0.;
    }

    return 100. * (1. - estimatedTimeRemaining() / estimatedTimeTotal());
}

double OperationsEstimator::elapsedToEstimateRatio() const
{
    OperationDoubleMap _elapsedToEstimateRatios = elapsedToEstimateRatios();
    double _elapsedToEstimateRatio { std::accumulate(
            std::begin(_elapsedToEstimateRatios), std::end(_elapsedToEstimateRatios), 0.,
            [this](const double &previous, const OperationDoublePair &current) {
                if (Operation { current.first } < _currentOperation) {
                    return previous + current.second;
                }
                return previous;
            }) };
    _elapsedToEstimateRatio =
        _elapsedToEstimateRatio / _currentOperation.toInt();
    _elapsedToEstimateRatio =
        _elapsedToEstimateRatio > 0. ? _elapsedToEstimateRatio : 1.0;
    return _elapsedToEstimateRatio;
}

const OperationDoubleMap OperationsEstimator::elapsedToEstimateRatios() const
{
    OperationDoubleMap _elapsedToEstimateRatios;
    if (_operationTimesCb) {
        OperationElapsedTimesMap operationTimes = _operationTimesCb();

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
            _elapsedToEstimateRatios.insert(
                    std::make_pair(operationTime.first, elapsedToEstimateRatio));
        }
    } else {
        for (auto &operationEstimate : _operationEstimates) {
            _elapsedToEstimateRatios.insert(std::make_pair(operationEstimate.first, 1.0));
        }
    }

    return _elapsedToEstimateRatios;
}

const ElapsedTime OperationsEstimator::estimatedTimeRemaining() const
{
    return std::accumulate(std::begin(_operationEstimates), std::end(_operationEstimates),
                           ElapsedTime::fromSeconds(0),
                           [this](const ElapsedTime &previous,
                                  const OperationElapsedTimesPair &current) {
                               if (Operation { current.first } >= _currentOperation) {
                                   // We might have a progress percent for the current
                                   // operation. If so, return a fraction of the estimated
                                   // time.
                                   if (Operation { current.first } == _currentOperation) {
                                       return previous + current.second
                                               - (current.second
                                                  * _currentOperationProgress);
                                   };
                                   return previous + current.second;
                               }
                               return previous;
                           });
}

const ElapsedTime OperationsEstimator::estimatedTimeTotal() const
{
    return { std::accumulate(
            std::begin(_operationEstimates), std::end(_operationEstimates),
            ElapsedTime::fromSeconds(0),
            [](const ElapsedTime &previous, const OperationElapsedTimesPair &current) {
                return previous + current.second;
            }) };
}

const OperationElapsedTimesMap OperationsEstimator::estimateOperations() const
{
    OperationElapsedTimesMap operationEstimates;

    if (!_enabled) {
        return operationEstimates;
    }

    if (_camera) {
        if (_camera->distortion_model) {
            PinholeDistortionModel *pinholeDistortionModel =
                dynamic_cast<PinholeDistortionModel *>(
                    _camera->distortion_model.get());
            if (pinholeDistortionModel) {
                operationEstimates.clear();
                operationEstimates.insert(std::make_pair(
                        Operation::Start().value(), ElapsedTime::fromMilliseconds(2500)));
                operationEstimates.insert(
                        std::make_pair(Operation::UndistortImages().value(),
                                       ElapsedTime::fromMilliseconds(400)));
                operationEstimates.insert(
                        std::make_pair(Operation::FindFeatures().value(),
                                       ElapsedTime::fromMilliseconds(500)));
                operationEstimates.insert(
                        std::make_pair(Operation::MatchFeatures().value(),
                                       ElapsedTime::fromMilliseconds(1500)));
                operationEstimates.insert(
                        std::make_pair(Operation::EstimateCameraParameters().value(),
                                       ElapsedTime::fromMilliseconds(100)));
                operationEstimates.insert(
                        std::make_pair(Operation::AdjustCameraParameters().value(),
                                       ElapsedTime::fromSeconds(30)));
                operationEstimates.insert(
                        std::make_pair(Operation::PrepareExposureCompensation().value(),
                                       ElapsedTime::fromMilliseconds(2500)));
                operationEstimates.insert(std::make_pair(Operation::FindSeams().value(),
                                                          ElapsedTime::fromSeconds(70)));
                operationEstimates.insert(std::make_pair(Operation::Compose().value(),
                                                          ElapsedTime::fromSeconds(100)));
                operationEstimates.insert(std::make_pair(Operation::Complete().value(),
                                                          ElapsedTime::fromSeconds(10)));
                return operationEstimates;
            }

            ScaramuzzaDistortionModel *scaramuzzaDistortionModel =
                dynamic_cast<ScaramuzzaDistortionModel *>(
                    _camera->distortion_model.get());
            if (scaramuzzaDistortionModel) {
                operationEstimates.insert(std::make_pair(
                        Operation::Start().value(), ElapsedTime::fromMilliseconds(1500)));
                operationEstimates.insert(
                        std::make_pair(Operation::UndistortImages().value(),
                                       ElapsedTime::fromSeconds(30)));
                operationEstimates.insert(std::make_pair(
                        Operation::FindFeatures().value(), ElapsedTime::fromSeconds(0)));
                operationEstimates.insert(std::make_pair(
                        Operation::MatchFeatures().value(), ElapsedTime::fromSeconds(0)));
                operationEstimates.insert(
                        std::make_pair(Operation::EstimateCameraParameters().value(),
                                       ElapsedTime::fromSeconds(0)));
                operationEstimates.insert(
                        std::make_pair(Operation::AdjustCameraParameters().value(),
                                       ElapsedTime::fromSeconds(30)));
                operationEstimates.insert(
                        std::make_pair(Operation::PrepareExposureCompensation().value(),
                                       ElapsedTime::fromSeconds(10)));
                operationEstimates.insert(std::make_pair(Operation::FindSeams().value(),
                                                          ElapsedTime::fromSeconds(100)));
                operationEstimates.insert(std::make_pair(Operation::Compose().value(),
                                                          ElapsedTime::fromSeconds(100)));
                operationEstimates.insert(std::make_pair(Operation::Complete().value(),
                                                          ElapsedTime::fromSeconds(10)));
                return operationEstimates;
            }
        }
    }

    operationEstimates.insert(std::make_pair(Operation::Start().value(),
                                              ElapsedTime::fromMilliseconds(1500)));
    operationEstimates.insert(std::make_pair(Operation::UndistortImages().value(),
                                              ElapsedTime::fromMilliseconds(400)));
    operationEstimates.insert(std::make_pair(Operation::FindFeatures().value(),
                                              ElapsedTime::fromSeconds(0)));
    operationEstimates.insert(std::make_pair(Operation::MatchFeatures().value(),
                                              ElapsedTime::fromMilliseconds(500)));
    operationEstimates.insert(std::make_pair(
            Operation::EstimateCameraParameters().value(), ElapsedTime::fromSeconds(0)));
    operationEstimates.insert(std::make_pair(Operation::AdjustCameraParameters().value(),
                                              ElapsedTime::fromSeconds(3)));
    operationEstimates.insert(
            std::make_pair(Operation::PrepareExposureCompensation().value(),
                           ElapsedTime::fromMilliseconds(2500)));
    operationEstimates.insert(
            std::make_pair(Operation::FindSeams().value(), ElapsedTime::fromSeconds(70)));
    operationEstimates.insert(
            std::make_pair(Operation::Compose().value(), ElapsedTime::fromSeconds(100)));
    operationEstimates.insert(
            std::make_pair(Operation::Complete().value(), ElapsedTime::fromSeconds(10)));
   
    return operationEstimates;
}

const OperationElapsedTimesMap OperationsEstimator::operationEstimateTimes() const
{
    return _operationEstimates;
}

void OperationsEstimator::setOperationTimesCb(const OperationTimesCb operationTimesCb)
{
    _operationTimesCb = operationTimesCb;
}

void OperationsEstimator::updateCurrentOperation(double progress)
{
    if (!_enabled) {
        return;
    }

    _currentOperationProgress = progress;
    log();
    updated();
}

} // namespace monitor
} // namespace stitcher
} // namespace airmap
