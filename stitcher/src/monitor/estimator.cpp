#include "airmap/monitor/estimator.h"

namespace airmap {
namespace stitcher {
namespace monitor {

Estimator::Estimator(const boost::optional<Camera> &camera,
                     const std::shared_ptr<Logger> logger, bool enabled,
                     bool logEnabled)
    : _camera(camera)
    , _currentEstimate(0)
    , _currentOperation(Operation::Start())
    , _currentOperationProgressPercent(0.)
    , _enabled(enabled || logEnabled)
    , _logEnabled(logEnabled)
    , _logger(logger)
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

    if (_logEnabled) {
        _logger->log(Logger::Severity::info,
                     (logPrefix + currentEstimate().str()).c_str(), "stitcher");
    }
}

const ElapsedTime Estimator::currentEstimate() const
{
    if (_currentEstimate.milliseconds() > 0) {
        return _currentEstimate;
    }

    if (_operationEstimates.begin() == _operationEstimates.end()) {
        const ElapsedTime estimate{std::accumulate(
            std::begin(_operationEstimates), std::end(_operationEstimates),
            ElapsedTime::fromSeconds(0),
            [](const ElapsedTime previous,
               const std::pair<Operation::Enum, ElapsedTime> &current) {
                return previous + current.second;
            })};
        return estimate;
    }

    const ElapsedTime estimate { std::accumulate(
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
                        };
                        return previous + currentElapsedTime;
                    };
                    return previous + current.second;
                }
                return previous;
            }) };

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
                dynamic_cast<PinholeDistortionModel *>(
                    camera.distortion_model.get());
            if (pinholeDistortionModel) {
                // _operationEstimates = {};
                return;
            }

            ScaramuzzaDistortionModel *scaramuzzaDistortionModel =
                dynamic_cast<ScaramuzzaDistortionModel *>(
                    camera.distortion_model.get());
            if (scaramuzzaDistortionModel) {
                _operationEstimates = {
                    {Operation::Start().value(), ElapsedTime::fromSeconds(5)},
                    {Operation::UndistortImages().value(),
                     ElapsedTime::fromSeconds(30)},
                    {Operation::FindFeatures().value(),
                     ElapsedTime::fromSeconds(0)},
                    {Operation::MatchFeatures().value(),
                     ElapsedTime::fromSeconds(0)},
                    {Operation::EstimateCameraParameters().value(),
                     ElapsedTime::fromSeconds(0)},
                    {Operation::AdjustCameraParameters().value(),
                     ElapsedTime::fromSeconds(30)},
                    {Operation::PrepareExposureCompensation().value(),
                     ElapsedTime::fromSeconds(10)},
                    {Operation::FindSeams().value(),
                     ElapsedTime::fromSeconds(100)},
                    {Operation::Compose().value(),
                     ElapsedTime::fromSeconds(100)},
                    {Operation::Complete().value(),
                     ElapsedTime::fromSeconds(5)},
                };
                return;
            }
        }
    }

    _operationEstimates = {
        {Operation::Start().value(), ElapsedTime::fromSeconds(5)},
        {Operation::UndistortImages().value(), ElapsedTime::fromSeconds(5)},
        {Operation::FindFeatures().value(), ElapsedTime::fromSeconds(5)},
        {Operation::MatchFeatures().value(), ElapsedTime::fromSeconds(5)},
        {Operation::EstimateCameraParameters().value(),
         ElapsedTime::fromSeconds(5)},
        {Operation::AdjustCameraParameters().value(),
         ElapsedTime::fromSeconds(5)},
        {Operation::PrepareExposureCompensation().value(),
         ElapsedTime::fromSeconds(5)},
        {Operation::FindSeams().value(), ElapsedTime::fromSeconds(5)},
        {Operation::Compose().value(), ElapsedTime::fromSeconds(5)},
        {Operation::Complete().value(), ElapsedTime::fromSeconds(5)},
    };
    return;
}

const OperationEstimateTimes Estimator::operationEstimateTimes() const
{
    return _operationEstimates;
}

void Estimator::setCurrentEstimate(const std::string &estimatedTimeRemaining)
{
    _currentEstimate = {estimatedTimeRemaining};
}

void Estimator::updateCurrentOperation(double progressPercent)
{
    _currentOperationProgressPercent = progressPercent;

    if (_logEnabled) {
        _logger->log(Logger::Severity::info,
                     (logPrefix + currentEstimate().str()).c_str(), "stitcher");
    }
}

} // namespace monitor
} // namespace stitcher
} // namespace airmap
