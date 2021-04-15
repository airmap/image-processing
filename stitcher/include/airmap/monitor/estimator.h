#pragma once

#include "airmap/camera.h"
#include "airmap/logging.h"
#include "airmap/monitor/operation.h"
#include "airmap/monitor/timer.h"

namespace airmap {
namespace stitcher {
namespace monitor {

/**
 * @brief Estimator
 * Manages estimation of stitch time remaining.
 */
class Estimator
{
public:
    using SharedPtr = std::shared_ptr<Estimator>;
    using UpdatedCb = std::function<void()>;

    Estimator(
            const std::shared_ptr<airmap::logging::Logger> logger,
            UpdatedCb updatedCb = []() {}, bool enabled = false, bool logEnabled = false)
        : _currentEstimate(0)
        , _currentProgress(-1.)
        , _enabled(enabled)
        , _logEnabled(logEnabled)
        , _logger(logger)
        , _updatedCb(updatedCb)
    {
    }

    virtual ~Estimator() { }

    static Estimator::SharedPtr create(
            const std::shared_ptr<airmap::logging::Logger> logger,
            UpdatedCb updatedCb = []() {}, bool enabled = false, bool logEnabled = false)
    {
        return std::make_shared<Estimator>(logger, updatedCb, enabled, logEnabled);
    }

    /**
     * @brief estimateLogPrefix
     * Prefix for estimate log statements.
     * This can be used by a parent process to scrape the logs
     * for estimate updates.
     */
    const std::string estimateLogPrefix = "Estimated time remaining: ";

    /**
     * @brief progressLogPrefix
     * Prefix for progress log statements.
     * This can be used by a parent process to scrape the logs
     * for progress updates.
     */
    const std::string progressLogPrefix = "Progress: ";

    /**
     * @brief currentEstimate
     * Calculates and returns the current estimate of time
     * remaining.
     */
    virtual const ElapsedTime currentEstimate() const
    {
        _logger->log(airmap::logging::Logger::Severity::info, "Estimator::currentEstimate", "stitcher");
        if (!_enabled) {
            return ElapsedTime::fromSeconds(0);
        }
        _logger->log(airmap::logging::Logger::Severity::info, "Estimator::currentEstimate woo", "stitcher");

        // If a parent process has set the estimate, return it directly.
        // For example, AirBoss monitors stdout of the child (stitcher)
        // process for estimates and sets its wrapped stitcher's estimate.
        if (_currentEstimate.get() > ElapsedTime::DurationType { 0 }) {
            return _currentEstimate;
        }

        return ElapsedTime::fromSeconds(0);
    }

    /**
     * @brief currentProgress
     * Calculates and returns the current progress
     * as a number between 0 and 100.
     */
    virtual double currentProgress() const
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

        return 0.;
    }

    /**
     * @brief disable
     * Disable the estimator.
     */
    void disable()
    {
        _enabled = false;
        _logEnabled = false;
    }

    /**
     * @brief disableLog
     *Disable logging of estimate and progress updates.
     */
    void disableLog() { _logEnabled = false; }

    /**
     * @brief enable
     * Enable the estimator.
     */
    void enable() { _enabled = true; }

    /**
     * @brief enableLog
     * Enable logging of estimate and progress updates.
     */
    void enableLog()
    {
        _enabled = true;
        _logEnabled = true;
    }

    /**
     * @brief setCurrentEstimate
     * Sets the current estimate value.
     * This is useful if the instance of the stitcher is managing
     * a child process.  In that case, the parent stitcher is scraping
     * the logs of the child process for estimate updates and setting
     * its estimate to that value.
     */
    void setCurrentEstimate(const std::string &estimatedTimeRemaining)
    {
        if (!_enabled) {
            return;
        }

        _currentEstimate = { estimatedTimeRemaining };
        updated();
    }

    /**
     * @brief setCurrentProgress
     * Sets the current progress value.  A number between 0 and 100.
     * This is useful if the instance of the stitcher is managing
     * a child process.  In that case, the parent stitcher is scraping
     * the logs of the child process for progress updates and setting
     * its progress to that value.
     */
    void setCurrentProgress(const std::string &progress)
    {
        if (!_enabled) {
            return;
        }

        _currentProgress = std::stod(progress);
        updated();
    }

protected:
    /**
     * @brief _currentEstimate
     * The current estimated time remaining.
     */
    ElapsedTime _currentEstimate;

    /**
     * @brief _currentProgress
     * The current progress of the overall stitch.
     * A number between 0 and 100.
     */
    double _currentProgress;

    /**
     * @brief _enabled
     * Whether the estimator is enabled.
     */
    bool _enabled;

    /**
     * @brief _logEnabled
     * Whether to log estimate and progress updates.
     */
    bool _logEnabled;

    /**
     * @brief _logger
     * A pointer to an instance of a logger.
     */
    const std::shared_ptr<airmap::logging::Logger> _logger;

    /**
     * @brief _updatedCb
     * A function that is called when the estimate or progress
     * is updated.
     */
    UpdatedCb _updatedCb;

    /**
     * @brief log
     * Logs estimate and progress updates.
     */
    void log() const
    {
        if (!_enabled || !_logEnabled) {
            return;
        }

        _logger->log(airmap::logging::Logger::Severity::info,
                     (estimateLogPrefix + currentEstimate().str()).c_str(), "stitcher");

        _logger->log(airmap::logging::Logger::Severity::info,
                     (progressLogPrefix + std::to_string(currentProgress())).c_str(),
                     "stitcher");
    }

    /**
     * @brief updated
     * Called when the estimate or progress is updated.  Currently,
     * this just calls _updatedCb if enabled.
     */
    void updated() const
    {
        if (!_enabled) {
            return;
        }

        _updatedCb();
    }
};

/**
 * @brief OperationsEstimator
 * Manages estimation of time remaining for all operations.
 */
class OperationsEstimator : public Estimator
{
public:
    using SharedPtr = std::shared_ptr<OperationsEstimator>;
    using OperationTimesCb = std::function<OperationElapsedTimesMap()>;

    OperationsEstimator(
            const std::shared_ptr<Camera> camera,
            const std::shared_ptr<airmap::logging::Logger> logger,
            UpdatedCb updatedCb = []() {}, bool enabled = false, bool logEnabled = false);

    static OperationsEstimator::SharedPtr create(
            const std::shared_ptr<Camera> camera,
            const std::shared_ptr<airmap::logging::Logger> logger,
            UpdatedCb updatedCb = []() {}, bool enabled = false, bool logEnabled = false);

    /**
     * @brief changeOperation
     * Change the current operation.
     * @param operation The current operation.
     */
    void changeOperation(const Operation &operation);

    /**
     * @brief currentEstimate
     * Calculates and returns the current estimate of time
     * remaining.
     */
    const ElapsedTime currentEstimate() const override;

    /**
     * @brief currentProgress
     * Calculates and returns the current progress
     * as a number between 0 and 100.
     */
    double currentProgress() const override;

    /**
     * @brief elapsedToEstimateRatio
     * Returns the average ratio of elapsed times to
     * estimates for all completed operations.
     */
    double elapsedToEstimateRatio() const;

    /**
     * @brief elapsedToEstimateRatios
     * Aggregates and returns the ratio of elapsed time to
     * estimated time for each completed operation.
     */
    const OperationDoubleMap elapsedToEstimateRatios() const;

    /**
     * @brief estimatedTimeRemaining
     * Calculate and return the estimated time remaining.
     * The estimate is adjusted based on the ratio of elapsed times to
     * estimated times for completed operations.
     */
    const ElapsedTime estimatedTimeRemaining() const;

    /**
     * @brief
     * Calculates and returns the estimated time for
     * all operations to complete.
     */
    const ElapsedTime estimatedTimeTotal() const;

    /**
     * @brief estimateOperations
     * Returns the initial estimates for each operation
     * based on known information about the stitch (e.g. camera
     * model and distortion model).
     */
    const OperationElapsedTimesMap estimateOperations() const;

    /**
     * @brief operationEstimateTimes
     * Return the previously calculated estimates for each operation.
     */
    const OperationElapsedTimesMap operationEstimateTimes() const;

    /**
     * @brief setOperationTimesCb
     * @param operationTimesCb A function, which is expected to return
     * the current elapsed times for each completed operation.
     */
    void setOperationTimesCb(const OperationTimesCb operationTimesCb);

    /**
     * @brief updateCurrentOperation
     * @param progress Progress of the current operation.  A number
     * between 0 and 1.
     */
    void updateCurrentOperation(double progress);

protected:
    /**
     * @brief _camera
     * The detected camera, if any.
     */
    const std::shared_ptr<Camera> _camera;

    /**
     * @brief _currentOperation
     * The current operation being executed.
     */
    Operation _currentOperation;

    /**
     * @brief _currentOperationProgress
     * Progress of the current operation.
     */
    double _currentOperationProgress;

    /**
     * @brief _operationEstimates
     * The current estimates for each operation.
     */
    OperationElapsedTimesMap _operationEstimates;

    /**
     * @brief _operationTimesCb
     * A function that returns the elapsed times of completed
     * operations.
     */
    OperationTimesCb _operationTimesCb;
};

} // namespace monitor
} // namespace stitcher
} // namespace airmap
