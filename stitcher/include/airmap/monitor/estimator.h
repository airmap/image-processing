#pragma once

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
 * Manages estimation of time remaining for all operations.
 */
class Estimator {
public:
    using OperationTimesCb = std::function<OperationElapsedTimesMap()>;
    using UpdatedCb = std::function<void()>;

    Estimator(
        const std::shared_ptr<Camera> camera,
        const std::shared_ptr<Logger> logger, UpdatedCb updatedCb = []() {},
        bool enabled = false, bool logEnabled = false);

    /**
     * @brief estimateLogPrefix
     * Prefix for estimate log statements.
     * This can be used by a parent process to scrape the logs
     * for estimate updates.
     */
    static const std::string estimateLogPrefix;

    /**
     * @brief progressLogPrefix
     * Prefix for progress log statements.
     * This can be used by a parent process to scrape the logs
     * for progress updates.
     */
    static const std::string progressLogPrefix;

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
    const ElapsedTime currentEstimate() const;

    /**
     * @brief currentProgress
     * Calculates and returns the current progress
     * as a number between 0 and 100.
     */
    double currentProgress() const;

    /**
     * @brief disable
     * Disable the estimator.
     */
    void disable();

    /**
     * @brief disableLog
     *Disable logging of estimate and progress updates.
     */
    void disableLog();

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
     * @brief enable
     * Enable the estimator.
     */
    void enable();

    /**
     * @brief enableLog
     * Enable logging of estimate and progress updates.
     */
    void enableLog();

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
     * @brief setCurrentEstimate
     * Sets the current estimate value.
     * This is useful if the instance of the stitcher is managing
     * a child process.  In that case, the parent stitcher is scraping
     * the logs of the child process for estimate updates and setting
     * its estimate to that value.
     */
    void setCurrentEstimate(const std::string &estimatedTimeRemaining);

    /**
     * @brief setCurrentProgress
     * Sets the current progress value.  A number between 0 and 100.
     * This is useful if the instance of the stitcher is managing
     * a child process.  In that case, the parent stitcher is scraping
     * the logs of the child process for progress updates and setting
     * its progress to that value.
     */
    void setCurrentProgress(const std::string &progress);

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

private:
    /**
     * @brief _camera
     * The detected camera, if any.
     */
    const std::shared_ptr<Camera> _camera;

    /**
     * @brief _currentEstimate
     * The current estimated time remaining.
     */
    ElapsedTime _currentEstimate;

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
    const std::shared_ptr<Logger> _logger;

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
    void log() const;

    /**
     * @brief updated
     * Called when the estimate or progress is updated.  Currently,
     * this just calls _updatedCb if enabled.
     */
    void updated() const;
};

} // namespace monitor
} // namespace stitcher
} // namespace airmap
