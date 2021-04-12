#pragma once

#include <atomic>
#include <sstream>

#include "airmap/camera.h"
#include "airmap/camera_models.h"
#include "airmap/logging.h"
#include "airmap/monitor/monitor.h"
#include "airmap/panorama.h"

using airmap::stitcher::monitor::Monitor;

namespace airmap {
namespace stitcher {

/**
 * @brief is a baseclass that can be put to the task of stitching a 360
 * Panorama.
 * @details Stitcher is cancellable, which means a single instance is single
 * use.
 */
class Stitcher {
public:
    /**
     * @brief The Report struct conveys metatada about the stitched image
     * @details the stitcher has decisions to make (how much do we scale the
     * input?) and stats to collect (how many features were detected), while the
     * caller may be intested in knowing what these were. The stitcher may
     * report them via this structure.
     */
    struct Report {
        /**
         * @brief inputScaled - the fraction to which the input was scaled.
         *  Stitcher has RAM requirements and it may decide that it can only
         * succeed with the RAM it is given if it scales the input images.
         */
        double inputScaled = 1.0;
        size_t inputSizeMB = 0;
    };

    /**
     * @brief The RetriableError class conveys a stitching error that can be
     * retried on.
     * @details The process of stitching (esp. feature extraction and matching)
     * can be indeterministic.
     */
    class RetriableError : public std::invalid_argument {
    public:
        RetriableError(const std::string &what)
            : std::invalid_argument(what)
        {
        }
    };

    using SharedPtr = std::shared_ptr<Stitcher>;
    virtual ~Stitcher() = default;

    /**
     * @brief cancel
     * Cancel an active stitch.
     */
    virtual void cancel() = 0;

    /**
     * @brief setFallbackMode
     * Set fallback mode.  Fallback mode might be enabled after a failure,
     * so that the stitcher can fallback to safer settings
     * (e.g. disabling OpenCL).
     */
    virtual void setFallbackMode() {}

    /**
     * @brief stitch is the Stitcher's main entry - subclasses should
     * fill with the actual stitching.
     */
    virtual Report stitch() = 0;
};

/**
 * @brief MonitoredStitcher
 * A Stitcher with a monitor and estimator, which can track and estimate
 * elapsed time of stitch operations.
 */
class MonitoredStitcher : public Stitcher {
public:
    using SharedPtr = std::shared_ptr<MonitoredStitcher>;
    using UpdatedCb = monitor::Estimator::UpdatedCb;

    MonitoredStitcher(
        const Panorama &panorama, const Panorama::Parameters &parameters,
        std::shared_ptr<Logger> logger, UpdatedCb updatedCb = []() {})
        : _camera(CameraModels().detect(panorama.front()))
        , _estimator(_camera, logger, updatedCb, parameters.enableEstimate,
                     parameters.enableEstimateLog)
        , _monitor(std::make_shared<Monitor>(_estimator, logger,
                                             parameters.enableElapsedTime,
                                             parameters.enableEstimateLog))
    {
        _estimator.setOperationTimesCb(
            [this]() { return _monitor->operationTimes(); });
    }

    monitor::Estimator &estimator() { return _estimator; }

    Monitor::SharedPtr monitor() { return _monitor; }

protected:
    /**
     * @brief _camera
     * The detected camera.
     */
    const std::shared_ptr<Camera> _camera;

    /**
     * @brief _estimator
     * The stitcher estimator.  Manages operation elapsed time estimates.
     */
    monitor::Estimator _estimator;

    /**
     * @brief _monitor
     * The stitcher monitor.  Manages operation elapsed time and estimates.
     */
    Monitor::SharedPtr _monitor;
};

/**
 * @brief The RetryingStitcher class is such that can retry the stitching based
 * on Parameters::retries
 */
class RetryingStitcher : public Stitcher {
public:
    RetryingStitcher(MonitoredStitcher::SharedPtr underlying,
                     const Panorama::Parameters &parameters,
                     std::shared_ptr<logging::Logger> logger)
        : _underlying(underlying)
        , _retries(parameters.retries)
        , _logger(logger)
    {
    }

    void cancel() override
    {
        _retries = 0;
        _underlying->cancel();
    }

    Report stitch() override
    {
        for (size_t i = 0; i < _retries; i++) {
            try {
                return _underlying->stitch();
            } catch (const RetriableError &e) {
                if (_retries == 0) {
                    throw;
                }
                _underlying->setFallbackMode();
                std::stringstream ss;
                ss << "Stitching failed with " << e.what()
                   << ", retrying, retries left " << _retries - i;
                _logger->log(logging::Logger::Severity::error, ss.str().c_str(),
                             "stitcher");
            }
        }
        return _underlying->stitch();
    }

protected:
    MonitoredStitcher::SharedPtr _underlying;
    std::atomic<size_t> _retries;
    std::shared_ptr<logging::Logger> _logger;
};

} // namespace stitcher
} // namespace airmap
