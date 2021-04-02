#include "gtest/gtest.h"

#include "airmap/camera.h"
#include "airmap/camera_models.h"
#include "airmap/logging.h"
#include "airmap/monitor/estimator.h"
#include "airmap/monitor/operation.h"

#include <boost/optional.hpp>

using airmap::logging::Logger;
using airmap::logging::stdoe_logger;
using airmap::stitcher::Camera;
using airmap::stitcher::CameraModels;
using airmap::stitcher::monitor::ElapsedTime;
using airmap::stitcher::monitor::Estimator;
using airmap::stitcher::monitor::Operation;
using airmap::stitcher::monitor::OperationDoubleMap;
using airmap::stitcher::monitor::OperationDoublePair;
using airmap::stitcher::monitor::OperationElapsedTimesMap;
using airmap::stitcher::monitor::OperationElapsedTimesPair;

class EstimatorTest : public ::testing::Test {
protected:
    EstimatorTest()
        : cameraAnafi(CameraModels::ParrotAnafiThermal())
        , cameraVesper(CameraModels::VantageVesperEONavigation())
        , logger(std::make_shared<stdoe_logger>())
    {
    }

    OperationElapsedTimesMap anafiOperationEstimates()
    {
        Estimator estimator = estimatorAnafi();
        estimator.enable();
        return estimator.estimateOperations();
    }

    Estimator estimatorAnafi() { return {cameraAnafi, logger}; }

    Estimator estimatorVesper() { return {cameraVesper, logger}; }

    OperationElapsedTimesMap
    getOperationTimes(const OperationElapsedTimesMap &operationEstimates,
                      const boost::optional<const Operation> _lastOperation =
                          boost::optional<const Operation>(),
                      const double multiplier = 1.)
    {
        OperationElapsedTimesMap operationTimes;

        if (!_lastOperation.has_value()) {
            return operationTimes;
        }

        const Operation &lastOperation = _lastOperation.get();

        for (auto &operationPair : operationEstimates) {
            Operation operation{operationPair.first};
            if (lastOperation > operation) {
                operationTimes.insert(std::make_pair(
                    operation.value(), operationPair.second * multiplier));
            }
        }

        return operationTimes;
    }

    Camera cameraAnafi;
    Camera cameraVesper;
    std::shared_ptr<Logger> logger;
};

TEST_F(EstimatorTest, changeOperations)
{
    Estimator estimator = estimatorAnafi();
    estimator.enable();

    const OperationElapsedTimesMap operationEstimates =
        anafiOperationEstimates();

    for (auto &operationTimePair : operationEstimates) {
        Operation operation{operationTimePair.first};
        estimator.changeOperation(operation);
        EXPECT_EQ(estimator.currentEstimate(),
                  estimator.estimatedTimeRemaining());
    }
}

TEST_F(EstimatorTest, changeOperationsWithCb)
{
    Estimator estimator = estimatorAnafi();
    estimator.enable();

    const OperationElapsedTimesMap operationEstimates =
        anafiOperationEstimates();

    Operation operation = Operation::Start();
    estimator.setOperationTimesCb([this, operationEstimates, operation]() {
        return getOperationTimes(operationEstimates, operation);
    });

    for (auto &operationTimePair : operationEstimates) {
        Operation operation{operationTimePair.first};
        estimator.changeOperation(operation);
        EXPECT_EQ(estimator.currentEstimate(),
                  estimator.estimatedTimeRemaining());
    }
}

TEST_F(EstimatorTest, changesOperationsAdjusted)
{
    Estimator estimator = estimatorAnafi();
    estimator.enable();

    const OperationElapsedTimesMap operationEstimates =
        anafiOperationEstimates();

    Operation operation = Operation::Start();
    estimator.setOperationTimesCb([this, operationEstimates, operation]() {
        return getOperationTimes(operationEstimates, operation, 2.);
    });

    estimator.changeOperation(operation);
    EXPECT_EQ(estimator.currentEstimate(), estimator.estimatedTimeRemaining());

    operation = Operation::UndistortImages();
    estimator.setOperationTimesCb([this, operationEstimates, operation]() {
        return getOperationTimes(operationEstimates, operation, 2.);
    });
    estimator.changeOperation(operation);
    EXPECT_EQ(estimator.currentEstimate(),
              estimator.estimatedTimeRemaining() * 2.);

    operation = Operation::FindFeatures();
    estimator.setOperationTimesCb([this, operationEstimates, operation]() {
        return getOperationTimes(operationEstimates, operation, 3.);
    });
    estimator.changeOperation(operation);
    EXPECT_EQ(estimator.currentEstimate(),
              estimator.estimatedTimeRemaining() * 3.);

    operation = Operation::MatchFeatures();
    estimator.setOperationTimesCb([this, operationEstimates, operation]() {
        return getOperationTimes(operationEstimates, operation, 0.5);
    });
    estimator.changeOperation(operation);
    estimator.changeOperation(operation);
    EXPECT_EQ(estimator.currentEstimate(),
              estimator.estimatedTimeRemaining() * 0.5);

    operation = Operation::EstimateCameraParameters();
    estimator.setOperationTimesCb([this, operationEstimates, operation]() {
        return getOperationTimes(operationEstimates, operation, 0.7);
    });
    estimator.changeOperation(operation);
    EXPECT_EQ(estimator.currentEstimate(),
              estimator.estimatedTimeRemaining() * 0.7);

    operation = Operation::AdjustCameraParameters();
    estimator.setOperationTimesCb([this, operationEstimates, operation]() {
        return getOperationTimes(operationEstimates, operation, 0.1);
    });
    estimator.changeOperation(operation);
    EXPECT_EQ(estimator.currentEstimate(),
              estimator.estimatedTimeRemaining() * 0.1);

    operation = Operation::PrepareExposureCompensation();
    estimator.setOperationTimesCb([this, operationEstimates, operation]() {
        OperationElapsedTimesMap operationTimes =
            getOperationTimes(operationEstimates, operation);

        OperationElapsedTimesMap adjustedOperationTimes;
        for (auto &operationTimePair : operationTimes) {
            adjustedOperationTimes.insert(std::make_pair(
                operationTimePair.first,
                operationTimePair.second *
                    static_cast<double>(operationTimePair.first)));
        }

        return adjustedOperationTimes;
    });
    estimator.changeOperation(operation);
    EXPECT_EQ(estimator.currentEstimate(),
              estimator.estimatedTimeRemaining() * 2.666666);

    operation = Operation::FindSeams();
    estimator.setOperationTimesCb([this, operationEstimates, operation]() {
        OperationElapsedTimesMap operationTimes =
            getOperationTimes(operationEstimates, operation);

        OperationElapsedTimesMap adjustedOperationTimes;
        for (auto &operationTimePair : operationTimes) {
            adjustedOperationTimes.insert(std::make_pair(
                operationTimePair.first,
                operationTimePair.second /
                        static_cast<double>(operationTimePair.first) +
                    1.));
        }

        return adjustedOperationTimes;
    });
    estimator.changeOperation(operation);
    EXPECT_EQ(estimator.currentEstimate(),
              estimator.estimatedTimeRemaining() * 0.495048);
}

TEST_F(EstimatorTest, estimatedTimeRemaining)
{
    Estimator estimator = estimatorAnafi();
    estimator.enable();

    const OperationElapsedTimesMap operationEstimates =
        anafiOperationEstimates();

    EXPECT_EQ(estimator.estimatedTimeRemaining(),
              estimator.estimatedTimeTotal());

    estimator.changeOperation(Operation::Start());
    EXPECT_EQ(estimator.estimatedTimeRemaining(),
              estimator.estimatedTimeTotal());

    estimator.changeOperation(Operation::UndistortImages());
    EXPECT_NE(estimator.estimatedTimeRemaining(),
              estimator.estimatedTimeTotal());
    EXPECT_EQ(estimator.estimatedTimeRemaining(),
              estimator.estimatedTimeTotal() -
                  operationEstimates.at(Operation::Start().value()));

    for (int i = 2; i < operationEstimates.size(); i++) {
        estimator.changeOperation(static_cast<Operation::Enum>(i));
    }

    EXPECT_EQ(estimator.estimatedTimeRemaining(),
              operationEstimates.at(
                  static_cast<Operation::Enum>(operationEstimates.size() - 1)));
}

// TODO(bkd):
// - current progress
// - current operation progress (updateCurrentOperation)
// - current operation estimate
// - updated callback
// - setCurrentEstimate
// - setCurrentProgress
