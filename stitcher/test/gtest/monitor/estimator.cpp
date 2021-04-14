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
using airmap::stitcher::monitor::OperationElapsedTimesMap;

class EstimatorTest : public ::testing::Test {
protected:
    EstimatorTest()
        : cameraAnafi(
              std::make_shared<Camera>(CameraModels::ParrotAnafiThermal()))
        , cameraVesper(std::make_shared<Camera>(
              CameraModels::VantageVesperEONavigation()))
        , logger(std::make_shared<stdoe_logger>())
    {
    }

    const OperationElapsedTimesMap anafiOperationEstimates()
    {
        Estimator estimator = estimatorAnafi();
        estimator.enable();
        return estimator.estimateOperations();
    }

    const OperationElapsedTimesMap vesperOperationEstimates()
    {
        Estimator estimator = estimatorVesper();
        estimator.enable();
        return estimator.estimateOperations();
    }

    Estimator estimatorAnafi(Estimator::UpdatedCb updatedCb = []() {})
    {
        return {cameraAnafi, logger, updatedCb};
    }

    Estimator estimatorVesper(Estimator::UpdatedCb updatedCb = []() {})
    {
        return {cameraVesper, logger, updatedCb};
    }

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

    std::shared_ptr<Camera> cameraAnafi;
    std::shared_ptr<Camera> cameraVesper;
    std::shared_ptr<airmap::logging::Logger> logger;
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
    EXPECT_EQ(estimator.currentEstimate(), estimator.estimatedTimeRemaining());
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

TEST_F(EstimatorTest, operationProgress)
{
    Estimator estimator = estimatorVesper();
    estimator.enable();

    estimator.changeOperation(Operation::Start());
    EXPECT_EQ(estimator.currentEstimate(), estimator.estimatedTimeRemaining());
    EXPECT_EQ(estimator.currentEstimate(), estimator.estimatedTimeTotal());

    estimator.changeOperation(Operation::UndistortImages());
    EXPECT_EQ(estimator.currentEstimate(), estimator.estimatedTimeRemaining());

    const ElapsedTime undistortEstimate =
        vesperOperationEstimates().at(Operation::UndistortImages().value());
    const ElapsedTime startingEstimatedTimeRemaining =
        estimator.estimatedTimeRemaining();

    for (int i = 0; i < 50; i++) {
        double operationProgress = static_cast<double>(i) / 50.;
        estimator.updateCurrentOperation(operationProgress);
        EXPECT_EQ(estimator.currentEstimate(),
                  estimator.estimatedTimeRemaining());
        EXPECT_EQ(estimator.currentEstimate(),
                  startingEstimatedTimeRemaining -
                      undistortEstimate * operationProgress);
    }
}

TEST_F(EstimatorTest, currentProgress)
{
    Estimator estimator = estimatorVesper();
    estimator.enable();

    const OperationElapsedTimesMap operationEstimates =
        vesperOperationEstimates();

    estimator.changeOperation(Operation::Start());
    const ElapsedTime estimatedTimeTotal = estimator.estimatedTimeTotal();
    estimator.changeOperation(Operation::UndistortImages());

    const ElapsedTime startingTimeRemaining =
        estimator.estimatedTimeRemaining();
    double startingProgress = 100. * (1 - startingTimeRemaining / estimatedTimeTotal);
    EXPECT_EQ(estimator.currentProgress(), startingProgress);

    const ElapsedTime undistortEstimate =
        vesperOperationEstimates().at(Operation::UndistortImages().value());
    const ElapsedTime estimatedTimeRemaining =
        estimator.estimatedTimeRemaining();
    const double undistortOperationPercent =
        undistortEstimate / estimatedTimeTotal;

    for (int i = 0; i < 50; i++) {
        double operationProgress = static_cast<double>(i) / 50.;
        estimator.updateCurrentOperation(operationProgress);
        EXPECT_EQ(estimator.currentProgress(),
                  100.
                          * (1.
                             - estimator.estimatedTimeRemaining()
                                     / estimator.estimatedTimeTotal()));

        EXPECT_LT(estimator.currentProgress()
                          - 100.
                                  * (startingProgress
                                     + operationProgress * undistortOperationPercent),
                  0.00001);
    }
}

TEST_F(EstimatorTest, disable)
{
    Estimator estimator = estimatorAnafi();
    estimator.enable();

    estimator.changeOperation(Operation::Start());
    const ElapsedTime startingEtimatedTimeRemaining =
        estimator.estimatedTimeRemaining();

    estimator.disable();
    for (int i = 1; i < estimator.estimateOperations().size(); i++) {
        Operation operation{static_cast<Operation::Enum>(i)};
        estimator.changeOperation(operation);
        EXPECT_EQ(estimator.currentEstimate(), startingEtimatedTimeRemaining);
        EXPECT_EQ(estimator.estimatedTimeRemaining(),
                  startingEtimatedTimeRemaining);
    }
}

TEST_F(EstimatorTest, setCurrentEstimate)
{
    Estimator estimator = estimatorVesper();
    estimator.enable();

    for (int i = 50; i >= 0; i--) {
        ElapsedTime estimate = ElapsedTime::fromSeconds(i);
        estimator.setCurrentEstimate(estimate.str());
        EXPECT_EQ(estimator.currentEstimate(), estimate);
    }
}

TEST_F(EstimatorTest, setCurrentProgress)
{
    Estimator estimator = estimatorVesper();
    estimator.enable();

    for (int i = 50; i >= 0; i--) {
        double progress = static_cast<double>(i) / 50.;
        estimator.setCurrentProgress(std::to_string(progress));
        EXPECT_EQ(estimator.currentProgress(), progress);
    }
}

TEST_F(EstimatorTest, updatedCallback)
{
    int calledCount = 0;
    Estimator estimator = estimatorVesper([&calledCount]() { calledCount++; });
    estimator.enable();

    estimator.changeOperation(Operation::Start());
    EXPECT_EQ(calledCount, 1);

    estimator.setCurrentEstimate("01:02:03.004");
    EXPECT_EQ(calledCount, 2);

    estimator.setCurrentProgress("45.32");
    EXPECT_EQ(calledCount, 3);
}
