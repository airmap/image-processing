#include "gtest/gtest.h"

#include "airmap/camera.h"
#include "airmap/camera_models.h"
#include "airmap/logging.h"
#include "airmap/monitor/monitor.h"
#include "airmap/monitor/operation.h"

using airmap::logging::Logger;
using airmap::logging::stdoe_logger;
using airmap::stitcher::Camera;
using airmap::stitcher::CameraModels;
using airmap::stitcher::monitor::ElapsedTime;
using airmap::stitcher::monitor::Estimator;
using airmap::stitcher::monitor::Monitor;
using airmap::stitcher::monitor::Operation;
using airmap::stitcher::monitor::OperationElapsedTimesMap;
using airmap::stitcher::monitor::OperationsEstimator;

class MonitorTest : public ::testing::Test {
protected:
    MonitorTest()
        : camera(std::make_shared<Camera>(CameraModels::ParrotAnafiThermal()))
        , logger(std::make_shared<stdoe_logger>())
        , estimator(OperationsEstimator::create(camera, logger))
    {
        estimator->enable();
    }

    Monitor createMonitor() { return { estimator, logger }; }

    std::shared_ptr<Camera> camera;
    std::shared_ptr<airmap::logging::Logger> logger;
    OperationsEstimator::SharedPtr estimator;
};

TEST_F(MonitorTest, changeOperation)
{
    Monitor monitor = createMonitor();
    monitor.enable();

    const OperationElapsedTimesMap operationEstimates = estimator->estimateOperations();

    EXPECT_EQ(estimator->currentEstimate(), estimator->estimatedTimeRemaining());
    EXPECT_EQ(estimator->currentEstimate(), estimator->estimatedTimeTotal());

    monitor.changeOperation(Operation::Start());
    EXPECT_EQ(estimator->currentEstimate(), estimator->estimatedTimeRemaining());
    EXPECT_EQ(estimator->currentEstimate(), estimator->estimatedTimeTotal());

    monitor.changeOperation(Operation::UndistortImages());
    EXPECT_EQ(estimator->currentEstimate(), estimator->estimatedTimeRemaining());
    EXPECT_EQ(estimator->estimatedTimeRemaining(),
              estimator->estimatedTimeTotal()
                      - operationEstimates.at(Operation::Start().value()));

    for (int i = 2; i < operationEstimates.size(); i++) {
        monitor.changeOperation(static_cast<Operation::Enum>(i));
    }

    EXPECT_EQ(estimator->estimatedTimeRemaining(),
              operationEstimates.at(
                      static_cast<Operation::Enum>(operationEstimates.size() - 1)));
}

TEST_F(MonitorTest, operationProgress)
{
    Monitor monitor = createMonitor();
    monitor.enable();

    const OperationElapsedTimesMap operationEstimates = estimator->estimateOperations();

    monitor.changeOperation(Operation::Start());
    EXPECT_EQ(estimator->currentEstimate(), estimator->estimatedTimeRemaining());
    EXPECT_EQ(estimator->currentEstimate(), estimator->estimatedTimeTotal());

    monitor.changeOperation(Operation::UndistortImages());
    EXPECT_EQ(estimator->currentEstimate(), estimator->estimatedTimeRemaining());

    const ElapsedTime undistortEstimate =
        operationEstimates.at(Operation::UndistortImages().value());
    const ElapsedTime startingEstimatedTimeRemaining =
            estimator->estimatedTimeRemaining();

    for (int i = 0; i < 50; i++) {
        double operationProgress = static_cast<double>(i) / 50.;
        estimator->updateCurrentOperation(operationProgress);
        EXPECT_EQ(estimator->currentEstimate(), estimator->estimatedTimeRemaining());
        EXPECT_EQ(estimator->currentEstimate(),
                  startingEstimatedTimeRemaining - undistortEstimate * operationProgress);
    }
}
