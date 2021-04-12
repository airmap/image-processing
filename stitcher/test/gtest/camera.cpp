#include "gtest/gtest.h"
#include "airmap/camera.h"

using airmap::stitcher::Camera;

class CameraTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        focal_length_meters = 4.04e-3;
        sensor_dimensions_meters = cv::Point2d(7.22e-3, 5.50e-3);
        sensor_dimensions_pixels = cv::Point2d(5344, 4016);
        principal_point = cv::Point2d(sensor_dimensions_pixels.x / 2,
                                      sensor_dimensions_pixels.y / 2);
        calibration_intrinsics = (cv::Mat_<double>(3, 3) <<
            2.9803291905661031e+03, 0.0, 2.3574166587045511e+03,
            0.0, 2.9704815213723482e+03, 1.6709805782521339e+03,
            0.0, 0.0, 1.0);
    }

    Camera createCamera()
    {
        return Camera(focal_length_meters, sensor_dimensions_meters,
                      sensor_dimensions_pixels, principal_point);
    }

    Camera createCalibratedCamera()
    {
        return Camera(focal_length_meters, sensor_dimensions_meters,
                      sensor_dimensions_pixels, principal_point,
                      std::make_shared<cv::Mat>(calibration_intrinsics));
    }

    cv::Point2d createFocalLengthPixels()
    {
        return cv::Point2d(2990.2714681440448, 2949.934545454546);
    }

    cv::Mat createCalibratedIntrinsicsMatrix(bool scale = false)
    {
        if (scale) {
            return (cv::Mat_<double>(3, 3) <<
                    1495.1357340720224, 0, 1336,
                    0, 1474.967272727273, 1004,
                    0, 0, 1);
        } else {
            return (cv::Mat_<double>(3, 3) <<
                    2990.2714681440448, 0, 2672,
                    0, 2949.934545454546, 2008,
                    0, 0, 1);
        }
    }

    cv::Mat createIntrinsicsMatrix(bool scale = false)
    {
        if (scale) {
            return (cv::Mat_<double>(3, 3) <<
                    1495.1357340720224, 0, 1336,
                    0, 1474.967272727273, 1004,
                    0, 0, 1);
        } else {
            return (cv::Mat_<double>(3, 3) <<
                    2990.2714681440448, 0, 2672,
                    0, 2949.934545454546, 2008,
                    0, 0, 1);
        }
    }

    cv::Point2d createFov(Camera::FOVUnits units = Camera::FOVUnits::Radians)
    {
        switch (units) {
            case Camera::FOVUnits::Degrees:
                return cv::Point2d(83.565676480369063, 68.485694677948686);
            case Camera::FOVUnits::Radians:
                return cv::Point2d(1.4584961962388268, 1.1953008626457622);
        }
    }

    // Camera intrinsics.
    double focal_length_meters;
    cv::Point2d sensor_dimensions_meters;
    cv::Point2d sensor_dimensions_pixels;
    cv::Point2d principal_point;
    cv::Mat calibration_intrinsics;

};

TEST_F(CameraTest, cameraStruct)
{
    Camera camera = createCamera();

    // Test intrinsics.
    EXPECT_DOUBLE_EQ(camera.focal_length_meters, focal_length_meters);
    EXPECT_DOUBLE_EQ(camera.sensor_dimensions_meters.x, sensor_dimensions_meters.x);
    EXPECT_DOUBLE_EQ(camera.sensor_dimensions_meters.y, sensor_dimensions_meters.y);
    EXPECT_DOUBLE_EQ(camera.sensor_dimensions_pixels.x, sensor_dimensions_pixels.x);
    EXPECT_DOUBLE_EQ(camera.sensor_dimensions_pixels.y, sensor_dimensions_pixels.y);
    EXPECT_DOUBLE_EQ(camera.principal_point.x, principal_point.x);
    EXPECT_DOUBLE_EQ(camera.principal_point.y, principal_point.y);
}

TEST_F(CameraTest, cameraFocalLengthPixels)
{
    Camera camera = createCamera();
    cv::Point2d actual = camera.focalLengthPixels();
    cv::Point2d expected = createFocalLengthPixels();
    EXPECT_DOUBLE_EQ(actual.x, expected.x);
    EXPECT_DOUBLE_EQ(actual.y, expected.y);
}

TEST_F(CameraTest, cameraFov)
{
    Camera camera = createCamera();
    cv::Point2d actual, expected;

    // Radians
    actual = camera.fov();
    expected = createFov();
    EXPECT_DOUBLE_EQ(actual.x, expected.x);
    EXPECT_DOUBLE_EQ(actual.y, expected.y);

    // Degrees
    actual = camera.fov(Camera::FOVUnits::Degrees);
    expected = createFov(Camera::FOVUnits::Degrees);
    EXPECT_DOUBLE_EQ(actual.x, expected.x);
    EXPECT_DOUBLE_EQ(actual.y, expected.y);
}

TEST_F(CameraTest, cameraIntrinsicsMatrix)
{
    Camera camera = createCamera();
    cv::Mat actual, expected;

    // full scale
    actual = camera.K();
    expected = createIntrinsicsMatrix();
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 0), expected.at<double>(0, 0));
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 1), expected.at<double>(0, 1));
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 2), expected.at<double>(0, 2));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 0), expected.at<double>(1, 0));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 1), expected.at<double>(1, 1));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 2), expected.at<double>(1, 2));
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 0), expected.at<double>(2, 0));
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 1), expected.at<double>(2, 1));
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 2), expected.at<double>(2, 2));

    // 0.5 scale
    actual = camera.K(0.5);
    expected = createIntrinsicsMatrix(true);
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 0), expected.at<double>(0, 0));
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 1), expected.at<double>(0, 1));
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 2), expected.at<double>(0, 2));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 0), expected.at<double>(1, 0));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 1), expected.at<double>(1, 1));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 2), expected.at<double>(1, 2));
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 0), expected.at<double>(2, 0));
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 1), expected.at<double>(2, 1));
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 2), expected.at<double>(2, 2));
}

TEST_F(CameraTest, cameraCalibratedIntrinsicsMatrix)
{
    Camera camera = createCalibratedCamera();
    cv::Mat actual;

    // full scale
    actual = camera.K();
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 0), calibration_intrinsics.at<double>(0, 0));
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 1), 0);
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 2), calibration_intrinsics.at<double>(0, 2));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 0), 0);
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 1), calibration_intrinsics.at<double>(1, 1));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 2), calibration_intrinsics.at<double>(1, 2));
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 0), 0);
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 1), 0);
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 2), 1);

    // 0.5 scale
    actual = camera.K(0.5);
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 0), calibration_intrinsics.at<double>(0, 0) * 0.5);
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 1), 0);
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 2), calibration_intrinsics.at<double>(0, 2) * 0.5);
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 0), 0);
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 1), calibration_intrinsics.at<double>(1, 1) * 0.5);
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 2), calibration_intrinsics.at<double>(1, 2) * 0.5);
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 0), 0);
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 1), 0);
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 2), 1);
}
