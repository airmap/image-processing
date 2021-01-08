#include "gtest/gtest.h"
#include "airmap/gimbal.h"

using airmap::stitcher::GimbalOrientation;

TEST(gimbal, gimbalStruct)
{
    double pitch = 90.0;
    double roll = 180.0;
    double yaw = -90.0;
    GimbalOrientation gimbal_degrees(pitch, roll, yaw);

    // Test attributes.
    EXPECT_DOUBLE_EQ(gimbal_degrees.pitch, pitch);
    EXPECT_DOUBLE_EQ(gimbal_degrees.roll, roll);
    EXPECT_DOUBLE_EQ(gimbal_degrees.yaw, yaw);

    // Test unit conversion.
    GimbalOrientation gimbal_radians = gimbal_degrees.convertTo(GimbalOrientation::Units::Radians);
    EXPECT_DOUBLE_EQ(gimbal_radians.pitch, pitch * M_PI/180.0);
    EXPECT_DOUBLE_EQ(gimbal_radians.roll, roll * M_PI/180.0);
    EXPECT_DOUBLE_EQ(gimbal_radians.yaw, yaw * M_PI/180.0);
}

TEST(gimbal, gimbalHomography) {
    GimbalOrientation gimbal(20, 0, 30);
    cv::Mat K, actual, expected;

    /**
     * Homography should be the same as the rotation matrix
     * when the intrinsics matrix is the identity matrix
     * (except normalized).
     */
    K = cv::Mat::eye(3, 3, CV_64F);
    actual = gimbal.homography(K);
    expected = gimbal.rotationMatrix();
    expected /= expected.at<double>(2, 2); // normalize
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 0), expected.at<double>(0, 0));
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 1), expected.at<double>(0, 1));
    EXPECT_DOUBLE_EQ(actual.at<double>(0, 2), expected.at<double>(0, 2));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 0), expected.at<double>(1, 0));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 1), expected.at<double>(1, 1));
    EXPECT_DOUBLE_EQ(actual.at<double>(1, 2), expected.at<double>(1, 2));
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 0), expected.at<double>(2, 0));
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 1), expected.at<double>(2, 1));
    EXPECT_DOUBLE_EQ(actual.at<double>(2, 2), expected.at<double>(2, 2));

    // Test homography with realistic intrinsics matrix.
    K = (cv::Mat_<double>(3, 3) <<
         3000, 0, 1500,
         0, 3000, 1500,
         0, 0, 1);
    actual = gimbal.homography(K);
    expected = (cv::Mat_<double>(3, 3) <<
                0.71903213887558559, 0.19483647518625002, 1728.9614100268288,
                -0.46249088321642634, -0.87578115693445568, 4410.5811835310878,
                -0.00017843627202011758, 0.0001298909834575, 1);
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
