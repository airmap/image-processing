#pragma once

#include "airmap/distortion.h"
#include "airmap/stitcher_configuration.h"

#include <boost/optional.hpp>
#include <opencv2/core/utility.hpp>

namespace airmap {
namespace stitcher {

/**
 * @brief Camera information
 * Contains information about the camera, including
 * intrinsics, distortion coefficients, and field of view.
 *
 * This is currently used for undistortion, homography calculation,
 * and rotation calculation.
 *
 * OpenCV camera calibration documentation:
 *  - https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d
 */
struct Camera
{
    using ConfigurationCb = std::function<const Configuration(
        const Configuration &configuration, StitchType stitchType)>;

    enum class FOVUnits { Degrees, Radians };

    /**
     * @brief calibration_intrinsics;
     * Intrinsic matrix from calibration.
     */
    boost::optional<cv::Mat> calibration_intrinsics;

    /**
     * @brief distortion_model
     * Distortion model.
     */
    std::shared_ptr<DistortionModel> distortion_model;

    /**
     * @brief focal_length_meters
     * Focal length in meters.
     */
    double focal_length_meters;

    /**
     * @brief principal_point
     * Principal point.
     */
    cv::Point2d principal_point;

    /**
     * @brief sensor_dimensions_meters
     * Sensor dimensions in meters.
     */
    cv::Point2d sensor_dimensions_meters;

    /**
     * @brief sensor_dimensions_pixels
     * Sensor dimensions in pixels.
     */
    cv::Point2d sensor_dimensions_pixels;

    /**
     * @brief Camera
     * Create a camera.
     */
    Camera(double _focal_length_meters, cv::Point2d _sensor_dimensions_meters,
           cv::Point2d _sensor_dimensions_pixels, cv::Point2d _principal_point,
           boost::optional<cv::Mat> _calibration_intrinsics =
               boost::optional<cv::Mat>(),
           std::shared_ptr<DistortionModel> _distortion_model = nullptr,
           ConfigurationCb configurationCb = [](
               const Configuration &configuration, StitchType stitchType) {
                   return configuration;
               })
        : calibration_intrinsics(_calibration_intrinsics)
        , distortion_model(std::move(_distortion_model))
        , focal_length_meters(_focal_length_meters)
        , principal_point(_principal_point)
        , sensor_dimensions_meters(_sensor_dimensions_meters)
        , sensor_dimensions_pixels(_sensor_dimensions_pixels)
        , _configurationCb(configurationCb)
    {
    }

    Camera()
        : focal_length_meters(0)
        , principal_point(cv::Point2d(0, 0))
        , sensor_dimensions_meters(cv::Point2d(0, 0))
        , sensor_dimensions_pixels(cv::Point2d(0, 0))
    {
    }

    /**
     * @brief configuration
     * If the configuration callback is set, return the result of
     * calling it with the provided configuration and stitch type.
     */
    const Configuration configuration(const Configuration &configuration,
                                      StitchType stitchType) const
    {
        if (_configurationCb) {
            return _configurationCb(configuration, stitchType);
        }

        return configuration;
    }

    /**
     * @brief focalLengthPixels
     * Focal length in pixels.
     */
    cv::Point2d focalLengthPixels() const
    {
        double x = focal_length_meters * sensor_dimensions_pixels.x / sensor_dimensions_meters.x;
        double y = focal_length_meters * sensor_dimensions_pixels.y / sensor_dimensions_meters.y;
        return cv::Point2d(x, y);
    }

    /**
     * @brief fov
     * Horizontal and vertical fields of view.
     * @params units - degrees or radians
     */
    cv::Point2d fov(FOVUnits units = FOVUnits::Radians) const
    {
        double x = 2 * atan(sensor_dimensions_meters.x / (2 * focal_length_meters));
        double y = 2 * atan(sensor_dimensions_meters.y / (2 * focal_length_meters));

        if (units == FOVUnits::Degrees) {
            x *= 180 / M_PI;
            y *= 180 / M_PI;
        }

        return cv::Point2d(x, y);
    }

    /**
     * @brief Intrinsics matrix.
     * @param scale
     */
    cv::Mat K(double scale = 1.0) const
    {
        if (calibration_intrinsics.has_value()) {
            cv::Mat K;
            calibration_intrinsics.value().copyTo(K);
            K.at<double>(0, 0) *= scale;
            K.at<double>(0, 2) *= scale;
            K.at<double>(1, 1) *= scale;
            K.at<double>(1, 2) *= scale;
            return K;
        }

        cv::Point2d focal_length_pixels = focalLengthPixels();
        return (cv::Mat_<double>(3, 3) <<
                focal_length_pixels.x * scale, 0, principal_point.x * scale,
                0, focal_length_pixels.y * scale, principal_point.y * scale,
                0, 0, 1);
    }

private:
    ConfigurationCb _configurationCb;
};

} // namespace stitcher
} // namespace airmap
