#pragma once

#include "airmap/distortion.h"
#include "airmap/stitcher_configuration.h"

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
class Camera {
public:
    using ConfigurationCb = std::function<const Configuration(
        const Configuration &configuration, StitchType stitchType)>;

    enum class FOVUnits { Degrees, Radians };

    /**
     * @brief Camera
     * Create a camera.
     */
    Camera(
        double _focal_length_meters, cv::Point2d _sensor_dimensions_meters,
        cv::Point2d _sensor_dimensions_pixels, cv::Point2d _principal_point,
        std::shared_ptr<cv::Mat> _calibration_intrinsics = nullptr,
        std::shared_ptr<DistortionModel> _distortion_model = nullptr,
        ConfigurationCb configurationCb = [](const Configuration &configuration,
                                             StitchType stitchType) {
            return configuration;
        });

    /**
     * @brief Camera
     * Create a camera.
     */
    Camera();

    /**
     * @brief calibration_intrinsics;
     * Intrinsic matrix from calibration.
     */
    std::shared_ptr<cv::Mat> calibration_intrinsics;

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
     * @brief configuration
     * If the configuration callback is set, return the result of
     * calling it with the provided configuration and stitch type.
     */
    const Configuration configuration(const Configuration &configuration,
                                      StitchType stitchType) const;

    /**
     * @brief focalLengthPixels
     * Focal length in pixels.
     */
    cv::Point2d focalLengthPixels() const;

    /**
     * @brief fov
     * Horizontal and vertical fields of view.
     * @params units - degrees or radians
     */
    cv::Point2d fov(FOVUnits units = FOVUnits::Radians) const;

    /**
     * @brief Intrinsics matrix.
     * @param scale
     */
    cv::Mat K(double scale = 1.0) const;

private:
    ConfigurationCb _configurationCb;
};

} // namespace stitcher
} // namespace airmap
