#include "airmap/camera_models.h"

namespace airmap {
namespace stitcher {

Camera::Camera(double _focal_length_meters,
               cv::Point2d _sensor_dimensions_meters,
               cv::Point2d _sensor_dimensions_pixels,
               cv::Point2d _principal_point,
               std::shared_ptr<cv::Mat> _calibration_intrinsics,
               std::shared_ptr<DistortionModel> _distortion_model,
               ConfigurationCb configurationCb)
    : calibration_intrinsics(_calibration_intrinsics)
    , distortion_model(std::move(_distortion_model))
    , focal_length_meters(_focal_length_meters)
    , principal_point(_principal_point)
    , sensor_dimensions_meters(_sensor_dimensions_meters)
    , sensor_dimensions_pixels(_sensor_dimensions_pixels)
    , _configurationCb(configurationCb)
{
}

Camera::Camera()
    : focal_length_meters(0)
    , principal_point(cv::Point2d(0, 0))
    , sensor_dimensions_meters(cv::Point2d(0, 0))
    , sensor_dimensions_pixels(cv::Point2d(0, 0))
{
}

const Configuration Camera::configuration(const Configuration &configuration,
                                          StitchType stitchType) const
{
    if (_configurationCb) {
        return _configurationCb(configuration, stitchType);
    }

    return configuration;
}

cv::Point2d Camera::focalLengthPixels() const
{
    double x = focal_length_meters * sensor_dimensions_pixels.x /
               sensor_dimensions_meters.x;
    double y = focal_length_meters * sensor_dimensions_pixels.y /
               sensor_dimensions_meters.y;
    return cv::Point2d(x, y);
}

cv::Point2d Camera::fov(FOVUnits units) const
{
    double x = 2 * atan(sensor_dimensions_meters.x / (2 * focal_length_meters));
    double y = 2 * atan(sensor_dimensions_meters.y / (2 * focal_length_meters));

    if (units == FOVUnits::Degrees) {
        x *= 180 / M_PI;
        y *= 180 / M_PI;
    }

    return cv::Point2d(x, y);
}

cv::Mat Camera::K(double scale) const
{
    if (calibration_intrinsics) {
        cv::Mat K;
        calibration_intrinsics->copyTo(K);
        K.at<double>(0, 0) *= scale;
        K.at<double>(0, 2) *= scale;
        K.at<double>(1, 1) *= scale;
        K.at<double>(1, 2) *= scale;
        return K;
    }

    cv::Point2d focal_length_pixels = focalLengthPixels();
    return (cv::Mat_<double>(3, 3) << focal_length_pixels.x * scale, 0,
            principal_point.x * scale, 0, focal_length_pixels.y * scale,
            principal_point.y * scale, 0, 0, 1);
}

} // namespace stitcher
} // namespace airmap
