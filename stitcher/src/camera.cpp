#include "airmap/camera_models.h"

#include <opencv2/core/utility.hpp>

namespace airmap {
namespace stitcher {

class Camera::Impl
{
public:
    using SharedPtr = std::shared_ptr<Impl>;

    Impl(cv::Point2d principal_point = cv::Point2d(0, 0),
         cv::Point2d sensor_dimensions_meters = cv::Point2d(0, 0),
         cv::Point2d sensor_dimensions_pixels = cv::Point2d(0, 0))
        : _principal_point(principal_point)
        , _sensor_dimensions_meters(sensor_dimensions_meters)
        , _sensor_dimensions_pixels(sensor_dimensions_pixels)
    {
    }

    static SharedPtr create(cv::Point2d principal_point = cv::Point2d(0, 0),
                            cv::Point2d sensor_dimensions_meters = cv::Point2d(0, 0),
                            cv::Point2d sensor_dimensions_pixels = cv::Point2d(0, 0))
    {
        return std::make_shared<Impl>(principal_point, sensor_dimensions_meters,
                                      sensor_dimensions_pixels);
    }

    cv::Point2d principalPoint() const { return _principal_point; }

    cv::Point2d sensorDimensionsMeters() const { return _sensor_dimensions_meters; }

    cv::Point2d sensorDimensionsPixels() const { return _sensor_dimensions_pixels; }

    void setPrincipalPoint(cv::Point2d principal_point)
    {
        _principal_point = principal_point;
    }

    void setSensorDimensionsMeters(cv::Point2d sensor_dimensions_meters)
    {
        _sensor_dimensions_meters = sensor_dimensions_meters;
    }

    void setSensorDimensionsPixels(cv::Point2d sensor_dimensions_pixels)
    {
        _sensor_dimensions_pixels = sensor_dimensions_pixels;
    }

private:
    /**
     * @brief principal_point
     * Principal point.
     */
    cv::Point2d _principal_point;

    /**
     * @brief sensor_dimensions_meters
     * Sensor dimensions in meters.
     */
    cv::Point2d _sensor_dimensions_meters;

    /**
     * @brief sensor_dimensions_pixels
     * Sensor dimensions in pixels.
     */
    cv::Point2d _sensor_dimensions_pixels;
};

Camera::Camera(double _focal_length_meters, cv::Point2d _sensor_dimensions_meters,
               cv::Point2d _sensor_dimensions_pixels, cv::Point2d _principal_point,
               std::shared_ptr<cv::Mat> _calibration_intrinsics,
               std::shared_ptr<DistortionModel> _distortion_model,
               ConfigurationCb configurationCb)
    : calibration_intrinsics(_calibration_intrinsics)
    , distortion_model(std::move(_distortion_model))
    , focal_length_meters(_focal_length_meters)
    , _configurationCb(configurationCb)
    , _impl(Impl::create(_principal_point, _sensor_dimensions_meters,
                         _sensor_dimensions_pixels))
{
}

Camera::Camera()
    : focal_length_meters(0)
    , _impl(Impl::create())
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
    cv::Point2d sensor_dimensions_meters = _impl->sensorDimensionsMeters();
    cv::Point2d sensor_dimensions_pixels = _impl->sensorDimensionsPixels();
    double x =
            focal_length_meters * sensor_dimensions_pixels.x / sensor_dimensions_meters.x;
    double y =
            focal_length_meters * sensor_dimensions_pixels.y / sensor_dimensions_meters.y;
    return cv::Point2d(x, y);
}

cv::Point2d Camera::fov(FOVUnits units) const
{
    cv::Point2d sensor_dimensions_meters = _impl->sensorDimensionsMeters();
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
    cv::Point2d principal_point = _impl->principalPoint();
    return (cv::Mat_<double>(3, 3) << focal_length_pixels.x * scale, 0,
            principal_point.x * scale, 0, focal_length_pixels.y * scale,
            principal_point.y * scale, 0, 0, 1);
}

cv::Point2d Camera::principalPoint() const
{
    return _impl->principalPoint();
}

cv::Point2d Camera::sensorDimensionsMeters() const
{
    return _impl->sensorDimensionsMeters();
}

cv::Point2d Camera::sensorDimensionsPixels() const
{
    return _impl->sensorDimensionsPixels();
}

} // namespace stitcher
} // namespace airmap
