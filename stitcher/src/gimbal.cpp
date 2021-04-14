#include "airmap/gimbal.h"

#include <opencv2/core/utility.hpp>

namespace airmap {
namespace stitcher {

GimbalOrientation::GimbalOrientation(double _pitch, double _roll, double _yaw,
                                     Units _units)
    : pitch(_pitch)
    , roll(_roll)
    , yaw(_yaw)
    , units(_units)
{
}

GimbalOrientation::GimbalOrientation(const GimbalOrientation &other)
    : pitch(other.pitch)
    , roll(other.roll)
    , yaw(other.yaw)
    , units(other.units)
{
}

GimbalOrientation GimbalOrientation::convertTo(Units _units)
{
    if (_units == units) {
        return *this;
    }

    switch (_units) {
    case Units::Degrees:
        return GimbalOrientation(pitch * 180.0 / M_PI, roll * 180.0 / M_PI,
                                 yaw * 180.0 / M_PI, Units::Degrees);
    case Units::Radians:
        return GimbalOrientation(pitch * M_PI / 180.0, roll * M_PI / 180.0,
                                 yaw * M_PI / 180.0, Units::Radians);
    }
}

cv::Mat GimbalOrientation::homography(cv::Mat K)
{
    cv::Mat R = rotationMatrix();
    cv::Mat H = K * R * K.inv();

    // normalize
    H /= H.at<double>(2, 2);

    return H;
}

cv::Mat GimbalOrientation::rotationMatrix()
{
    GimbalOrientation gimbal_orientation = convertTo(Units::Radians);

    double x = -gimbal_orientation.pitch;
    double y = -gimbal_orientation.yaw;
    double z = -gimbal_orientation.roll;

    cv::Mat Rx =
            (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(x), -sin(x), 0, sin(x), cos(x));

    cv::Mat Ry =
            (cv::Mat_<double>(3, 3) << cos(y), 0, sin(y), 0, 1, 0, -sin(y), 0, cos(y));

    cv::Mat Rz =
            (cv::Mat_<double>(3, 3) << cos(z), -sin(z), 0, sin(z), cos(z), 0, 0, 0, 1);

    cv::Mat R = Rx * Ry.inv() * Rz;
    R.at<double>(1, 1) *= -1;
    R.at<double>(2, 1) *= -1;
    R.at<double>(3, 1) *= -1;
    return R;
}

cv::Mat GimbalOrientation::rotateTo(GimbalOrientation &to)
{
    cv::Mat R1 = rotationMatrix();
    cv::Mat R2 = to.rotationMatrix();
    cv::Mat R = R1 * R2.t();
    return R;
}

std::string GimbalOrientation::toString(bool compact)
{
    if (compact) {
        return "[" + std::to_string(pitch) + ", " + std::to_string(roll) + ", "
                + std::to_string(yaw) + "]";
    } else {
        return "pitch: " + std::to_string(pitch) + " roll: " + std::to_string(roll)
                + " yaw: " + std::to_string(yaw);
    }
}

} // namespace stitcher
} // namespace airmap
