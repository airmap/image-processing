#pragma once

#include <opencv2/core/utility.hpp>

namespace airmap {
namespace stitcher {

/**
 * @brief GimbalOrientation
 * Contains gimbal orientation for an image, as well
 * as rotation and homography calculations.
 */
struct GimbalOrientation
{
    enum class Units { Degrees, Radians };

    double pitch;
    double roll;
    double yaw;
    Units units;

    GimbalOrientation(double _pitch = 0.0, double _roll = 0.0, double _yaw = 0.0, Units _units = Units::Degrees)
        : pitch(_pitch), roll(_roll), yaw(_yaw), units(_units)
    {
    }

    GimbalOrientation(const GimbalOrientation &other)
        : pitch(other.pitch), roll(other.roll), yaw(other.yaw), units(other.units)
    {
    }

    /**
     * @brief convertTo
     * Convert unit of angles.
     * @params _units - degrees or radians
     */
    GimbalOrientation convertTo(Units _units)
    {
        if (_units == units) {
            return *this;
        }

        switch (_units) {
        case Units::Degrees:
            return GimbalOrientation(pitch * 180.0/M_PI, roll * 180.0/M_PI, yaw * 180.0/M_PI, Units::Degrees);
        case Units::Radians:
            return GimbalOrientation(pitch * M_PI/180.0, roll * M_PI/180.0, yaw * M_PI/180.0, Units::Radians);
        }
    }

    /**
     * @brief homography
     * Calculate the homography for the current rotation
     * and the given camera intrinsics matrix.
     */
    cv::Mat homography(cv::Mat K)
    {
        cv::Mat R = rotationMatrix();
        cv::Mat H = K * R * K.inv();

        // normalize
        H /= H.at<double>(2, 2);

        return H;
    }

    /**
     * @brief rotationMatrix
     * Calculate the rotation matrix for the given pose.
     * This is not a normal rotation sequence, since it's also
     * converting between camera and vehicle frames.  There is
     * definitely room for improvement here.
     */
    cv::Mat rotationMatrix()
    {
        GimbalOrientation gimbal_orientation = convertTo(Units::Radians);

        double x = -gimbal_orientation.pitch;
        double y = -gimbal_orientation.yaw;
        double z = -gimbal_orientation.roll;

        cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
            1, 0, 0,
            0, cos(x), -sin(x),
            0, sin(x), cos(x)
        );

        cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
            cos(y), 0, sin(y),
            0, 1, 0,
            -sin(y), 0, cos(y)
        );

        cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
            cos(z), -sin(z), 0,
            sin(z), cos(z), 0,
            0, 0, 1
        );

        cv::Mat R = Rx * Ry.inv() * Rz;
        R.at<double>(1, 1) *= -1;
        R.at<double>(2, 1) *= -1;
        R.at<double>(3, 1) *= -1;
        return R;
    }

    /**
     * @brief rotateTo
     * Calculate the rotation between the current pose
     * to the given pose.
     * NOTE(bkd): Ideally, this would return a GimbalOrientation,
     * but need to implement reverse of rotation matrix to Euler angles.
     * @param to - pose to rotate to
     */
    cv::Mat rotateTo(GimbalOrientation &to)
    {
        cv::Mat R1 = rotationMatrix();
        cv::Mat R2 = to.rotationMatrix();
        cv::Mat R = R1 * R2.t();
        return R;
    }

    /**
     * @brief toString
     * Returns a string for debugging.
     * @param compact
     */
    std::string toString(bool compact = false)
    {
        if (compact) {
            return "[" + std::to_string(pitch) + ", " + std::to_string(roll) + ", "
                    + std::to_string(yaw) + "]";
        } else {
            return "pitch: " + std::to_string(pitch) + " roll: "
                    + std::to_string(roll) + " yaw: " + std::to_string(yaw);
        }
    }
};

} // namespace stitcher
} // namespace airmap
