#pragma once

#include <cmath>
#include <string>

#include "airmap/opencv/forward.h"

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

    GimbalOrientation(double _pitch = 0.0, double _roll = 0.0, double _yaw = 0.0,
                      Units _units = Units::Degrees);

    GimbalOrientation(const GimbalOrientation &other);

    /**
     * @brief convertTo
     * Convert unit of angles.
     * @params _units - degrees or radians
     */
    GimbalOrientation convertTo(Units _units);

    /**
     * @brief homography
     * Calculate the homography for the current rotation
     * and the given camera intrinsics matrix.
     */
    cv::Mat homography(cv::Mat K);

    /**
     * @brief rotationMatrix
     * Calculate the rotation matrix for the given pose.
     * This is not a normal rotation sequence, since it's also
     * converting between camera and vehicle frames.  There is
     * definitely room for improvement here.
     */
    cv::Mat rotationMatrix();

    /**
     * @brief rotateTo
     * Calculate the rotation between the current pose
     * to the given pose.
     * NOTE(bkd): Ideally, this would return a GimbalOrientation,
     * but need to implement reverse of rotation matrix to Euler angles.
     * @param to - pose to rotate to
     */
    cv::Mat rotateTo(GimbalOrientation &to);

    /**
     * @brief toString
     * Returns a string for debugging.
     * @param compact
     */
    std::string toString(bool compact = false);
};

} // namespace stitcher
} // namespace airmap
