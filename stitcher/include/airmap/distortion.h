#pragma once

#include "airmap/images.h"

#include "boost/filesystem.hpp"
#include "boost/format.hpp"

using boost::filesystem::path;

namespace airmap {
namespace stitcher {

/**
 * @brief DistortionModel
 * Abstract base class for distortion models.
 */
class DistortionModel
{
public:
    using CropROICb = std::function<const cv::Rect(const cv::Mat &image)>;
    DistortionModel(bool enabled_ = true, CropROICb crop_roi_cb_ = nullptr);
    virtual ~DistortionModel() = 0;

    /**
     * @brief crop
     * Crop a single image.
     */
    void crop(cv::Mat &image, cv::Rect &roi);

    /**
     * @brief crop
     * Crop multiple images.
     */
    void crop(std::vector<cv::Mat> &images);

    /**
     * @brief enabled
     * Whether undistortion is enabled.
     */
    bool enabled() const;

    /**
     * @brief undistort
     * Undistort multiple images.
     * @param images Images to undistort.
     * @param K Camera intrinsics matrix.
     */
    virtual void undistort(std::vector<cv::Mat> &images,
                           cv::InputArray K = cv::noArray()) = 0;

    /**
     * @brief undistort
     * Undistort a single image.
     * @param image Image to undistort.
     * @param K Camera intrinsics matrix.
     */
    virtual void undistort(cv::Mat &image, cv::InputArray K = cv::noArray()) = 0;

protected:
    bool _enabled;
    CropROICb _crop_roi_cb;
};

/**
 * @brief OpencCVDistortionModel
 * Pinhole distortion model.
 *
 * OpenCV undistort documentation:
 *  - https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html#ga69f2545a8b62a6b0fc2ee060dc30559d
 */
class PinholeDistortionModel : public DistortionModel
{
public:
    /**
     * @brief Parameters
     * Camera distortion coefficients from calibration.
     */
    struct Parameters
    {
        /**
         * @brief Parameters.
         * Create pinhole camera distortion parameters.
         * The order of parameters is consistent with OpenCV.
         */
        Parameters(double _k1, double _k2, double _p1, double _p2, double _k3);

        /**
         * @brief Parameters.
         * Create pinhole camera distortion parameters from a
         * coefficients vector.
         */
        Parameters(cv::Mat &coefficients_mat);

        /**
         * @brief Parameters.
         * Create pinhole camera distortion parameters from a
         * reference.
         */
        Parameters(const Parameters &parameters_);

        /**
         * @brief Parameters.
         * Create empty pinhole camera distortion parameters.
         */
        Parameters();

        /**
         * @brief coefficients
         * Coefficients vector.
         */
        cv::Mat coefficients() const;

        /**
         * @brief k1
         * k1 coefficient
         */
        double k1() const;

        /**
         * @brief k2
         * k2 coefficient
         */
        double k2() const;

        /**
         * @brief k3
         * k3 coefficient
         */
        double k3() const;

        /**
         * @brief p1
         * p1 coefficient
         */
        double p1() const;

        /**
         * @brief p2
         * p2 coefficient
         */
        double p2() const;

        /**
         * @brief matToArray
         * Convert coefficients cv::Mat to array.
         * @param coefficients_mat Coefficients column vector as cv::Mat
         */
        static std::array<double, 5> matToArray(cv::Mat &coefficients_mat);

        /**
         * @brief operator==
         * Whether the given Parameters object is equal to the current.
         * @param other Parameters reference to compare to.
         */
        bool operator==(const Parameters &other) const;

        /**
         * @brief operator==
         * Whether the given Parameters object is equal to the current.
         * @param other Parameters pointer to compare to.
         */
        bool operator==(const Parameters *other) const;

    protected:
        /**
         * @brief
         * Pinhole distortion coefficients as array.
         */
        std::array<double, 5> _coefficients;
    };

    /**
     * @brief PinholeDistortionModel
     * Create a pinhole distortion model from the given parameters.
     * @param parameters_ Distortion model parameters
     * @param enabled_ Whether the model is enabled (enabling undistortion).
     */
    PinholeDistortionModel(const Parameters parameters_, bool enabled_ = true,
                           CropROICb crop_roi_cb_ = nullptr);

    /**
     * @brief undistort
     * Undistort multiple images.
     * @param images Images to undistort.
     * @param K Camera intrinsics matrix.
     */
    void undistort(std::vector<cv::Mat> &images,
                   cv::InputArray K = cv::noArray());

    /**
     * @brief undistort
     * Undistort a single image.
     * @param image Image to undistort.
     * @param K Camera intrinsics matrix.
     */
    void undistort(cv::Mat &image, cv::InputArray K = cv::noArray());

protected:
    /**
     * @brief _parameters
     * Distortion model parameters.
     */
    const Parameters _parameters;
};

/**
 * @brief operator<<
 * Representation of pinhole parameters object as stream.
 * @param os ostream reference.
 * @param parameters Parameters reference.
 */
std::ostream& operator<<(std::ostream &os,
                         const PinholeDistortionModel::Parameters &parameters);

/**
 * @brief Scaramuzza
 * Scaramuzza distortion model.
 *
 * createPerspectiveUndistortionMaps and worldToCamera are ported from
 * code provided with the toolbox.
 *
 * Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich
 * Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
 *
 * More information: https://sites.google.com/site/scarabotix/ocamcalib-toolbox
 */
class ScaramuzzaDistortionModel : public DistortionModel
{
public:
    /**
     * @brief Parameters
     * Holds coefficients and other parameters from Scaramuzza calibration.
     */
    struct Parameters
    {
        /**
         * @brief pol
         * Polynomial coefficients.
         */
        std::vector<double> pol;

        /**
         * @brief inv_pol
         * Inverse polynomial coefficients.
         */
        std::vector<double> inv_pol;

        /**
         * @brief xc
         * x center pixels.
         */
        double xc;

        /**
         * @brief yc
         * y center in pixels.
         */
        double yc;

        /**
         * @brief c
         * Affine transformation parameter.
         */
        double c;

        /**
         * @brief d
         * Affine transformation parameter.
         */
        double d;

        /**
         * @brief e
         * Affine transformation parameter.
         */
        double e;

        /**
         * @brief width
         * Image width.
         */
        int width;

        /**
         * @brief height
         * Image height.
         */
        int height;

        /**
         * @brief scale_factor
         * Acts as a zoom/crop.
         */
        double scale_factor;

        /**
         * @brief resolution_scale
         * Adjust for calibration parameters from a different resolution.
         * For example, if the calibration is based on video frames at
         * half the resolution of the images to be undistorted, set
         * this to 2, as .
         */
        double resolution_scale;

        /**
         * @brief Parameters
         * Create Scaramuzza distortion model parameters.
         * @param _pol Polynomial coefficients.
         * @param _inv_pol Inverse polynomial coefficients.
         * @param _xc x center pixels.
         * @param _yc y center in pixels.
         * @param _c Affine transformation parameter. 
         * @param _d Affine transformation parameter.
         * @param _e Affine transformation parameter.
         * @param _width Image width.
         * @param _height Image height.
         * @param _scale_factor Acts as a zoom/crop.
         * @param _resolution_scale Adjust for calibration parameters from a
         * different resolution.
         */
        Parameters(std::vector<double> _pol, std::vector<double> _inv_pol,
                   double _xc, double _yc, double _c, double _d, double _e,
                   int _width, int _height, double _scale_factor,
                   double _resolution_scale);

        /**
         * @brief Parameters
         * Create empty Scaramuzza distortion model parameters.
         */
        Parameters();

    };

    /**
     * @brief ScaramuzzaDistortionModel
     * @param parameters_ Camera distortion coefficients from calibration.
     * @param enabled_ Whether undistortion is enabled.
     */
    ScaramuzzaDistortionModel(const Parameters parameters_,
                              bool enabled_ = true,
                              CropROICb crop_roi_cb_ = nullptr);

    /**
     * @brief createPerspectiveUndistortionMaps
     * Create x and y undistortion maps.
     * @param map_x x map
     * @param map_y y map
     */
    void createPerspectiveUndistortionMaps(cv::Mat &map_x, cv::Mat &map_y);

    /**
     * @brief undistort
     * Undistort a single image.
     * @param image Image to undistort.
     * @param K Camera intrinsics matrix.
     */
    void undistort(cv::Mat &image, cv::InputArray K = cv::noArray());

    /**
     * @brief undistort
     * Undistort multiple images.
     * @param images Images to undistort.
     * @param K Camera intrinsics matrix.
     */
    void undistort(std::vector<cv::Mat> &images,
                   cv::InputArray K = cv::noArray());

    /**
     * @brief worldToCamera
     * Projects a 3D world point onto the image.
     * @param world_point 3D world coordinates to be projected onto the
     * camera sensor frame.
     * @param camera_point 2D image coordinates of the projection.
     */
    void worldToCamera(cv::Point3d &world_point, cv::Point2d &camera_point);

protected:
    /**
     * @brief _parameters
     * Scaramuzza distortion model parameters.
     */
    const Parameters _parameters;
};

} // namespace stitcher
} // namespace airmap
