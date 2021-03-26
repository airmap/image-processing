#pragma once

#include "airmap/camera.h"
#include "airmap/distortion.h"
#include "airmap/panorama.h"

#include <boost/optional.hpp>

namespace airmap {
namespace stitcher {

/**
 * @brief CameraModels
 * Canned camera modelss.
 */
struct CameraModels
{
    std::map<std::string, Camera> models;

    CameraModels()
    {
        models["AnafiThermal"] = ParrotAnafiThermal();
        models["GreenSeer EO Navigation Lens"] = VantageVesperEONavigation();
    }

    /**
     * @brief detect
     * Detect the camera model and return a Camera instance.
     * @param image
     */
    boost::optional<Camera> detect(const GeoImage &image)
    {
        for (const auto &model : models) {
            if (image.cameraModel == model.first) {
                return model.second;
            }
        }

        return boost::optional<Camera>();
    }

    /**
     * @brief ParrotAnafiThermal Camera
     * Parrot Anafi Thermal camera information.
     */
    static Camera ParrotAnafiThermal(bool distortion_model_enabled = false)
    {
        double focal_length_meters = 4.04e-3;
        cv::Point2d sensor_dimensions_meters(7.22e-3, 5.50e-3);
        cv::Point2d sensor_dimensions_pixels(5344, 4016);
        cv::Point2d principal_point(sensor_dimensions_pixels.x / 2,
                                    sensor_dimensions_pixels.y / 2);

        auto distortion_parameters = PinholeDistortionModel::Parameters(
            2.8391010208309218e-02, -2.7239202041003809e-02,
            -2.4700935014356916e-03, 6.1345950301455029e-03, 0.0);
        auto distortion_model = std::make_shared<PinholeDistortionModel>(
            distortion_parameters, distortion_model_enabled);

        cv::Mat calibration_intrinsics = (cv::Mat_<double>(3, 3) <<
            2.9803291905661031e+03, 0.0, 2.3574166587045511e+03,
            0.0, 2.9704815213723482e+03, 1.6709805782521339e+03,
            0.0, 0.0, 1.0);

        return Camera(focal_length_meters, sensor_dimensions_meters,
                      sensor_dimensions_pixels, principal_point,
                      calibration_intrinsics, distortion_model);
    }

    /**
     * @brief VantageVesperEONavigation Camera
     * Vantage Vesper EO Navigation camera information.
     */
    static Camera VantageVesperEONavigation() {
        double focal_length_meters = 6.00e-3;
        cv::Point2d sensor_dimensions_meters(7.68e-3, 4.32e-3);
        cv::Point2d sensor_dimensions_pixels(3840, 2160);
        cv::Point2d principal_point(sensor_dimensions_pixels.x / 2,
                                    sensor_dimensions_pixels.y / 2);

        std::vector<double> pol = { -2.863252e+03, 0.000000e+00, 4.060925e-04, -2.040622e-07, 5.342978e-11 };
        std::vector<double>inv_pol = { -2319.757678, -49354.733874, -202871.366882, -464932.147854, -669902.465355, -625830.239054, -379865.964545, -145131.700716, -31852.275059, -3077.921163 };
        double xc = 2061.334454;
        double yc = 1141.123668;
        double c = 0.999510;
        double d = -0.000090;
        double e = 0.000028;
        double width = 3840;
        double height = 2160;
        double scale_factor = 1.5;
        double resolution_scale = 1;

        auto distortion_parameters = ScaramuzzaDistortionModel::Parameters(
            pol, inv_pol, xc, yc, c, d, e, width, height, scale_factor,
            resolution_scale);
        auto distortion_model = std::make_shared<ScaramuzzaDistortionModel>(
            distortion_parameters, true, [height](const cv::Mat &image) -> cv::Rect {
                return cv::Rect(cv::Point(0, 0), cv::Point(image.size().width,
                                (2083./height) * image.size().height));
            });

        Camera::ConfigurationCb configurationCb = [](
            const Configuration &configuration, StitchType stitchType) {
                Configuration updatedConfiguration = configuration;
                if (stitchType == StitchType::ThreeSixty) {
                    updatedConfiguration.match_conf = 0.45f;
                }
                return updatedConfiguration;
        };

        return Camera(focal_length_meters, sensor_dimensions_meters,
                      sensor_dimensions_pixels, principal_point,
                      boost::optional<cv::Mat>(),
                      distortion_model, configurationCb);
    }
};

} // namespace stitcher
} // namespace airmap
