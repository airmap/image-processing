#pragma once

#include <map>

#include "airmap/camera.h"
#include "airmap/panorama.h"

namespace airmap {
namespace stitcher {

/**
 * @brief CameraModels
 * Canned camera modelss.
 */
struct CameraModels
{
    std::map<std::string, Camera> models;

    CameraModels();

    /**
     * @brief detect
     * Detect the camera model and return a Camera instance.
     * @param image
     */
    std::shared_ptr<Camera> detect(const GeoImage &image)
    {
        for (const auto &model : models) {
            if (image.cameraModel == model.first) {
                return std::make_shared<Camera>(model.second);
            }
        }

        return {};
    }

    /**
     * @brief ParrotAnafiThermal Camera
     * Parrot Anafi Thermal camera information.
     */
    static Camera ParrotAnafiThermal(bool distortion_model_enabled = false);

    /**
     * @brief VantageVesperEONavigation Camera
     * Vantage Vesper EO Navigation camera information.
     */
    static Camera VantageVesperEONavigation();
};

} // namespace stitcher
} // namespace airmap
