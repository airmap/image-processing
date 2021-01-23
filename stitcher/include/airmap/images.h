#pragma once

#include <boost/format.hpp>

#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/stitching.hpp>

#include "gimbal.h"
#include "logger.h"
#include "panorama.h"

using namespace airmap::logging;

namespace airmap {
namespace stitcher {

/**
 * @brief SourceImages
 * A struct to contain and manage source images.
 */
struct SourceImages
{
    /**
     * @brief panorama
     * Source image paths and metadata.
     */
    const Panorama &panorama;

    /**
     * @brief gimbal_orientations
     * Gimbal orientation.
     */
    std::vector<GimbalOrientation> gimbal_orientations;

    /**
     * @brief images
     * The loaded images.
     */
    std::vector<cv::Mat> images;

    /**
     * @brief sizes
     * Sizes of the original images (but changed after warping).
     */
    std::vector<cv::Size> sizes;

    /**
     * @brief logger
     */
    std::shared_ptr<logging::Logger> _logger;

    /**
     * @brief minimumImageCount
     * The minimum number of images.  Used by ensureImageCount.
     */
    int minimumImageCount;

    /**
     * @brief SourceImages
     * @param panorama Source image paths and metadata.
     */
    SourceImages(const Panorama &panorama, std::shared_ptr<logging::Logger> logger,
                 const int _minimumImageCount = 2)
        : panorama(panorama)
        , images()
        , sizes()
        , _logger(logger)
        , minimumImageCount(_minimumImageCount)
    {
        resize(static_cast<size_t>(panorama.size()));
        load();
        ensureImageCount();
    }

    /**
     * @brief clear
     * Clear all storage.
     */
    void clear();

    /**
     * @brief ensureImageCount
     * Throws if there are less than 2 images.
     * @throws std::invalid_argument
     */
    void ensureImageCount();

    /**
     * @brief filter
     * Remove images not in keep_indices.
     * @param keep_indices
     */
    void filter(std::vector<int> &keep_indices);

    /**
     * @brief load
     * Open images and load associated metadata.
     */
    void load();

    /**
     * @brief reload
     * Reload original images.
     */
    void reload();

    /**
     * @brief resize
     * Resize storage vectors.
     * @param new_size
     */
    void resize(size_t new_size);

    /**
     * @brief scale
     * Scale images in place.
     * @param scale
     * @param interpolation
     */
    void scale(double scale, int interpolation = cv::INTER_LINEAR_EXACT);
};

} // namespace stitcher
} // namespace airmap
