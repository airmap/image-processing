#pragma once

#include "airmap/camera.h"
#include "airmap/camera_models.h"
#include "airmap/distortion.h"
#include "airmap/gimbal.h"
#include "airmap/images.h"
#include "airmap/logging.h"
#include "airmap/monitor/estimator.h"
#include "airmap/opencv/forward.h"
#include "airmap/opencv/seam_finders.h"
#include "airmap/stitcher.h"
#include "airmap/stitcher_configuration.h"

#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/stitching/detail/autocalib.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/timelapsers.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/stitching/warpers.hpp>

namespace airmap {
namespace stitcher {

/**
 * @brief The OpenCVStitcher performs basic, same callstack stitching using OpenCV
 * high level stitching API.
 */
class OpenCVStitcher : public OperationsMonitoredStitcher
{
public:
    OpenCVStitcher(
            const Panorama &panorama, const Panorama::Parameters &parameters,
            const std::string &outputPath,
            std::shared_ptr<airmap::logging::Logger> logger,
            monitor::Estimator::UpdatedCb updatedCb = []() {}, bool debug = false,
            path debugPath = path("debug"));

    Report stitch() override;
    void cancel() override;
    void postprocess(cv::Mat&& result);
    void setFallbackMode() override;
    void setUseOpenCL(bool enabled = true);

protected:
    bool _debug;
    path _debugPath;
    std::shared_ptr<airmap::logging::Logger> _logger;
    Panorama _panorama;
    Panorama::Parameters _parameters;
    std::string _outputPath;
};

/**
 * @brief stitcher class
 * Provides a generic stitcher that can produce a variety of different
 * types of panoramas, depending on configuration.
 * We currently only use this for stitching 360 panoramas, but I've
 * tried to keep this abstract.
 * This is a refactor of the sample code from opencv:
 * https://github.com/opencv/opencv/blob/4.2.0/samples/cpp/stitching_detailed.cpp
 */
class LowLevelOpenCVStitcher : public OpenCVStitcher
{
public:
    /**
     * @brief WarpResults
     * Contains the results of the warping operation.
     */
    struct WarpResults
    {
        //! Top left corners of projected image.
        std::vector<cv::Point> corners;
        //! Warped masks used for finding seams and blending.
        std::vector<cv::UMat> masks_warped;
        //! Warped images used for blending.
        std::vector<cv::UMat> images_warped;
        //! Float versions of warped images for finding seams.
        std::vector<cv::UMat> images_warped_f;
        //! Sizes of the warped images.
        std::vector<cv::Size> sizes;
        //! Masks used for warping images.
        std::vector<cv::UMat> masks;

        /**
         * @brief WarpResults
         * Create an instance of warp results for the given number of images.
         * @param image_count
         */
        WarpResults(size_t image_count)
            : corners(image_count)
            , masks_warped(image_count)
            , images_warped(image_count)
            , images_warped_f(image_count)
            , sizes(image_count)
            , masks(image_count)
        {
        }
    };

    /**
     * @brief Stitcher
     * Create an instance of the stitcher with the given configuration.
     * @param config
     */
    LowLevelOpenCVStitcher(
            const Configuration &config, const Panorama &panorama,
            const Panorama::Parameters &parameters, const std::string &outputPath,
            std::shared_ptr<airmap::logging::Logger> logger,
            monitor::Estimator::UpdatedCb updatedCb = []() {}, bool debug = false,
            path debugPath = path("debug"));

    Report stitch() override;
    void cancel() override;

private:
    /**
     * @brief _config
     * The stitcher configuration.
     */
    const Configuration _config;

    /**
     * @brief stitch
     * Stitch the input images into a panorama.
     * @param result
     */
    Stitcher::Report stitch(cv::Mat &result);

    /**
     * @brief adjustCameraParameters
     * Refine the estimated camera parameters through bundle adjustmnt.
     * @param features
     * @param matches
     * @param cameras
     */
    void adjustCameraParameters(std::vector<cv::detail::ImageFeatures> &features,
                                std::vector<cv::detail::MatchesInfo> &matches,
                                std::vector<cv::detail::CameraParams> &cameras);

    /**
     * @brief compose
     * Compose the warped images into the final panorama.
     * @param source_images
     * @param cameras
     * @param exposure_compensator
     * @param warp_results
     * @param work_scale
     * @param warped_image_scale
     * @param result
     */
    void compose(SourceImages &source_images,
                 std::vector<cv::detail::CameraParams> &cameras,
                 cv::Ptr<cv::detail::ExposureCompensator> &exposure_compensator,
                 WarpResults &warp_results, double work_scale,
                 double compose_scale, float warped_image_scale, cv::Mat &result);

    /**
     * @brief debugFeatures
     * Draw features on source images and save the results.
     * @param source_images
     * @param features
     * @param flags
     */
    void debugFeatures(SourceImages &source_images,
                       std::vector<cv::detail::ImageFeatures> &features,
                       cv::DrawMatchesFlags flags = cv::DrawMatchesFlags::DEFAULT);

    /**
     * @brief debugImages
     * Save images in current state for debugging.
     * @param source_images
     * @param path
     */
    void debugImages(std::vector<cv::Mat> &images, path debug_path);

    /**
     * @brief debugMatches
     * Draw feature matches on source images and save the results.
     * @param source_images
     * @param features
     * @param matches
     * @param conf_threshold
     * @param flags
     */
    void debugMatches(SourceImages &source_images,
                      std::vector<cv::detail::ImageFeatures> &features,
                      std::vector<cv::detail::MatchesInfo> &matches,
                      float conf_threshold,
                      cv::DrawMatchesFlags flags = cv::DrawMatchesFlags::DEFAULT);

    /**
     * @brief debugWarpResults
     * Save warp result images.
     * @param warp_results
     */
    void debugWarpResults(WarpResults &warp_results);

    /**
     * @brief estimateCameraParameters
     * Takes features of all images, pairwise matches between all images, and
     * estimates rotations between camera frames.
     * @param features
     * @param matches
     * @return
     */
    std::vector<cv::detail::CameraParams>
    estimateCameraParameters(std::vector<cv::detail::ImageFeatures> &features,
                             std::vector<cv::detail::MatchesInfo> &matches);

    /**
     * @brief findFeatures
     * Find features in the source images.  Scale images to work_scale first.
     * @param source_images
     * @return
     */
    std::vector<cv::detail::ImageFeatures> findFeatures(SourceImages &source_images);

    /**
     * @brief findMedianFocalLength
     * Find the median focal length from estimations.
     * @param cameras
     * @return
     */
    double findMedianFocalLength(std::vector<cv::detail::CameraParams> &cameras);

    /**
     * @brief findSeams
     * Estimate seams from the warp results and populate corner estimates.
     * @param warp_results
     */
    void findSeams(WarpResults &warp_results);

    /**
     * @brief getBundleAdjuster
     * Create and return a bundle adjuster according to configuration.
     * @return
     */
    cv::Ptr<cv::detail::BundleAdjusterBase> getBundleAdjuster();

    /**
     * @brief getComposeScale
     * Determine compose scale from source image sizes and _config.compose_megapix.
     * @param source_images
     * @return
     */
    double getComposeScale(SourceImages &source_images);

    /**
     * @brief getEstimator
     * Create and return a camera rotation estimator according to configuration..
     * @return
     */
    cv::Ptr<cv::detail::Estimator> getEstimator();

    /**
     * @brief getExposureCompensator
     * Create and return an exposure compensator according to configuration.
     * @return
     */
    cv::Ptr<cv::detail::ExposureCompensator> getExposureCompensator();

    /**
     * @brief getFeaturesFinder
     * Create and return a feature finder according to configuration.
     * @return
     */
    cv::Ptr<cv::Feature2D> getFeaturesFinder();

    /**
     * @brief getFeaturesMatcher
     * Create and return a feature matcher according to configuration.
     * @return
     */
    cv::Ptr<cv::detail::FeaturesMatcher> getFeaturesMatcher();

    /**
     * @brief getSeamFinder
     * Create and return a seam finder/estimator according to configuration.
     * @return
     */
    cv::Ptr<cv::detail::SeamFinder> getSeamFinder();

    /**
     * @brief getSeamScale
     * Determine seam scale from source image sizes and _config.seam_megapix.
     * @param source_images
     * @return
     */
    double getSeamScale(SourceImages &source_images);

    /**
     * @brief getWarperCreator
     * Create and return a warper creator according to configuration.
     * @return
     */
    cv::Ptr<cv::WarperCreator> getWarperCreator();

    /**
     * @brief getWaveCorrect
     * Determine wave correct type from configuration.
     * @return
     */
    cv::detail::WaveCorrectKind getWaveCorrect();

    /**
     * @brief getWorkScale
     * Determine work scale from image sizes and _config.work_megapix.
     * @param source_images
     * @return
     */
    double getWorkScale(SourceImages &source_images);

    /**
     * @brief matchFeatures
     * Matches features seen in multiple images and populates pairwise matches.
     * @param features
     * @return
     */
    std::vector<cv::detail::MatchesInfo>
    matchFeatures(std::vector<cv::detail::ImageFeatures> &features);

    /**
     * @brief prepareBlender
     * Create blender according to configuration and feed it the warp results.
     * @param warp_results
     * @return
     */
    cv::Ptr<cv::detail::Blender> prepareBlender(WarpResults &warp_results);

    /**
     * @brief prepareExposureCompensation
     * Get exposure compensator and feed it the warp results in preparation for
     * composition.
     * @param warp_results
     * @return
     */
    cv::Ptr<cv::detail::ExposureCompensator>
    prepareExposureCompensation(WarpResults &warp_results);

    /**
     * @brief undistortImages
     * Optionally undistort the images, depending on whether the
     * camera model can be identified, and is required for that camera.
     * @param source_images Source images object.
     */
    void undistortImages(SourceImages &source_images);

    /**
     * @brief undistortCropImages
     * Optionally crop images based on the distortion model.
     * camera model can be identified, and is required for that camera.
     * This happens after feature matching and bundle adjustment so that the
     * entire frame can be used for computing the homography before cropping
     * removes areas where there are good features.
     * @param source_images Source images object.
     */
    void undistortCropImages(SourceImages &source_images);

    /**
     * @brief warpImages
     * Warp images using the estimated/refined camera intrinsics and rotations.
     * @param source_images
     * @param cameras
     * @param warped_image_scale
     * @param seam_work_aspect
     * @return
     */
    WarpResults warpImages(SourceImages &source_images,
                           std::vector<cv::detail::CameraParams> &cameras,
                           float warped_image_scale, float seam_work_aspect);

    /**
     * @brief waveCorrect
     * Perform wave correction on the estimated camera rotations.
     * @param cameras
     */
    void waveCorrect(std::vector<cv::detail::CameraParams> &cameras);
};

} // namespace stitcher
} // namespace airmap
