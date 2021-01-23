#pragma once

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

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

#include "camera.h"
#include "camera_models.h"
#include "distortion.h"
#include "gimbal.h"
#include "images.h"
#include "logger.h"
#include "stitcher.h"

using boost::filesystem::path;

namespace airmap {
namespace stitcher {

/**
 * @brief The OpenCVStitcher performs basic, same callstack stitching using OpenCV
 * high level stitching API.
 */
class OpenCVStitcher : public Stitcher {
public:
    OpenCVStitcher(const Panorama &panorama,
                   const Panorama::Parameters &parameters,
                   const std::string &outputPath,
                   std::shared_ptr<Logger> logger)
        : _panorama(panorama)
        , _parameters(parameters)
        , _outputPath(outputPath)
        , _logger(logger)
    {
    }

    Report stitch() override;
    void cancel() override;
    void postprocess(cv::Mat&& result);

protected:
    Panorama _panorama;
    Panorama::Parameters _parameters;
    std::string _outputPath;
    std::shared_ptr<Logger> _logger;
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
    enum class BundleAdjusterType { Ray, Reproj, AffinePartial, No };
    enum class EstimatorType { Affine, Homography };
    enum class ExposureCompensatorType { Channels, ChannelsBlocks, Gain, GainBlocks, No };
    enum class FeaturesFinderType { Akaze, Orb, Sift, Surf };
    enum class FeaturesMatcherType { Affine, Homography };
    enum class SeamFinderType {
        DpColor,
        DpColorGrad,
        GraphCutColor,
        GraphCutColorGrad,
        No,
        Voronoi
    };
    enum class StitchType { ThreeSixty };
    enum class WaveCorrectType { Horizontal, Vertical };
    enum class WarperType {
        Affine,
        CompressedPlaneA2B1,
        CompressedPlaneA1_5B1,
        CompressedPlanePortraitA2B1,
        CompressedPlanePortraitA1_5B1,
        Cylindrical,
        Fisheye,
        Mercator,
        Planer,
        PaniniA2B1,
        PaniniA1_5B1,
        PaniniPortraitA2B1,
        PaniniPortraitA1_5B1,
        Spherical,
        Stereographic,
        TransverseMercator
    };

    /**
     * @brief Configuration struct
     * Specifies the type of panorama to create, the types of
     * different components (e.g. features finder, matcher, etc.),
     * as well as parameters for those components.
     */
    struct Configuration
    {
        /*!
         * This, along with image size, determines whether to use a simple blender
         * or one of the more complex blenders (e.g. multi-band or feather).
         * For a multi-band blender, it is used to calculate the number of bands.
         * For a feather blender, it is used to calculate the sharpness.
         */
        float blend_strength;

        //! The type of blender to use (e.g. multi-band or feather).
        int blender_type;

        /*!
         * The type of bundle adjuster (e.g. ray, reprojection) to use to refine
         * camera parameters after estimation.
         */
        BundleAdjusterType bundle_adjuster_type;

        //! Megapixels an image will be scaled down to for the composition step.
        double compose_megapix;

        //! Whether to create debug artifacts (e.g. feature detection, matching, warping, etc.)
        bool debug;

        //! Path to debug artifacts directory.
        path debug_path;

        /*!
         * The type of estimator (e.g. affine or homography) to use to estimate initial
         * camera parameters.
         */
        EstimatorType estimator_type;

        /*!
         * The type of exposure compensator (e.g. channels, gain, etc.) to use.
         * during blending.
         */
        ExposureCompensatorType exposure_compensator_type;

        //! TODO(bkd): I haven't yet read details of what this does. */
        int exposure_compensation_nr_feeds;

        //! TODO(bkd): I haven't yet read details of what this does. */
        int exposure_compensation_nr_filtering;

        //! TODO(bkd): I haven't yet read details of what this does. */
        int exposure_compensation_block_size;

        //! The type of features finder (e.g. ORB, SIFT, etc.) to use.
        FeaturesFinderType features_finder_type;

        /*!
         * The type of features matcher (e.g. affine, homography) to use
         * to match features shared between image pairs.
         */
        FeaturesMatcherType features_matcher_type;

        //! The maximum number of features that the features finder will keep.
        int features_maximum;

        /*!
         * A threshold given to the features matcher.  Two features are matched
         * if the ratio between descriptor distances is greater than this value.
         */
        float match_conf;

        /*!
         * A threshold given to the bundle adjuster.  A match is kept
         * if its confidence score is greater than this value.  Images with
         * too few matches with a confidence greater than this are filtered out
         * before estimating camera parameters.
         */
        double match_conf_thresh;

        /*!
         * If a homography features matcher is used, a value of -1 will
         * use a BestOf2NearestMatcher.  Otherwise, a BestOf2NearestRangeMatcher
         * is used and this value is provided as a parameter.
         * TODO(bkd): I've not read much about this, but it is worth looking into.
         * I believe that it's used as an alternativee to masking with other
         * features matchers.
         */
        int range_width;

        //! Megapixels an image will be scaled down to after finding features.
        double seam_megapix;

        /*!
         * The type of seam finder (e.g. GraphCut, GraphCustColorGrad, Voronoi,
         * etc.) to use to find seams between warped images.
         */
        SeamFinderType seam_finder_type;

        /*!
         * Various components have optional CUDA support.  If this is set to
         * true, then they will attempt to use CUDA versions.
         */
        bool try_cuda;

        /*!
         * The type of warper (e.g. affine, cylindrical, planer, spherical,
         * stereographic, etc.) to use.  This is used to warp the images according
         * to the associated projection algorithm.
         */
        WarperType warper_type;

        /*!
         * Specifies whether to perform wave correction on the estimated/refined
         * camera paraemters.
         */
        bool wave_correct;

        /*!
         * The type of wave correction (e.g. horizontal, vertical) to use.
         * When estimating relative rotations between images, the camera rotation
         * relative to the world is unknown.  There are assumptions made about
         * about how those relate to 3D world coordinates, which result in an
         * unnatural wave effect in the stitched image.  Assuming that there are
         * not significant rotations relative to the horizon (e.g. roll/bank),
         * the wave corrector attempts to use the null space in the stitch to
         * determine global rotation, which is used to modify the camera parameters
         * to remove the wave effect.
         */
        WaveCorrectType wave_correct_type;

        //! Megapixels an image will be scaled down to for features detection.
        double work_megapix;

        /**
         * @brief Configuration
         * Create a configuration from a set of defaults.
         * This currently only allows for choosing a default configuration
         * for 360 panoramas, but the intent is to allow for more canned
         * defaults in the future.
         * @param stitchType The stitching default to use.
         */
        Configuration(StitchType stitchType, bool _debug = false, path _debugPath = path("debug"))
        {
            switch (stitchType) {
            case StitchType::ThreeSixty:
                blend_strength = 5;
                blender_type = cv::detail::Blender::MULTI_BAND;
                bundle_adjuster_type = BundleAdjusterType::Ray;
                compose_megapix = 8;
                estimator_type = EstimatorType::Homography;
                exposure_compensator_type = ExposureCompensatorType::GainBlocks;
                exposure_compensation_nr_feeds = 1;
                exposure_compensation_nr_filtering = 2;
                exposure_compensation_block_size = 32;
                features_finder_type = FeaturesFinderType::Orb;
                features_matcher_type = FeaturesMatcherType::Homography;
                features_maximum = 1000;
                match_conf = 0.3f;
                match_conf_thresh = 1.0;
                range_width = -1;
                seam_megapix = 0.1;
                seam_finder_type = SeamFinderType::GraphCutColorGrad;
                try_cuda = false;
                warper_type = WarperType::Spherical;
                wave_correct = true;
                wave_correct_type = WaveCorrectType::Horizontal;
                work_megapix = 0.6;
                break;
            }

            debug = _debug;
            debug_path = _debugPath;
        }

        /**
         * @brief Configuration
         * Create a custom stitching configuration.
         * @param blend_strength
         * @param blender_type
         * @param bundle_adjuster_type
         * @param compose_megapix
         * @param debug
         * @param debug_path
         * @param estimator_type
         * @param exposure_compensator_type
         * @param exposure_compensation_nr_feeds
         * @param exposure_compensation_nr_filtering
         * @param exposure_compensation_block_size
         * @param features_finder_type
         * @param features_matcher_type
         * @param features_maximum
         * @param match_conf
         * @param match_conf_thresh
         * @param range_width
         * @param seam_megapix
         * @param seam_finder_type
         * @param try_cuda
         * @param warper_type
         * @param wave_correct
         * @param wave_correct_type
         * @param work_megapix
         */
        Configuration(float blend_strength, int blender_type,
                      BundleAdjusterType bundle_adjuster_type, double compose_megapix,
                      bool debug, path debug_path,
                      EstimatorType estimator_type,
                      ExposureCompensatorType exposure_compensator_type,
                      int exposure_compensation_nr_feeds,
                      int exposure_compensation_nr_filtering,
                      int exposure_compensation_block_size,
                      FeaturesFinderType features_finder_type,
                      FeaturesMatcherType features_matcher_type, int features_maximum,
                      float match_conf, double match_conf_thresh, int range_width,
                      double seam_megapix, SeamFinderType seam_finder_type, bool try_cuda,
                      WarperType warper_type, bool wave_correct,
                      WaveCorrectType wave_correct_type, double work_megapix)
            : blend_strength(blend_strength)
            , blender_type(blender_type)
            , bundle_adjuster_type(bundle_adjuster_type)
            , compose_megapix(compose_megapix)
            , debug(debug)
            , debug_path(debug_path)
            , estimator_type(estimator_type)
            , exposure_compensator_type(exposure_compensator_type)
            , exposure_compensation_nr_feeds(exposure_compensation_nr_feeds)
            , exposure_compensation_nr_filtering(exposure_compensation_nr_filtering)
            , exposure_compensation_block_size(exposure_compensation_block_size)
            , features_finder_type(features_finder_type)
            , features_matcher_type(features_matcher_type)
            , features_maximum(features_maximum)
            , match_conf(match_conf)
            , match_conf_thresh(match_conf_thresh)
            , range_width(range_width)
            , seam_megapix(seam_megapix)
            , seam_finder_type(seam_finder_type)
            , try_cuda(try_cuda)
            , warper_type(warper_type)
            , wave_correct(wave_correct)
            , wave_correct_type(wave_correct_type)
            , work_megapix(work_megapix)
        {
        }
    };

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
    LowLevelOpenCVStitcher(const Configuration &config,
            const Panorama &panorama,
            const Panorama::Parameters &parameters,
            const std::string &outputPath,
            std::shared_ptr<Logger> logger);

    Report stitch() override;
    void cancel() override;

private:
    const Configuration &config;

    /**
     * @brief stitch
     * Stitch the input images into a panorama.
     * @param panorama
     * @param result
     */
    void stitch(cv::Mat &result);

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
                 float warped_image_scale, cv::Mat &result);

    /**
     * @brief debugFeatures
     * Draw features on source images and save the results.
     * @param source_images
     * @param features
     * @param scale
     * @param flags
     */
    void debugFeatures(SourceImages &source_images,
                       std::vector<cv::detail::ImageFeatures> &features,
                       double scale,
                       cv::DrawMatchesFlags flags = cv::DrawMatchesFlags::DEFAULT);

    /**
     * @brief debugImages
     * Save images in current state for debugging.
     * @param source_images
     * @param path
     */
    void debugImages(SourceImages &source_images, path debug_path);

    /**
     * @brief debugMatches
     * Draw feature matches on source images and save the results.
     * @param source_images
     * @param features
     * @param matches
     * @param scale
     * @param conf_threshold
     * @param flags
     */
    void debugMatches(SourceImages &source_images,
                      std::vector<cv::detail::ImageFeatures> &features,
                      std::vector<cv::detail::MatchesInfo> &matches,
                      double scale,
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
     * @param work_scale
     * @return
     */
    std::vector<cv::detail::ImageFeatures> findFeatures(SourceImages &source_images,
                                                        double work_scale);

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
     * Determine compose scale from source image sizes and config.compose_megapix.
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
     * Determine seam scale from source image sizes and config.seam_megapix.
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
     * Determine work scale from image sizes and config.work_megapix.
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
     * @param debug Whether to save undistorted images for debugging.
     * @param debug_path Path to the directory to save debug images.
     */
    void undistortImages(SourceImages &source_images, bool debug,
                         path debug_path = path("debug") / "undistort");

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
