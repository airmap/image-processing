#pragma once

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

enum class BundleAdjusterType{
    Ray,
    Reproj,
    AffinePartial,
    No
};

enum class EstimatorType {
    Affine,
    Homography
};

enum class ExposureCompensatorType {
    Channels,
    ChannelsBlocks,
    Gain,
    GainBlocks,
    No
};

enum class FeaturesFinderType {
    Akaze,
    Orb,
    Sift,
    Surf
};

enum class FeaturesMatcherType {
    Affine,
    Homography
};

enum class SeamFinderType {
    DpColor,
    DpColorGrad,
    GraphCutColor,
    GraphCutColorGrad,
    No,
    Voronoi
};

enum class StitchType {
    ThreeSixty,
    No
};

enum class WaveCorrectType {
    Horizontal,
    Vertical
};

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

    /**
     * @brief seam_finder_graph_cut_terminal_cost
     * The cost/weight of a terminal node (pixel in a source or sink).
     * 
     * More info: https://www.cc.gatech.edu/~turk/my_papers/graph_cuts.pdf
     */
    float seam_finder_graph_cut_terminal_cost;

    /**
     * @brief seam_finder_graph_cut_bad_region_penalty
     * An additional cost/weight added for a pixel that is in neither the
     * source nor the sink.
     * 
     * My (basic) understanding (which matches emperical results) is that
     * a higher penalty makes it less likely for a seam to go through a
     * region where there is no overlap between two images.  In practice,
     * this seems effective in reducing gaps in the stitch.
     * 
     * More info: https://www.cc.gatech.edu/~turk/my_papers/graph_cuts.pdf
     */
    float seam_finder_graph_cut_bad_region_penalty;

    /**
     * @brief stitch_type
     * The type of stitch (e.g. ThreeSixty).
     */
    StitchType stitch_type;

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
    Configuration(StitchType stitchType)
    {
        switch (stitchType) {
        case StitchType::ThreeSixty:
            blend_strength = 5;
            blender_type = cv::detail::Blender::MULTI_BAND;
            bundle_adjuster_type = BundleAdjusterType::Ray;
            compose_megapix = -1;
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
            seam_finder_graph_cut_terminal_cost = 10000.f;
            seam_finder_graph_cut_bad_region_penalty = 10000000.f;
            try_cuda = false;
            warper_type = WarperType::Spherical;
            wave_correct = true;
            wave_correct_type = WaveCorrectType::Horizontal;
            work_megapix = 0.6;
            break;
        case StitchType::No:
            break;
        }
        stitch_type = stitchType;
    }

    /**
     * @brief Configuration
     * Create a custom stitching configuration.
     * @param blend_strength
     * @param blender_type
     * @param bundle_adjuster_type
     * @param compose_megapix
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
     * @param seam_finder_graph_cut_terminal_cost
     * @param seam_finder_graph_cut_bad_region_penalty
     * @param stitch_type
     * @param try_cuda
     * @param warper_type
     * @param wave_correct
     * @param wave_correct_type
     * @param work_megapix
     */
    Configuration(float blend_strength, int blender_type,
                    BundleAdjusterType bundle_adjuster_type, double compose_megapix,
                    EstimatorType estimator_type,
                    ExposureCompensatorType exposure_compensator_type,
                    int exposure_compensation_nr_feeds,
                    int exposure_compensation_nr_filtering,
                    int exposure_compensation_block_size,
                    FeaturesFinderType features_finder_type,
                    FeaturesMatcherType features_matcher_type,
                    int features_maximum, float match_conf,
                    double match_conf_thresh, int range_width, 
                    double seam_megapix, SeamFinderType seam_finder_type,
                    float seam_finder_graph_cut_terminal_cost,
                    float seam_finder_graph_cut_bad_region_penalty,
                    bool try_cuda, WarperType warper_type, bool wave_correct,
                    WaveCorrectType wave_correct_type, double work_megapix,
                    StitchType stitch_type = StitchType::No)
        : blend_strength(blend_strength)
        , blender_type(blender_type)
        , bundle_adjuster_type(bundle_adjuster_type)
        , compose_megapix(compose_megapix)
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
        , seam_finder_graph_cut_terminal_cost(seam_finder_graph_cut_terminal_cost)
        , seam_finder_graph_cut_bad_region_penalty(
            seam_finder_graph_cut_bad_region_penalty)
        , stitch_type(stitch_type)
        , try_cuda(try_cuda)
        , warper_type(warper_type)
        , wave_correct(wave_correct)
        , wave_correct_type(wave_correct_type)
        , work_megapix(work_megapix)
    {
    }
};

} // namespace stitcher
} // namespace airmap
