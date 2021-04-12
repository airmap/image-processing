#include "airmap/stitcher_configuration.h"

#include <opencv2/stitching/detail/blenders.hpp>

namespace airmap {
namespace stitcher {

Configuration::Configuration(StitchType stitchType)
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

Configuration::Configuration(
    float blend_strength, int blender_type,
    BundleAdjusterType bundle_adjuster_type, double compose_megapix,
    EstimatorType estimator_type,
    ExposureCompensatorType exposure_compensator_type,
    int exposure_compensation_nr_feeds, int exposure_compensation_nr_filtering,
    int exposure_compensation_block_size,
    FeaturesFinderType features_finder_type,
    FeaturesMatcherType features_matcher_type, int features_maximum,
    float match_conf, double match_conf_thresh, int range_width,
    double seam_megapix, SeamFinderType seam_finder_type,
    float seam_finder_graph_cut_terminal_cost,
    float seam_finder_graph_cut_bad_region_penalty, bool try_cuda,
    WarperType warper_type, bool wave_correct,
    WaveCorrectType wave_correct_type, double work_megapix,
    StitchType stitch_type)
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

} // namespace stitcher
} // namespace airmap
