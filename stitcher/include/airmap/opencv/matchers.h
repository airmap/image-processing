#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/stitching/detail/matchers.hpp>

#include <iostream>

using cv::ParallelLoopBody;
using cv::UMat;
using cv::detail::BestOf2NearestMatcher;
using cv::detail::FeaturesMatcher;
using cv::detail::ImageFeatures;
using cv::detail::MatchesInfo;

namespace airmap {
namespace stitcher {
namespace opencv {
namespace detail {

/**
 * @brief ThreeSixtyPanoramaOrientationMatcher
 * A custom matcher for determining if the spherical projector/warper
 * will result in an upside down panorama.
 * TODO(bkd): more details about input assumptions
 */
class ThreeSixtyPanoramaOrientationMatcher : public BestOf2NearestMatcher {
public:
    ThreeSixtyPanoramaOrientationMatcher(bool try_use_gpu = false,
                                         float match_conf = 0.3f,
                                         int num_matches_thresh1 = 6,
                                         int num_matches_thresh2 = 6);

    void operator()(const std::vector<ImageFeatures> &features,
                    std::vector<MatchesInfo> &pairwise_matches,
                    const UMat &mask = UMat());

    void match(const ImageFeatures &features1, const ImageFeatures &features2,
               MatchesInfo &matches_info) override;
};

} // namespace detail
} // namespace opencv
} // namespace stitcher
} // namespace airmap
