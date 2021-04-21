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
 * @brief ThreeSixtyPanoramaOrientationMatchPairsBody
 * A simplified MatchPairsBody that doesn't bother to calculate and
 * store the inverse match and homography.
 * ParallelLoopBody is used to compute a single matching operation (from one
 * image's features to another) and allows for either sequential
 * (e.g. CpuMatcher) or parallel (e.g. GpuMatcher) execution, depending on
 * GPU support.
 */
struct ThreeSixtyPanoramaOrientationMatchPairsBody : ParallelLoopBody {
    ThreeSixtyPanoramaOrientationMatchPairsBody(
        FeaturesMatcher &_matcher, const std::vector<ImageFeatures> &_features,
        std::vector<MatchesInfo> &_pairwise_matches,
        std::vector<std::pair<int, int>> &_near_pairs);

    /**
     * @brief operator()
     * Performs matching for the given range of pair indexes.
     * @param r The range of indexes to perform matches on.
     */
    void operator()(const cv::Range &r) const CV_OVERRIDE;

    FeaturesMatcher &matcher;
    const std::vector<ImageFeatures> &features;
    std::vector<MatchesInfo> &pairwise_matches;
    std::vector<std::pair<int, int>> &near_pairs;
};

/**
 * @brief ThreeSixtyPanoramaOrientationMatcher
 * A custom matcher for determining if the spherical projector/warper
 * will result in an upside down panorama.
 * This is a simplified version of BestOf2NearestMatcher that doesn't
 * bother refining the homography calculation on inliers, since we
 * only calculate the hoomography in order to identify inliers, which
 * are then used to calculate a match confidence.
 */
class ThreeSixtyPanoramaOrientationMatcher : public BestOf2NearestMatcher {
public:
    ThreeSixtyPanoramaOrientationMatcher(bool try_use_gpu = false,
                                         float match_conf = 0.3f,
                                         int num_matches_thresh1 = 6,
                                         int num_matches_thresh2 = 6);

    /**
     * @brief operator()
     * Performs matching for the given image features.  The features
     * vector is expected to contain the features for 3 corresponding
     * sets of images:
     *  - original images
     *  - warped images
     *  - warped images rotated 180 degrees
     * Matching is performed between:
     *  - original images and warped images
     *  - original images and rotated warped images
     * The matches vector is populated with these two consecutive sets
     * of matches.
     * @param features A vector of image features.
     * @param pairwise_matches An empty vector of matches (which will
     * be populated after matching).
     * @param mask Unused and expected to be empty.
     */
    void operator()(const std::vector<ImageFeatures> &features,
                    std::vector<MatchesInfo> &pairwise_matches,
                    const UMat &mask = UMat());

    /**
     * @brief match
     * Performs matching between two sets of features (two images).
     * @param features1 The features for image 1.
     * @param features2 The features for image 2.
     * @param matches_info The resulting matches between the images/features.
     */
    void match(const ImageFeatures &features1, const ImageFeatures &features2,
               MatchesInfo &matches_info) override;
};

} // namespace detail
} // namespace opencv
} // namespace stitcher
} // namespace airmap
