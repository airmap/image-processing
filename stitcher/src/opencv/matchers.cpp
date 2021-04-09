#include "airmap/opencv/matchers.h"

namespace airmap {
namespace stitcher {
namespace opencv {
namespace detail {

struct ThreeSixtyPanoramaOrientationMatchPairsBody : ParallelLoopBody {
    ThreeSixtyPanoramaOrientationMatchPairsBody(
        FeaturesMatcher &_matcher, const std::vector<ImageFeatures> &_features,
        std::vector<MatchesInfo> &_pairwise_matches,
        std::vector<std::pair<int, int>> &_near_pairs)
        : matcher(_matcher)
        , features(_features)
        , pairwise_matches(_pairwise_matches)
        , near_pairs(_near_pairs)
    {
    }

    void operator()(const cv::Range &r) const CV_OVERRIDE
    {
        cv::RNG rng = cv::theRNG(); // save entry rng state
        const int num_images = static_cast<int>(features.size() / 3);
        for (int i = r.start; i < r.end; ++i) {
            cv::theRNG() =
                cv::RNG(rng.state +
                        i); // force "stable" RNG seed for each processed pair

            int from = near_pairs[i].first;
            int to = near_pairs[i].second;
            int pair_idx = to >= 2 * num_images ? from + num_images : from;

            matcher(features[from], features[to], pairwise_matches[pair_idx]);
            pairwise_matches[pair_idx].src_img_idx = from;
            pairwise_matches[pair_idx].dst_img_idx = to;
        }
    }

    FeaturesMatcher &matcher;
    const std::vector<ImageFeatures> &features;
    std::vector<MatchesInfo> &pairwise_matches;
    std::vector<std::pair<int, int>> &near_pairs;
};

ThreeSixtyPanoramaOrientationMatcher::ThreeSixtyPanoramaOrientationMatcher(
    bool try_use_gpu, float match_conf, int num_matches_thresh1,
    int num_matches_thresh2)
    : BestOf2NearestMatcher(try_use_gpu, match_conf, num_matches_thresh1,
                            num_matches_thresh2)
{
}

void ThreeSixtyPanoramaOrientationMatcher::operator()(
    const std::vector<ImageFeatures> &features,
    std::vector<MatchesInfo> &pairwise_matches, const UMat &mask)
{
    assert(mask.empty());

    const int num_images = static_cast<int>(features.size() / 3);
    std::vector<std::pair<int, int>> near_pairs;
    for (int i = 0; i < num_images; i++) {
        if (features[i].keypoints.size() > 0) {
            // Match original images with warped images.
            if (features[num_images + i].keypoints.size() > 0) {
                near_pairs.push_back(std::make_pair(i, num_images + i));
            }
            // Match original images with warped and rotated images.
            if (features[2 * num_images + i].keypoints.size() > 0) {
                near_pairs.push_back(std::make_pair(i, 2 * num_images + i));
            }
        }
    }

    pairwise_matches.clear();
    pairwise_matches.resize(near_pairs.size());
    ThreeSixtyPanoramaOrientationMatchPairsBody body(
        *this, features, pairwise_matches, near_pairs);
    if (is_thread_safe_) {
        parallel_for_(cv::Range(0, static_cast<int>(near_pairs.size())), body);
    } else {
        body(cv::Range(0, static_cast<int>(near_pairs.size())));
    }
}

void ThreeSixtyPanoramaOrientationMatcher::match(
    const cv::detail::ImageFeatures &features1,
    const cv::detail::ImageFeatures &features2,
    cv::detail::MatchesInfo &matches_info)
{
    (*impl_)(features1, features2, matches_info);

    // Check if it makes sense to find homography
    if (matches_info.matches.size() < static_cast<size_t>(num_matches_thresh1_))
        return;

    // Construct point-point correspondences for homography estimation
    cv::Mat src_points(1, static_cast<int>(matches_info.matches.size()),
                       CV_32FC2);
    cv::Mat dst_points(1, static_cast<int>(matches_info.matches.size()),
                       CV_32FC2);
    for (size_t i = 0; i < matches_info.matches.size(); ++i) {
        const cv::DMatch &m = matches_info.matches[i];

        cv::Point2f p = features1.keypoints[m.queryIdx].pt;
        p.x -= features1.img_size.width * 0.5f;
        p.y -= features1.img_size.height * 0.5f;
        src_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;

        p = features2.keypoints[m.trainIdx].pt;
        p.x -= features2.img_size.width * 0.5f;
        p.y -= features2.img_size.height * 0.5f;
        dst_points.at<cv::Point2f>(0, static_cast<int>(i)) = p;
    }

    // Find pair-wise motion
    matches_info.H = cv::findHomography(src_points, dst_points,
                                        matches_info.inliers_mask, cv::RANSAC);
    if (matches_info.H.empty() || std::abs(cv::determinant(matches_info.H)) <
                                      std::numeric_limits<double>::epsilon())
        return;

    // Find number of inliers
    matches_info.num_inliers = 0;
    for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
        if (matches_info.inliers_mask[i])
            matches_info.num_inliers++;

    // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic
    // Image Stitching using Invariant Features"
    matches_info.confidence =
        matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());
}

} // namespace detail
} // namespace opencv
} // namespace stitcher
} // namespace airmap
