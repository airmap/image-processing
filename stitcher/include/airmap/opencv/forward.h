#pragma once

namespace cv {

class Mat;
class UMat;

namespace detail {

class PairwiseSeamFinder;
class GraphCutSeamFinder;
class ImageFeatures;
class MatchesInfo;

} // namespace detail
} // namespace cv

namespace airmap {
namespace stitcher {
namespace opencv {

enum InterpolationFlags {
    INTER_NEAREST = 0,
    INTER_LINEAR = 1,
    INTER_CUBIC = 2,
    INTER_AREA = 3,
    INTER_LANCZOS4 = 4,
    INTER_LINEAR_EXACT = 5,
    INTER_MAX = 7,
};

enum GraphCutSeamFinderCostType { COST_COLOR, COST_COLOR_GRAD };

} // namespace opencv
} // namespace stitcher
} // namespace airmap
