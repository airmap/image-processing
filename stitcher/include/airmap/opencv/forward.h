#pragma once

namespace cv {

class _InputArray;
typedef const _InputArray &InputArray;

class _InputOutputArray;
typedef const _InputOutputArray &InputOutputArray;

class Mat;
class UMat;

template<typename T>
class Point_;
typedef Point_<double> Point2d;

template<typename T>
class Point3_;
typedef Point3_<double> Point3d;

template<typename T>
class Rect_;

typedef Rect_<int> Rect2i;
typedef Rect2i Rect;

namespace detail {

class PairwiseSeamFinder;
class GraphCutSeamFinder;
struct ImageFeatures;
struct MatchesInfo;

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

int defaultInterpolationFlags();

cv::InputArray noArray();

} // namespace opencv
} // namespace stitcher
} // namespace airmap
