#include "airmap/opencv/forward.h"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

namespace airmap {
namespace stitcher {
namespace opencv {

int defaultInterpolationFlags() { return cv::INTER_LINEAR_EXACT; }

cv::InputArray noArray()
{
    return cv::noArray();
}

} // namespace opencv
} // namespace stitcher
} // namespace airmap
