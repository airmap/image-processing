#include "airmap/opencv/forward.h"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>

namespace airmap {
namespace stitcher {
namespace opencv {

cv::InputArray noArray()
{
    return cv::noArray();
}

} // namespace opencv
} // namespace stitcher
} // namespace airmap
