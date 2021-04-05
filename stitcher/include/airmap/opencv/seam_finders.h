#pragma once

#include <vector>

#include <opencv2/core/ocl.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/util.hpp>

#include "airmap/monitor/monitor.h"

#include <iostream>

using airmap::stitcher::monitor::Monitor;

namespace airmap {
namespace stitcher {
namespace opencv {
namespace detail {

class MonitoredGraphCutSeamFinder : public cv::detail::GraphCutSeamFinder {
public:
    MonitoredGraphCutSeamFinder(
        Monitor::SharedPtr monitor,
        int cost_type = cv::detail::GraphCutSeamFinder::COST_COLOR_GRAD,
        float terminal_cost = 10000.0f, float bad_region_penalty = 1000.0f);

    ~MonitoredGraphCutSeamFinder();

    void find(const std::vector<cv::UMat> &src,
              const std::vector<cv::Point> &corners,
              std::vector<cv::UMat> &masks) override;

private:
    class Impl; // avoid GCGraph dependency in header
    cv::Ptr<cv::detail::PairwiseSeamFinder> _impl;
};

} // namespace detail
} // namespace opencv
} // namespace stitcher
} // namespace airmap