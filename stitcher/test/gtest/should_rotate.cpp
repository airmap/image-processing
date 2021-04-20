#include "gtest/gtest.h"

#include "airmap/logging.h"
#include "airmap/opencv_stitcher.h"
#include "airmap/panorama.h"
#include "airmap/stitcher_configuration.h"
#include "util/images.h"

using airmap::logging::stdoe_logger;
using util::images::Images;

namespace airmap {
namespace stitcher {

std::list<GeoImage> input = Images::original();
std::vector<cv::Mat> warped_images = Images::warped();

class TestLowLevelOpenCVStitcher : public LowLevelOpenCVStitcher {
public:
    TestLowLevelOpenCVStitcher()
        : LowLevelOpenCVStitcher(
              Configuration(StitchType::ThreeSixty), Panorama{input},
              Panorama::Parameters{
                  Panorama::Parameters::defaultMemoryBudgetMB()},
              "", std::make_shared<stdoe_logger>())
    {
    }

    bool shouldRotateThreeSixty(std::vector<cv::Mat> &warped_images)
    {
        Stitcher::Report report;
        SourceImages source_images(_panorama, _logger);
        source_images.scaleToAvailableMemory(
            _parameters.memoryBudgetMB, _parameters.maxInputImageSize,
            report.inputSizeMB, report.inputScaled);
        source_images.scale(getSeamScale(source_images));
        return LowLevelOpenCVStitcher::shouldRotateThreeSixty(
            source_images.images_scaled, warped_images);
    }
};

TEST(shouldRotate, shouldRotate)
{
    TestLowLevelOpenCVStitcher stitcher;
    EXPECT_EQ(input.size(), warped_images.size());
    EXPECT_EQ(stitcher.shouldRotateThreeSixty(warped_images), true);
}

TEST(shouldRotate, shouldNotRotate)
{
    TestLowLevelOpenCVStitcher stitcher;
    EXPECT_EQ(input.size(), warped_images.size());

    std::vector<cv::Mat> warped_rotated_images;
    cv::Mat warped_rotated_image;
    for (size_t i = 0; i < warped_images.size(); i++) {
        float x = (warped_images[i].size().width - 1) / 2.f;
        float y = (warped_images[i].size().height - 1) / 2.f;
        cv::Point2f center(x, y);
        cv::Mat rotation = cv::getRotationMatrix2D(center, 180., 1.);
        cv::warpAffine(warped_images[i], warped_rotated_image, rotation,
                       warped_images[i].size());
        warped_rotated_images.push_back(warped_rotated_image);
    }

    EXPECT_EQ(stitcher.shouldRotateThreeSixty(warped_rotated_images), false);
}

} // namespace stitcher
} // namespace airmap
