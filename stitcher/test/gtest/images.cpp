#include "airmap/images.h"
#include "airmap/logging.h"
#include "airmap/panorama.h"
#include "opencv_assert/mat_compare.h"
#include "gtest/gtest.h"

#include <boost/filesystem.hpp>

using airmap::stitcher::GeoImage;
using airmap::stitcher::Panorama;
using airmap::stitcher::SourceImages;
using airmap::logging::Logger;
using airmap::logging::stdoe_logger;
using boost::filesystem::path;

struct Images {
    Images()
    {
        image_directory = path(__FILE__).parent_path() / ".." / "fixtures" / "panorama_aus_1";
        const std::vector<path> image_paths {
            image_directory / "P5050970.JPG",
            image_directory / "P5060971.JPG",
            image_directory / "P5060972.JPG",
            image_directory / "P5070973.JPG",
            image_directory / "P5070974.JPG",
            image_directory / "P5080975.JPG",
            image_directory / "P5080976.JPG",
            image_directory / "P5090977.JPG",
            image_directory / "P5090978.JPG",
            image_directory / "P5100979.JPG",
            image_directory / "P5100980.JPG",
            image_directory / "P5110981.JPG",
            image_directory / "P5110982.JPG",
            image_directory / "P5120983.JPG",
            image_directory / "P5120984.JPG",
            image_directory / "P5130985.JPG",
            image_directory / "P5130986.JPG",
            image_directory / "P5130987.JPG",
            image_directory / "P5140988.JPG",
            image_directory / "P5140989.JPG",
            image_directory / "P5150990.JPG",
            image_directory / "P5150991.JPG",
            image_directory / "P5160992.JPG",
            image_directory / "P5160993.JPG",
            image_directory / "P5160994.JPG"
        };
        for (auto &image_path : image_paths) {
            images.push_back(GeoImage::fromExif(image_path.string()));
        }
    }

    boost::filesystem::path image_directory;
    std::list<boost::filesystem::path> image_paths;
    std::list<GeoImage> images;
};

std::list<GeoImage> input = Images().images;

class SourceImagesTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        Panorama panorama = Panorama(input);
        logger = std::make_shared<stdoe_logger>();
        source_images = std::make_shared<SourceImages>(std::move(panorama), logger);
    }

    std::shared_ptr<airmap::logging::Logger> logger;
    std::shared_ptr<SourceImages> source_images;
};

TEST_F(SourceImagesTest, sourceImagesStruct)
{
    EXPECT_EQ(source_images->gimbal_orientations.size(), 25);
    EXPECT_EQ(source_images->images.size(), 25);
}

TEST_F(SourceImagesTest, sourceImagesClear)
{
    EXPECT_EQ(source_images->gimbal_orientations.size(), 25);
    EXPECT_EQ(source_images->images.size(), 25);

    source_images->clear();

    EXPECT_EQ(source_images->gimbal_orientations.size(), 0);
    EXPECT_EQ(source_images->images.size(), 0);
}

TEST_F(SourceImagesTest, sourceImagesEnsureImageCount)
{
    std::vector<int> keep_indices;
    EXPECT_NO_THROW(source_images->ensureImageCount());

    keep_indices = { 0, 1 };
    source_images->filter(keep_indices);
    EXPECT_NO_THROW(source_images->ensureImageCount());

    keep_indices = { 0 };
    EXPECT_THROW(source_images->filter(keep_indices), std::invalid_argument);
    EXPECT_THROW(source_images->ensureImageCount(), std::invalid_argument);

    source_images->clear();
    EXPECT_THROW(source_images->ensureImageCount(), std::invalid_argument);
}

TEST_F(SourceImagesTest, sourceImagesFilter)
{
    int remove_index = 5;
    cv::Size image_size = source_images->images[remove_index].size();
    cv::Point2i principal_point(image_size.width / 2, image_size.height / 2);
    cv::Vec3b principal_point_values_removed =
        source_images->images[remove_index].at<cv::Vec3b>(
            principal_point.y, principal_point.x);

    cv::Mat expected;
    source_images->images[remove_index].copyTo(expected);

    /**
     * Make sure that no other images match the image to
     * be removed.
     */
    for (size_t i = 0; i < source_images->images.size(); ++i) {
        cv::Mat actual = source_images->images[i];

        if (i == remove_index) {
            EXPECT_PRED_FORMAT2(opencv_assert::CvMatEq, actual, expected);
        } else {
            EXPECT_PRED_FORMAT2(opencv_assert::CvMatNe, actual, expected);
        }
    }

    // Remove image at remove_index.
    std::vector<int> keep_indices;
    for (size_t i = 0; i < source_images->images.size(); ++i) {
        if (i != remove_index) {
            keep_indices.push_back(i);
        }
    }
    source_images->filter(keep_indices);
    EXPECT_EQ(source_images->images.size(), 24);

    /**
     * Make sure that no remaining images match the removed image.
     */
    for (size_t i = 0; i < source_images->images.size(); ++i) {
        cv::Mat actual = source_images->images[i];
        EXPECT_PRED_FORMAT2(opencv_assert::CvMatNe, actual, expected);
    }
}

/**
 * This currently throws an exception during reload.  It only
 * happpens as part of these tests.  valgrind shows invalid reads
 * during std::list<GeoImage> deallocation.
 */
TEST_F(SourceImagesTest, DISABLED_sourceImagesReload)
{
    EXPECT_EQ(source_images->images.size(), 25);

    source_images->clear();
    EXPECT_EQ(source_images->images.size(), 0);

    source_images->reload();
    EXPECT_EQ(source_images->images.size(), 25);
}

TEST_F(SourceImagesTest, sourceImagesResize)
{
    source_images->resize(10);
    EXPECT_EQ(source_images->gimbal_orientations.size(), 10);
    EXPECT_EQ(source_images->images.size(), 10);

    source_images->resize(5);
    EXPECT_EQ(source_images->gimbal_orientations.size(), 5);
    EXPECT_EQ(source_images->images.size(), 5);
}

TEST_F(SourceImagesTest, sourceImagesScale)
{
    cv::Size image_size = source_images->images[0].size();
    for (size_t i = 0; i < source_images->images.size(); ++i) {
        EXPECT_EQ(source_images->images[i].size(), image_size);
    }

    for (size_t i = 0; i < source_images->images_scaled.size(); ++i) {
        EXPECT_EQ(source_images->images_scaled[i].size(), image_size);
    }

    double scale = 0.8;
    source_images->scale(scale);
    for (size_t i = 0; i < source_images->images.size(); ++i) {
        EXPECT_EQ(source_images->images[i].size().width, image_size.width);
        EXPECT_EQ(source_images->images[i].size().height, image_size.height);
    }
    image_size.width = round(image_size.width * scale);
    image_size.height  = round(image_size.height * scale);
    for (size_t i = 0; i < source_images->images_scaled.size(); ++i) {
        EXPECT_EQ(source_images->images_scaled[i].size().width, image_size.width);
        EXPECT_EQ(source_images->images_scaled[i].size().height, image_size.height);
    }

    image_size = source_images->images[0].size();
    scale = 0.30;
    source_images->scale(scale);
    for (size_t i = 0; i < source_images->images.size(); ++i) {
        EXPECT_EQ(source_images->images[i].size().width, image_size.width);
        EXPECT_EQ(source_images->images[i].size().height, image_size.height);
    }
    image_size.width = round(image_size.width * scale);
    image_size.height = round(image_size.height * scale);
    for (size_t i = 0; i < source_images->images_scaled.size(); ++i) {
        EXPECT_EQ(source_images->images_scaled[i].size().width, image_size.width);
        EXPECT_EQ(source_images->images_scaled[i].size().height, image_size.height);
    }

    image_size = source_images->images[0].size();
    scale = 0.31;
    source_images->scale(scale);
    image_size.width = round(image_size.width * scale);
    image_size.height = round(image_size.height * scale);
    for (size_t i = 0; i < source_images->images_scaled.size(); ++i) {
        EXPECT_EQ(source_images->images_scaled[i].size().width, image_size.width);
        EXPECT_EQ(source_images->images_scaled[i].size().height, image_size.height);
    }
}
