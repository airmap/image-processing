#pragma once

#include "airmap/panorama.h"
#include "gtest/gtest.h"

#include <vector>

#include <boost/filesystem.hpp>
#include <opencv2/imgcodecs.hpp>

using airmap::stitcher::GeoImage;
using boost::filesystem::path;

namespace util {
namespace images {

struct Images {
    static std::list<GeoImage> original()
    {
        path image_directory;
        std::vector<path> image_paths;
        std::list<GeoImage> images;

        image_directory = path(__FILE__).parent_path() / ".." / ".." /
                          "fixtures" / "panorama_aus_1";
        image_paths = {
            image_directory / "P5050970.JPG", image_directory / "P5060971.JPG",
            image_directory / "P5060972.JPG", image_directory / "P5070973.JPG",
            image_directory / "P5070974.JPG", image_directory / "P5080975.JPG",
            image_directory / "P5080976.JPG", image_directory / "P5090977.JPG",
            image_directory / "P5090978.JPG", image_directory / "P5100979.JPG",
            image_directory / "P5100980.JPG", image_directory / "P5110981.JPG",
            image_directory / "P5110982.JPG", image_directory / "P5120983.JPG",
            image_directory / "P5120984.JPG", image_directory / "P5130985.JPG",
            image_directory / "P5130986.JPG", image_directory / "P5130987.JPG",
            image_directory / "P5140988.JPG", image_directory / "P5140989.JPG",
            image_directory / "P5150990.JPG", image_directory / "P5150991.JPG",
            image_directory / "P5160992.JPG", image_directory / "P5160993.JPG",
            image_directory / "P5160994.JPG"};
        for (auto &image_path : image_paths) {
            images.push_back(GeoImage::fromExif(image_path.string()));
        }

        return images;
    }

    static std::vector<cv::Mat> warped()
    {
        path image_directory;
        std::vector<path> image_paths;
        std::vector<cv::Mat> images;

        image_directory = path(__FILE__).parent_path() / ".." / ".." /
                          "fixtures" / "panorama_aus_1" / "warped";
        image_paths = {image_directory / "0.jpg",  image_directory / "1.jpg",
                       image_directory / "2.jpg",  image_directory / "3.jpg",
                       image_directory / "4.jpg",  image_directory / "5.jpg",
                       image_directory / "6.jpg",  image_directory / "7.jpg",
                       image_directory / "8.jpg",  image_directory / "9.jpg",
                       image_directory / "10.jpg", image_directory / "11.jpg",
                       image_directory / "12.jpg", image_directory / "13.jpg",
                       image_directory / "14.jpg", image_directory / "15.jpg",
                       image_directory / "16.jpg", image_directory / "17.jpg",
                       image_directory / "18.jpg", image_directory / "19.jpg",
                       image_directory / "20.jpg", image_directory / "21.jpg",
                       image_directory / "22.jpg", image_directory / "23.jpg",
                       image_directory / "24.jpg"};
        for (auto &image_path : image_paths) {
            images.push_back(cv::imread(image_path.string()));
        }

        return images;
    }
};

} // namespace images
} // namespace util
