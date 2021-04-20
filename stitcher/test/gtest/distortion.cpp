#include "gtest/gtest.h"

#include "airmap/camera.h"
#include "airmap/camera_models.h"
#include "airmap/distortion.h"
#include "util/mat_compare.h"

#include "boost/filesystem.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using util::opencv_assert::CvMatEq;
using util::opencv_assert::CvMatNe;

namespace airmap {
namespace stitcher {

using boost::filesystem::path;

//
// 
// PinholeDistortionModel Tests
// 
//
class PinholeDistortionModelTest : public ::testing::Test
{
protected:
    Camera createCamera(bool distortion_model_enabled = false)
    {
        return CameraModels::ParrotAnafiThermal(distortion_model_enabled);
    }

    cv::Mat createDistortionVector()
    {
        double k1 = 2.8391010208309218e-02;
        double k2 = -2.7239202041003809e-02;
        double k3 = 0.0;
        double p1 = -2.4700935014356916e-03;
        double p2 = 6.1345950301455029e-03;

        return (cv::Mat_<double>(5, 1) <<
                k1, k2, p1, p2, k3);
    }

    PinholeDistortionModel::Parameters createParameters()
    {
        cv::Mat distortion_vector = createDistortionVector();
        double k1 = distortion_vector.at<double>(0, 0);
        double k2 = distortion_vector.at<double>(1, 0);
        double p1 = distortion_vector.at<double>(2, 0);
        double p2 = distortion_vector.at<double>(3, 0);
        double k3 = distortion_vector.at<double>(4, 0);
        return PinholeDistortionModel::Parameters(k1, k2, p1, p2, k3);
    }

    cv::Mat createSourceImage()
    {
        path image_directory = path(__FILE__).parent_path() / ".." / "fixtures"
                                                            / "panorama_aus_1";
        path image_path = image_directory / "P5050970.JPG";
        return cv::imread(image_path.string());
    }

    cv::Mat createUndistortedImage()
    {
        path image_directory = path(__FILE__).parent_path() / ".." / "fixtures"
                                                            / "distortion"
                                                            / "anafi_thermal"
                                                            / "undistorted";
        path image_path = image_directory / "P5050970.png";
        return cv::imread(image_path.string());
    }
};

TEST_F(PinholeDistortionModelTest, parametersStruct)
{
    cv::Mat distortion_vector = createDistortionVector();
    double k1 = distortion_vector.at<double>(0, 0);
    double k2 = distortion_vector.at<double>(1, 0);
    double p1 = distortion_vector.at<double>(2, 0);
    double p2 = distortion_vector.at<double>(3, 0);
    double k3 = distortion_vector.at<double>(4, 0);

    auto distortion = createParameters();
    EXPECT_DOUBLE_EQ(distortion.k1(), k1);
    EXPECT_DOUBLE_EQ(distortion.k2(), k2);
    EXPECT_DOUBLE_EQ(distortion.k3(), k3);
    EXPECT_DOUBLE_EQ(distortion.p1(), p1);
    EXPECT_DOUBLE_EQ(distortion.p2(), p2);
}

TEST_F(PinholeDistortionModelTest, pinholeDistortionVector)
{
    auto expected_parameters = createParameters();
    cv::Mat actual_vector = createDistortionVector();
    PinholeDistortionModel::Parameters actual_parameters(actual_vector);
    EXPECT_EQ(expected_parameters, actual_parameters);
}

TEST_F(PinholeDistortionModelTest, pinholeUndistortDisabled)
{
    auto expected_parameters = createParameters();
    Camera camera = createCamera();
    EXPECT_TRUE(camera.distortion_model);
    EXPECT_FALSE(camera.distortion_model->enabled());
}

TEST_F(PinholeDistortionModelTest, pinholeUndistortImages)
{
    auto expected_parameters = createParameters();
    Camera camera = createCamera(true);
    EXPECT_TRUE(camera.distortion_model);

    cv::Mat expected_image = createSourceImage();
    std::vector<cv::Mat> images = { expected_image };
    camera.distortion_model->undistort(images, camera.K());
    cv::Mat actual_image = images[0];
    EXPECT_PRED_FORMAT2(CvMatNe, actual_image, expected_image);
}

TEST_F(PinholeDistortionModelTest, pinholeUndistortImage)
{
    auto expected_parameters = createParameters();
    Camera camera = createCamera(true);
    EXPECT_TRUE(camera.distortion_model);

    cv::Mat actual_image = createSourceImage();
    camera.distortion_model->undistort(actual_image, camera.K());
    cv::Mat expected_image = createSourceImage();
    EXPECT_PRED_FORMAT2(CvMatNe, actual_image, expected_image);

    expected_image = createUndistortedImage();
    EXPECT_PRED_FORMAT2(CvMatEq, actual_image, expected_image);
}

//
// 
// ScaramuzzaDistortionModel Tests
// 
//
class ScaramuzzaDistortionModelTest : public ::testing::Test
{
protected:
    Camera createCamera()
    {
        return CameraModels::VantageVesperEONavigation();
    }

    ScaramuzzaDistortionModel::Parameters createParameters()
    {
        std::vector<double> pol = { -1.304378e+03, 0.000000e+00, 5.113289e-04,
                                    -3.677822e-07, 2.496957e-10 };
        std::vector<double>inv_pol = { 1199.260777, -2648.472276, -12773.986580,
                                       -27539.670273, -36582.498387,
                                       -29546.356228, -14151.354913,
                                       -3691.813594, -403.401517 };
        double xc = 959.042242;
        double yc = 539.041192;
        double c = 0.999434;
        double d = -0.000385;
        double e = -0.000025;
        double width = 1920;
        double height = 1080;
        double scale_factor = 2;
        double resolution_scale = 0.5; // test images are 1/4 scale
        return ScaramuzzaDistortionModel::Parameters(pol, inv_pol, xc, yc, c, d,
                                                     e, width, height,
                                                     scale_factor,
                                                     resolution_scale);
    }

    cv::Mat createSourceImage()
    {
        path image_directory = path(__FILE__).parent_path() / ".." / "fixtures"
                                                            / "distortion" / "vesper"
                                                            / "original";
        path image_path = image_directory / "0.jpg";
        return cv::imread(image_path.string());
    }

    cv::Mat createUndistortedImage()
    {
        path image_directory = path(__FILE__).parent_path() / ".." / "fixtures"
                                                            / "distortion" / "vesper"
                                                            / "undistorted";
        path image_path = image_directory / "0.png";
        return cv::imread(image_path.string());
    }
};

TEST_F(ScaramuzzaDistortionModelTest, scaramuzzaUndistortImage)
{
    ScaramuzzaDistortionModel::Parameters parameters = createParameters();
    ScaramuzzaDistortionModel distortion_model(parameters);

    Camera camera = CameraModels::VantageVesperEONavigation();
    cv::Mat actual_image = createSourceImage();
    distortion_model.undistort(actual_image, camera.K());
    cv::Mat expected_image = createUndistortedImage();
    EXPECT_PRED_FORMAT2(CvMatEq, actual_image, expected_image);
}

} // namespace stitcher
} // namespace airmap
