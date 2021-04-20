#include "mat_compare.h"

#include "opencv2/core.hpp"

using util::opencv_assert::CvMatCompare;
using util::opencv_assert::CvMatEq;
using util::opencv_assert::CvMatNe;

TEST(matCompare, dimensions)
{
    cv::Mat a, b;

    a = (cv::Mat_<double>(5, 1) << 1, 2, 3, 4, 5);
    b = (cv::Mat_<double>(5, 1) << 1, 2, 3, 4, 5);
    EXPECT_PRED_FORMAT2(CvMatEq, a, b);

    a = (cv::Mat_<double>(5, 1) << 1, 2, 3, 4, 5);
    b = (cv::Mat_<double>(1, 5) << 1, 2, 3, 4, 5);
    EXPECT_PRED_FORMAT2(CvMatNe, a, b);
}

TEST(matCompare, size)
{
    cv::Mat a, b;

    a = (cv::Mat_<double>(2, 2) << 1, 2, 3, 4);
    b = (cv::Mat_<double>(2, 2) << 1, 2, 3, 4);
    EXPECT_PRED_FORMAT2(CvMatEq, a, b);

    a = (cv::Mat_<double>(2, 2) << 1, 2, 3, 4);
    b = (cv::Mat_<double>(1, 1) << 1, 2);
    EXPECT_PRED_FORMAT2(CvMatNe, a, b);
}

TEST(matCompare, singleChannel)
{
    cv::Mat a = (cv::Mat_<double>(2, 2) << 1, 3, 2, 4);
    cv::Mat b = (cv::Mat_<double>(2, 2) << 1, 2, 3, 4);
    EXPECT_PRED_FORMAT2(CvMatNe, a, b);
}

TEST(matCompare, multiChannel)
{
    cv::Mat channel_b(3, 1, CV_8UC1, cv::Scalar(1, 1, 1));
    cv::Mat channel_g(3, 1, CV_8UC1, cv::Scalar(2, 2, 2));
    cv::Mat channel_r(3, 1, CV_8UC1, cv::Scalar(3, 3, 3));

    cv::Mat a(3, 1, CV_8UC3);
    cv::Mat b(3, 1, CV_8UC3);

    std::vector<cv::Mat> channels_a, channels_b;

    /**
     * Two 3-channel matrices with the same values.
     */
    channels_a = {channel_b, channel_g, channel_r};
    channels_b = {channel_b, channel_g, channel_r};
    merge(channels_a, a);
    merge(channels_b, b);
    EXPECT_PRED_FORMAT2(CvMatEq, a, b);

    /**
     * Two 3-channel matrices with inverted channels.
     */
    channels_a = {channel_b, channel_g, channel_r};
    channels_b = {channel_r, channel_g, channel_b};
    merge(channels_a, a);
    merge(channels_b, b);
    EXPECT_PRED_FORMAT2(CvMatNe, a, b);
}
