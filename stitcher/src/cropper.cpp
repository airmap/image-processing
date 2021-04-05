#include "cropper.h"

#include <algorithm>
#include <iterator>

#include <opencv2/imgproc.hpp>

cv::Mat Cropper::cropNullEdges(const cv::Mat &original, double maximumRatio)
{
    // To crop null edges, we will:
    //
    // 1) Validate that the image meets a minimum requirements to avoid doing work that
    // will fail.
    //
    // 2) Convert the image to a mask -- black for null regions and white for image data.
    // The mask will be used to facilitate contour detection along the image/null
    // boundaries.
    //
    // 3) Find the contour along the mask. Black-and-white is important here.
    //
    // 4) Build a minimum bounding rectangle along the contour and repeatedly erode the
    // contour along the bounding rectangle and updating the bounding rectangle. Erosion
    // helps to minimize noise from non-linear null regions intersecting the boundary.
    //
    // 5) Crop to the finalized bounding rectangle.
    if (1.0 <= maximumRatio) {
        return original;
    }

    cv::Mat panorama_mask;
    cv::Rect bounds;
    cv::Mat crop_mask;
    cv::Mat cropped;

    // Step name and its functor
    using step_type = std::pair<const char *, std::function<void(void)>>;
    std::vector<step_type> steps {
        { "Validating image minimums", [&]() { validateMinimumSize(original); } },
        { "Building crop mask", [&]() { panorama_mask = buildPanoramaMask(original); } },
        { "Finding contours along the crop mask",
          [&]() {
              std::tie(crop_mask, bounds) = maskContour(panorama_mask);
              panorama_mask.release();
          } },
        { "Eroding down to find boundary",
          [&]() {
              bounds = findErosionBoundary(crop_mask, original);
              crop_mask.release();
          } },
        { "Cropping",
          [&]() {
              double ratio = (double(bounds.area()) / original.size().area());
              if (maximumRatio <= ratio) {
                  cropped = original(bounds);
              } else {
                  cropped = original;
              }
          } }
    };

    // Run the steps, logging each.
    for (auto step : steps) {
        try {
            step.second();
        } catch (const cv::Exception &e) {
            throw;
        } catch (const std::exception &e) {
            throw;
        }
    }
    return cropped;
}

void Cropper::validateMinimumSize(const cv::Mat &img)
{
    // I just want to prevent negatives or division by zero after mathing.
    // Originally I had thought that copyMakeBorder would create a border by overwriting
    // existing pixel data instead of expanding the image to add more pixel area. I had
    // then understood that, at the end of this function, removing the border would
    // involve subtracting the border area. I didn't want to enforce any real minimum size
    // of the image beyond just making sure that pixel index wouldn't go negative if I'd
    // tried something like (image.size().width - borderPixels) / 2. Both concerns were
    // mistaken but I left the guard rails in place.
    if ((img.size().width < minwidth_) || (img.size().height < minheight_)) {
        std::string errmsg { "Resulting panorama size is too small for cropping: " };
        errmsg += std::to_string(img.size().width) + " x "
                + std::to_string(img.size().height);
        throw std::invalid_argument { errmsg };
    }
}

cv::Mat Cropper::buildPanoramaMask(const cv::Mat &original)
{
    // Null regions in the panorama are black pixels. We will use cv::threshold
    // to convert non-black pixels to white pixels. That will give us a black-and-white
    // image which is easy to detect contours. Using cv::threshold requires the image to
    // be in grayscale.
    //
    // The OpenCV tutorial on thresholding has good visualization about how to build a
    // mask: https://docs.opencv.org/4.2.0/db/d8e/tutorial_threshold.html
    cv::Mat bordered;
    cv::copyMakeBorder(original, bordered, borderPixels_, borderPixels_, borderPixels_,
                       borderPixels_, cv::BORDER_CONSTANT, { 0.f });

    // Grayscale requires 8-bit data. Downscale to eight-bit data.
    cv::Mat eightbit;
    bordered.convertTo(eightbit, CV_8U);
    bordered.release();

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(eightbit, gray, cv::COLOR_RGB2GRAY);
    eightbit.release();

    cv::Mat graymask;
    cv::threshold(gray, graymask, 0, 255, cv::THRESH_BINARY);
    return graymask;
}

cv::Rect Cropper::bestContourRect(const cv::Mat &img)
{
    // The Python tutorial has good imagery to learn how contouring works:
    // https://docs.opencv.org/4.2.0/dd/d49/tutorial_py_contour_features.html

    contours_.clear();
    cv::findContours(img, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours_.empty()) {
        std::string errmsg { "No contours found!" };
        throw std::logic_error { errmsg };
    }
    // In case multiple contours are found, use the one with the maximum inner area.
    // TODO: this assumes that it's general noise (eg, non/overlapping camera FoVs,
    // different origins, etc) which cause the separate contours.
    // TODO: If somehow non-overlapping fields of views were stitched then this is
    // FUBAR and will cut all but the largest.
    areas_.clear();
    areas_.reserve(contours_.size());
    std::transform(contours_.begin(), contours_.end(), std::back_inserter(areas_),
                   [](const Contour &cc) { return contourArea(cc); });
    auto max_contour = contours_.begin()
            + std::distance(areas_.begin(),
                            std::max_element(areas_.begin(), areas_.end()));
    cv::Rect bounds;
    bounds = boundingRect(*max_contour);
    return bounds;
}

Cropper::CropMask Cropper::maskContour(const cv::Mat &img)
{
    auto bounds = bestContourRect(img);
    cv::Mat crop_mask { img.size(), CV_8U };
    cv::rectangle(crop_mask, bounds.tl(), bounds.br(), cv::Scalar { 255.f }, cv::FILLED);
    return std::make_pair(crop_mask, bounds);
}

cv::Mat Cropper::erodeMask(const cv::Mat &input_black_and_white_mask)
{
    cv::Mat eroded_mask { input_black_and_white_mask.clone() };
    cv::Mat sub { input_black_and_white_mask.clone() };
    while (auto nz = countNonZero(sub)) {
        cv::Mat eroded { eroded_mask.clone() };
        cv::erode(eroded_mask, eroded,
                  cv::getStructuringElement(cv::MORPH_RECT, cv::Size { 3, 3 }));
        eroded_mask = eroded;
        subtract(eroded_mask, input_black_and_white_mask, sub);
        if (countNonZero(sub) == nz) {
            throw std::logic_error { "infinite erosion" };
        }
    }
    return eroded_mask;
}

cv::Rect Cropper::findErosionBoundary(const cv::Mat &uneroded_mask,
                                      const cv::Mat &original)
{

    // erodeMask will return a tighter mask after erosion.
    cv::Mat eroded_mask = erodeMask(uneroded_mask);
    return bestContourRect(
            { eroded_mask,
              // Remember that our border at the beginning increased our
              // image area. So, we provide a Region of Interest to clip
              // our contour area.
              cv::Rect { cv::Point { borderPixels_, borderPixels_ },
                         cv::Point { original.size().width, original.size().height } } });
}
