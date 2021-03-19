#include "airmap/images.h" 

namespace airmap {
namespace stitcher {

SourceImages::SourceImages(const Panorama &panorama,
                           std::shared_ptr<Logger> logger,
                           const int _minimumImageCount)
    : panorama(panorama)
    , images()
    , images_scaled()
    , _logger(logger)
    , minimumImageCount(_minimumImageCount)
{
    resize(static_cast<size_t>(panorama.size()));
    load();
    ensureImageCount();
}

void SourceImages::clear()
{
    gimbal_orientations.clear();

    for (size_t i = 0; i < images.size(); ++i) {
        images[i].release();
        images_scaled[i].release();
    }
    images.clear();
    images_scaled.clear();
}

void SourceImages::ensureImageCount()
{
    if (static_cast<int>(images.size()) < minimumImageCount) {
        std::string message = (boost::format("Need at least %1% images, but only have %2%.") % minimumImageCount % images.size()).str();
        _logger->log(Logger::Severity::error, message.c_str(), "stitcher");
        throw std::invalid_argument(message);
    }
}

void SourceImages::filter(std::vector<int> &keep_indices)
{
    size_t original_count = images.size();
    size_t keep_count = keep_indices.size();
    std::vector<GimbalOrientation> gimbal_orientations_;
    std::vector<cv::Mat> images_;
    std::vector<cv::Mat> images_scaled_;
    gimbal_orientations_.reserve(keep_count);
    images_.reserve(keep_count);
    images_scaled_.reserve(keep_count);

    for (int keep_index : keep_indices) {
        size_t index = static_cast<size_t>(keep_index);
        gimbal_orientations_.push_back(gimbal_orientations[index]);
        images_.push_back(images[index]);
        images_scaled_.push_back(images_scaled[index]);
    }

    gimbal_orientations = gimbal_orientations_;
    images = images_;
    images_scaled = images_scaled_;

    std::stringstream message;
    message << "Discarded " << original_count - keep_count << " images.";
    _logger->log(Logger::Severity::debug, message, "stitcher");

    ensureImageCount();
}

void SourceImages::load()
{
    time_t prevts = 0;
    size_t i = 0;
    for (GeoImage panorama_image : panorama) {
        assert(prevts <= panorama_image.createdTimestampSec);
        prevts = panorama_image.createdTimestampSec;

        std::string path = panorama_image.path;
        cv::Mat image = cv::imread(path);
        if (image.empty()) {
            std::stringstream ss;
            ss << "Can't read image " << path;
            throw std::invalid_argument(ss.str());
        }

        gimbal_orientations[i] = GimbalOrientation(panorama_image.cameraPitchDeg,
            panorama_image.cameraRollDeg, panorama_image.cameraYawDeg);

        images[i] = image;
        images_scaled[i] = image;
        ++i;
    }
}

void SourceImages::reload()
{
    size_t new_size = static_cast<size_t>(panorama.size());
    clear();
    resize(new_size);
    load();
}

void SourceImages::resize(size_t new_size)
{
    gimbal_orientations.resize(new_size);
    images.resize(new_size);
    images_scaled.resize(new_size);
}

void SourceImages::scale(double scale, int interpolation)
{
    for (size_t i = 0; i < images.size(); ++i) {
        cv::resize(images[i], images_scaled[i], cv::Size(), scale, scale, interpolation);
    }
}

void SourceImages::scaleToAvailableMemory(size_t memoryBudgetMB,
                        size_t &maxInputImageSize, size_t &inputSizeMB,
                        double &inputScaled, int interpolation)
{
    size_t totalNoOfInputPixels = 0;
    for (auto &image : images) {
        size_t pixels = image.cols * image.rows;
        inputSizeMB += (image.elemSize() * pixels) / (1024 * 1024);
        totalNoOfInputPixels += pixels;
    }

    double maxInputImageScale = 
        std::min(1.0, (1.0 * images.size() * maxInputImageSize)
            / totalNoOfInputPixels);

    // From this vantage point we consider the stitching algorithm a given. It needs a
    // lot of RAM, or, more practically, to process an input of size X it needs Y
    // megabytes of RAM and there's likely a linear relationship between X and Y,
    // i.e.: Y = inputBudgetMultiplier * X. If Y is greater than
    // parameters.memoryBudgetMB we'll stand no chance of succeeding and so we have no
    // choice, but to scale the input. We've empirically established and will continue
    // to calibrate the value of:
    static constexpr double inputBudgetMultiplier = 5;
    double maxRAMBudgetScale = memoryBudgetMB
                                    / (inputBudgetMultiplier * inputSizeMB);
    inputScaled = std::min(maxRAMBudgetScale, maxInputImageScale);

    if (inputScaled < 1.0) {
        if (inputScaled < 0.2) {
            std::stringstream ss;
            ss << "The RAM budget given ( " << memoryBudgetMB
               << "MB) enforces too much scaling (" << inputScaled << ") of the ("
               << inputSizeMB << " MB of) input, aborting.";
            throw std::invalid_argument(ss.str());
        }

        // Stitching is indeterministic and it may be retried on it - knowing that,
        // nudge the calculated scale by a small, random amount to hopefully push the
        // stitcher from a hypothetical sticky error condition.
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-0.01, 0.01);
        inputScaled += dis(gen);

        // Scale the images.
        scale(inputScaled, interpolation);
        images = images_scaled;

        std::stringstream message;
        message << "Scaled " << inputSizeMB << " MB of input to "
                << inputSizeMB * inputScaled << " MB (by "
                << inputScaled << "), the lesser of: ";
        _logger->log(Logger::Severity::info, message, "stitcher");

        message.str("");
        message << " - " << maxRAMBudgetScale << " to fit the given RAM budget of "
                << memoryBudgetMB << " MB and ";
        _logger->log(Logger::Severity::info, message, "stitcher");

        size_t maxInputImgWidth = std::sqrt(4 * maxInputImageSize / 3);
        size_t maxInputImgHeight = 3 * maxInputImgWidth / 4;
        message.str("");
        message << " - " << maxInputImageScale << " max input image size of "
                << maxInputImgWidth << "x" << maxInputImgHeight;
        _logger->log(Logger::Severity::info, message, "stitcher");
    }
}

} // namespace stitcher
} // namespace airmap
