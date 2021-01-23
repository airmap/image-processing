#include "airmap/images.h" 

namespace airmap {
namespace stitcher {

void SourceImages::clear()
{
    gimbal_orientations.clear();
    sizes.clear();

    for (size_t i = 0; i < images.size(); ++i) {
        images[i].release();
    }
    images.clear();
}

void SourceImages::ensureImageCount()
{
    if (images.size() < minimumImageCount) {
        std::string message = (boost::format("Need at least %1% images, but only have %2%.") % minimumImageCount % images.size()).str();
        LOG(error) << message.c_str();
        throw std::invalid_argument(message);
    }
}

void SourceImages::filter(std::vector<int> &keep_indices)
{
    size_t original_count = images.size();
    size_t keep_count = keep_indices.size();
    std::vector<GimbalOrientation> gimbal_orientations_;
    std::vector<cv::Mat> images_;
    std::vector<cv::Size> sizes_;
    gimbal_orientations_.reserve(keep_count);
    images_.reserve(keep_count);
    sizes_.reserve(keep_count);

    for (int keep_index : keep_indices) {
        size_t index = static_cast<size_t>(keep_index);
        gimbal_orientations_.push_back(gimbal_orientations[index]);
        images_.push_back(images[index]);
        sizes_.push_back(sizes[index]);
    }

    gimbal_orientations = gimbal_orientations_;
    images = images_;
    sizes = sizes_;

    LOG(debug) << "Discarded" << original_count - keep_count << "images.";

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
        sizes[i] = image.size();
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
    sizes.resize(new_size);
}

void SourceImages::scale(double scale, int interpolation)
{
    for (size_t i = 0; i < images.size(); ++i) {
        cv::resize(images[i], images[i], cv::Size(), scale, scale, interpolation);
    }
}

} // namespace stitcher
} // namespace airmap
