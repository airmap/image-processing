#pragma once

#include <cassert>
#include <cmath>
#include <limits>
#include <list>
#include <memory>
#include <utility>
#include <string>

namespace airmap {
namespace filesystem {

/**
 * @brief The path class is a basic ersatz for a std::filesystem (C++17) or boost::filesystem
 * that doesn't require linking anything
 * 
 * boost::filesystem::path is used in other parts of the project, but keeping it
 * it out of the headers so that consuming projects don't have to link to boost
 * if only using headers.
 */
class path {
    public:
        path();
        path(const std::string& p);

        inline std::string string() const {
            return _path;
        }

        path parent_path() const;
        std::string stem() const;

        inline bool operator==(const path& other) const {
            return _path == other._path;
        }

        inline bool operator!=(const path& other) const {
            return _path != other._path;
        }

        inline static char separator() {
#ifdef _WIN32
            return '\\';
#else
            return '/';
#endif
        }

        inline path operator/(const path& other) const {
            return _path + separator() + other._path;
        }

    private:
        std::string _path;
};

} // namespace filesystem

namespace stitcher {

class geocoordinate_t : public std::pair<double, double>
{
public:
    static constexpr double pi = M_PI;
    static constexpr double deg2Rad = pi / 180.0;
    static constexpr double rad2Deg = 180.0 / pi;

    geocoordinate_t(double lng, double lat)
        : std::pair<double, double>(lng, lat)
    {
    }

    inline double lng() const { return std::get<0>(*this); }
    inline double lat() const { return std::get<1>(*this); }

    float distance_metres(const geocoordinate_t &to) const;
};

class geo_bounding_box_t : protected std::pair<geocoordinate_t, geocoordinate_t>
{
public:
    geo_bounding_box_t(const geocoordinate_t &centre)
        : std::pair<geocoordinate_t, geocoordinate_t>(centre, centre)
    {
    }

    geo_bounding_box_t()
        : std::pair<geocoordinate_t, geocoordinate_t>({ 1, 1 }, { -1, -1 })
    {
    }

    geo_bounding_box_t &add(const geocoordinate_t &coord)
    {
        if (isValid()) {
            std::get<0>(*this) = geocoordinate_t { std::min(min().lng(), coord.lng()),
                                                   std::min(min().lat(), coord.lat()) };
            std::get<1>(*this) = geocoordinate_t { std::max(max().lng(), coord.lng()),
                                                   std::max(max().lat(), coord.lat()) };
        } else {
            std::get<0>(*this) = coord;
            std::get<1>(*this) = coord;
        }
        return *this;
    }

    inline geocoordinate_t min() const { return std::get<0>(*this); }
    inline geocoordinate_t max() const { return std::get<1>(*this); }
    inline bool isValid() const { return min().lat() <= max().lat(); }

    geocoordinate_t centre() const
    {
        return geocoordinate_t {
            std::get<0>(*this).lng()
                    + (std::get<1>(*this).lng() - std::get<0>(*this).lng()) / 2,
            std::get<0>(*this).lat()
                    + (std::get<1>(*this).lat() - std::get<0>(*this).lat()) / 2
        };
    }
};


/**
 * @brief The GeoImage struct holds metadata on a geotagged image
 */
struct GeoImage
{
    using shared_ptr = std::shared_ptr<GeoImage>;

    std::string path;
    geocoordinate_t geoCoordinate;
    std::string cameraMake;
    std::string cameraModel;
    double cameraPitchDeg;
    double cameraRollDeg;
    double cameraYawDeg;
    time_t createdTimestampSec;
    time_t downloadedTimestampSec;
    //... grow it as needed

    /**
     * @brief The Earlier struct is a functor usable with std::sort to sort GeoImages
     * by time.
     */
    struct Earlier
    {
        bool operator()(const GeoImage &a, const GeoImage &b) const
        {
            return a.createdTimestampSec == b.createdTimestampSec
                    ? a.path < b.path
                    : a.createdTimestampSec < b.createdTimestampSec;
        }
    };

    static GeoImage fromExif(const std::string &imagePath);
};

/**
 * @brief The Panorama class is a list of images that can potentially be stitched
 * together to form a 360 panorama.
 */
class Panorama : public std::list<GeoImage>
{
public:
    static constexpr long MaxTimeFromPreviousSec = 30;
    static constexpr long MaxSpatialDistanceMts = 5;
    static constexpr double MaxAngularDistanceDeg = 20;
    static constexpr int MinImageCount = 7;
    static constexpr char PanoramaFileExtension[] = ".panorama.jpg";

    struct Parameters
    {
        inline explicit Parameters(
                size_t _memoryBudgetMB,
                bool _alsoCreateCubeMap = true,
                bool _enableOpenCL = true,
                size_t _retries = 6,
                double _maximumCropRatio = 99. / 100,
                size_t _maxInputImageSize =
                        12740198 // empirical (Anafi image cols x rows scaled to 0.8)
                )
            : memoryBudgetMB { _memoryBudgetMB }
            , alsoCreateCubeMap(_alsoCreateCubeMap)
            , enableOpenCL(_enableOpenCL)
            , retries(_retries)
            , maxInputImageSize { _maxInputImageSize }
            , maximumCropRatio { _maximumCropRatio }

        {
        }

        /**
         * @brief Panorama::Parameters::defaultMemoryBudgetMB
         * @details using posix, tries to work out how much RAM is installed on the local HW,
         * removes what it thinks is reasonably in use (~3GB) and considers the rest the available
         * RAM budget.
         * @return An estimation of the available RAM budget.
         */
        static size_t defaultMemoryBudgetMB();

        /**
         * @brief memoryBudgetMB
         *  How much RAM headroom can the stitcher assume it has to its exclusive
         * disposal.
         */
        size_t memoryBudgetMB;

        /**
         * @brief alsoCreateCubeMap
         *  Whether to also genarate cube maps.
         */
        bool alsoCreateCubeMap;

        /**
         * @brief enableOpenCL
         * Enable OpenCL support, if the hardware supports it.
         */
        bool enableOpenCL;

        /**
         * @brief retries
         *  Panorama stitching is indeterministic and it may fail where it would
         * succeed later
         */
        size_t retries;

        /**
         * @brief maxInputImageSize
         *  No of pixels, to which to scale each input image down.
         * @details
         *  We're getting pretty consistently worse results accepting un-scaled images
         * vs those scaled to roughly 0.8 and we think this is because hi-res images
         * produce sufficently many features to overwhelm the feature matcher and thus
         * produce worse results.
         */
        size_t maxInputImageSize;

        /**
         * @brief maximumCropRatio
         * Maximum distortion ratio of the inner crop area to the outer crop area.
         * Closer to zero means to permit more cropping (and distortion of the
         * panorama). Closer to one means to limit the amount of cropped area.
         *
         * If the resulting area would be cropped beyond this ratio then the original
         * imag is returned as if no cropping was performed.
         */
        double maximumCropRatio;
    };

    inline Panorama()
        : _minCreationTimestamp(std::numeric_limits<time_t>::max())
        , _maxCreationTimestamp(0)
        , _lastDownloadedTimestamp(0)
        , _successful(true)
    {
    }

    inline Panorama(const std::list<GeoImage> &images)
        : _minCreationTimestamp(std::numeric_limits<time_t>::max())
        , _maxCreationTimestamp(0)
        , _lastDownloadedTimestamp(0)
        , _successful(true)
    {
        for (const auto& image : images) {
            if (!add(image)) {
                throw std::invalid_argument{"exif of " + image.path + " doesn't fit previous images"};
            }
        }
    }

    inline Panorama(const GeoImage &image)
        : Panorama(std::list<GeoImage>{image})
    {}


    inline time_t minTimestampSec() const { return _minCreationTimestamp; }
    inline time_t maxTimestampSec() const { return _maxCreationTimestamp; }

    geocoordinate_t centre() const { return _geoBBox.centre(); }

    /**
     * @brief add adds the given image to the mix iif the image looks like it may be
     * stichable with the images already held. Notably whether the image's exif is close
     * enough spatially and temporarily.
     * @param image
     * @return true if image was accepted, false otherwise and the caller must continue
     * seeing a Panorama360 instance that can accept the image
     */
    bool add(const GeoImage &image);

    /**
     * @brief returns 'this' broken down into smaller 'jobs' that can be stitched in 2
     * stages.
     * @details Smaller jobs, subsets of stitchable images can be more robust and take
     * less time.
     * @param noSubimages - number of subimages/subjobs to produce
     * @return list of subimages
     */
    std::list<Panorama> subimages(size_t noSubimages) const;

    /**
     * @brief inputPaths returns paths of all constituent images
     * @return paths of all constituent images
     */
    std::list<std::string> inputPaths() const;

    /**
     * @brief debugDescribe
     * @return human readable description of this panorama for debug purposes
     */
    std::string debugDescribe() const;

    /**
     * @brief lastDownloadedTimestamp
     * @return creation timestamp (sec since epoch) of the last downloaded image within
     * this panorama
     */
    time_t lastDownloadedTimestamp() const { return _lastDownloadedTimestamp; }

private:
    bool endsWith(const std::string &stack, const std::string &needle)
    {
        return stack.find(needle, stack.size() - needle.size()) != std::string::npos;
    }

    geo_bounding_box_t _geoBBox;
    time_t _minCreationTimestamp;
    time_t _maxCreationTimestamp;
    time_t _lastDownloadedTimestamp;
    bool _successful;
};

} // namespace stitcher
} // namespace airmap
