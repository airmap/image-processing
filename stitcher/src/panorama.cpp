#include "airmap/panorama.h"
#include "TinyEXIF/TinyEXIF.h"

#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <time.h>

namespace airmap {
namespace filesystem {

path::path() {}

path::path(const std::string &p)
    : _path(p)
{
    if (_path.back() == separator()) {
        _path.erase(_path.size() - 1, 1);
    }
}

path path::parent_path() const
{
    size_t last_slash = _path.find_last_of(separator());
    if (last_slash == std::string::npos) {
        return {};
    }
    return _path.substr(0, last_slash);
}

std::string path::stem() const
{
    std::string result = _path;
    size_t pos = _path.find_last_of(separator());
    if (pos != std::string::npos) {
        result.erase(pos);
    }
    pos = result.find_last_of(".");
    if (pos != std::string::npos) {
        result.erase(0, pos);
    }
    return result;
}
} // namespace filesystem

namespace stitcher {

float geocoordinate_t::distance_metres(const geocoordinate_t &to) const
{
    geocoordinate_t from_rad { deg2Rad * lng(), deg2Rad * lat() };
    geocoordinate_t to_rad { deg2Rad * to.lng(), deg2Rad * to.lat() };

    // Haversine Formula
    long double dlong = from_rad.lng() - to_rad.lng();
    long double dlat = from_rad.lat() - to_rad.lat();

    long double ans = pow(sin(dlat / 2), 2)
            + cos(from_rad.lat()) * cos(to_rad.lat()) * pow(sin(dlong / 2), 2);

    ans = 2 * asin(sqrt(ans));

    // Radius of Earth in
    // Kilometers, R = 6371
    // Use R = 3956 for miles
    long double R = 6371000;

    // Calculate the result
    ans = ans * R;

    return ans;
}

size_t Panorama::Parameters::defaultMemoryBudgetMB()
{
    size_t deviceMemoryMB = 4000ul; // default
    try { // hope for posix
        struct sysinfo si;
        int result = sysinfo(&si);
        if (result == 0) {
            deviceMemoryMB = si.totalram / (1024 * 1024);
        }
    } catch (...) {
    }
    return deviceMemoryMB
            - std::min(3000ul,
                       deviceMemoryMB / 2); // assume half, but no more than
                                            // 3GB of RAM is in use by other apps.
}

GeoImage GeoImage::fromExif(const std::string &imagePath)
{
    // The usage of tinyexif advises to gulp up the entire file. We're dealing with VERY
    // large files here, so it costs a lot do so (see
    // https://github.com/cdcseacave/TinyEXIF/issues/7) We thus only read up to the limit
    // of 120KB based on the assumption stated here:
    // https://dev.exiv2.org/projects/exiv2/wiki/The_Metadata_in_JPEG_files
    //   In theory, Exif APP1 is recorded immediately after the SOI marker (the marker
    //   indicating the beginning of the file). However, this leads to the incompatibility
    //   between the Exif and JFIF standards because both of them specify that their
    //   particular application segment (APP0 for JFIF, APP1 for Exif) must be the first
    //   in the image file. In practice, most JPEG files contain a JFIF marker segment
    //   (APP0) that precedes the Exif APP1. This allows older readers to correctly handle
    //   the format JFIF segment, while newer readers also decode the following Exif
    //   segment, being less strict about requiring it to appear first.
    // and based on the fact that (see: https://code.tools/man/3pm/Image::MetaData::JPEG/)
    //   a JPEG Segment cannot be longer than 64KB;
    static const size_t max_offset = 1024u * 64 * 2;
    std::ifstream file(imagePath, std::ifstream::in | std::ifstream::binary);
    file.seekg(0, std::ios::end);
    size_t length = std::min(static_cast<size_t>(file.tellg()), max_offset);
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> data(length);
    file.read((char *)data.data(), length);

    // parse image EXIF and XMP metadata
    TinyEXIF::EXIFInfo imageEXIF(data.data(), length);
    if (!imageEXIF.Fields) {
        throw std::invalid_argument("Can't extract exif metadata from" + imagePath);
    }
    struct stat fileInfo;
    if (stat(imagePath.c_str(), &fileInfo) != 0) { // Use stat( ) to get the info
        throw std::invalid_argument("Can't extract exif metadata from" + imagePath);
    }
    time_t file_created = fileInfo.st_mtime;
    assert(file_created > 0);

    std::tm image_created = {};
    strptime(imageEXIF.DateTime.c_str(), "%Y:%m:%d %H:%M:%S", &image_created);

    return GeoImage { imagePath,
                      geocoordinate_t(imageEXIF.GeoLocation.Longitude,
                                      imageEXIF.GeoLocation.Latitude),
                      imageEXIF.Make,
                      imageEXIF.Model,
                      imageEXIF.GeoLocation.PitchDegree,
                      imageEXIF.GeoLocation.RollDegree,
                      imageEXIF.GeoLocation.YawDegree,
                      std::max(static_cast<time_t>(0), std::mktime(&image_created)),
                      file_created };
}

constexpr char Panorama::PanoramaFileExtension[];

bool Panorama::add(const GeoImage &image)
{
    assert(image.createdTimestampSec >= _maxCreationTimestamp);
    if (size() > 0
        && (image.geoCoordinate.distance_metres(_geoBBox.centre()) > MaxSpatialDistanceMts
            || image.createdTimestampSec - _maxCreationTimestamp > MaxTimeFromPreviousSec
            || filesystem::path(image.path).parent_path()
                    != filesystem::path(front().path).parent_path())) {
        return false;
    }
    if (!endsWith(image.path, PanoramaFileExtension)) {
        _geoBBox.add(image.geoCoordinate);
        _minCreationTimestamp =
                std::min(_minCreationTimestamp, image.createdTimestampSec);
        _maxCreationTimestamp =
                std::max(_maxCreationTimestamp, image.createdTimestampSec);
        _lastDownloadedTimestamp =
                std::max(_lastDownloadedTimestamp, image.downloadedTimestampSec);
        push_back(image);
    }
    return true;
}

std::list<std::string> Panorama::inputPaths() const
{
    time_t prevts = 0;
    std::list<std::string> result;
    for (const auto &img : *this) {
        result.push_back(img.path);
        assert(prevts <= img.createdTimestampSec);
        prevts = img.createdTimestampSec;
    }
    return result;
}

std::string Panorama::debugDescribe() const
{
    std::stringstream ss;
    ss << "Panorama with " << size() << " images in "
       << filesystem::path(inputPaths().front()).parent_path().string()
       << ", taken (exif): ["
       << std::put_time(gmtime(&_minCreationTimestamp), "%Y-%m-%d %H:%M:%S") << ".."
       << std::put_time(gmtime(&_maxCreationTimestamp), "%Y-%m-%d %H:%M:%S") << "]";
    return ss.str();
}

} // namespace stitcher
} // namespace airmap
