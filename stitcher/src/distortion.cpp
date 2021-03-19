#include "airmap/distortion.h"

namespace airmap {
namespace stitcher {

//
// 
// DistortionModel
// 
//
DistortionModel::DistortionModel(bool enabled_, CropROICb crop_roi_cb_)
    : _enabled(enabled_)
    , _crop_roi_cb(crop_roi_cb_)
{
}

DistortionModel::~DistortionModel() { }

const bool DistortionModel::enabled() const
{
    return _enabled;
}

void DistortionModel::crop(cv::Mat &image, cv::Rect &roi) {
    if (_crop_roi_cb) {
        cv::Mat cropped = image(roi);
        cropped.copyTo(image);
    }
}

void DistortionModel::crop(std::vector<cv::Mat> &images) {
    if (_crop_roi_cb) {
        cv::Rect roi = _crop_roi_cb(images[0]);
        for (auto &image : images) {
            crop(image, roi);
        }
    }
}

//
// 
// PinholeDistortionModel::Parameters
// 
//

PinholeDistortionModel::Parameters::Parameters(double _k1, double _k2, double _p1, double _p2, double _k3)
{
    _coefficients = { _k1, _k2, _p1, _p2, _k3 };
}

PinholeDistortionModel::Parameters::Parameters(cv::Mat &coefficients_mat)
{
    _coefficients = matToArray(coefficients_mat);
}

PinholeDistortionModel::Parameters::Parameters(const Parameters &parameters_)
{
    cv::Mat coefficients_mat = parameters_.coefficients();
    _coefficients = matToArray(coefficients_mat);
}

PinholeDistortionModel::Parameters::Parameters()
{
    _coefficients = { 0.0, 0.0, 0.0, 0.0, 0.0 };
}

cv::Mat PinholeDistortionModel::Parameters::coefficients() const
{
    return (cv::Mat_<double>(5, 1) <<
        k1(), k2(), p1(), p2(), k3());
}

double PinholeDistortionModel::Parameters::k1() const
{
    return _coefficients[0];
}

double PinholeDistortionModel::Parameters::k2() const
{
    return _coefficients[1];
}

double PinholeDistortionModel::Parameters::k3() const
{
    return _coefficients[4];
}

double PinholeDistortionModel::Parameters::p1() const
{
    return _coefficients[2];
}

double PinholeDistortionModel::Parameters::p2() const
{
    return _coefficients[3];
}

std::array<double, 5> PinholeDistortionModel::Parameters::matToArray(
    cv::Mat &coefficients_mat)
{
    double k1 = coefficients_mat.at<double>(0, 0);
    double k2 = coefficients_mat.at<double>(1, 0);
    double p1 = coefficients_mat.at<double>(2, 0);
    double p2 = coefficients_mat.at<double>(3, 0);
    double k3 = coefficients_mat.at<double>(4, 0);
    return { k1, k2, p1, p2, k3 };
}

bool PinholeDistortionModel::Parameters::operator==(const Parameters &other) const
{
    return _coefficients == other._coefficients;
}

bool PinholeDistortionModel::Parameters::operator==(const Parameters *other) const
{
    return _coefficients == other->_coefficients;
}

std::ostream& operator<<(std::ostream &os,
                         const PinholeDistortionModel::Parameters &parameters)
{
    return os << "k1: " << std::to_string(parameters.k1())
                << " k2: " << std::to_string(parameters.k2())
                << " k3: " << std::to_string(parameters.k3())
                << " p1: " << std::to_string(parameters.p1())
                << " p2: " << std::to_string(parameters.p2())
                << std::endl;
}

//
// 
// PinholeDistortionModel
// 
//

PinholeDistortionModel::PinholeDistortionModel(const Parameters parameters_,
                                               bool enabled_,
                                               CropROICb crop_roi_cb_)
    : DistortionModel(enabled_, crop_roi_cb_)
    , _parameters(parameters_)
{
}

void PinholeDistortionModel::undistort(cv::Mat &image, cv::InputArray K)
{
    cv::Mat undistorted_image;
    cv::undistort(image, undistorted_image, K, _parameters.coefficients());
    image = undistorted_image;
}

void PinholeDistortionModel::undistort(std::vector<cv::Mat> &images,
                                       cv::InputArray K)
{
    for (size_t i = 0; i < images.size(); ++i) {
        undistort(images[i], K);
    }
}

//
// 
// ScaramuzzaDistortionModel::Parameters
// 
//
ScaramuzzaDistortionModel::Parameters::Parameters(std::vector<double> _pol,
        std::vector<double> _inv_pol, double _xc, double _yc, double _c,
        double _d, double _e, int _width, int _height, double _scale_factor,
        double _resolution_scale)
    : pol(_pol), inv_pol(_inv_pol) , xc(_xc), yc(_yc), c(_c) , d(_d)
    , e(_e), width(_width), height(_height)
    , scale_factor(_scale_factor), resolution_scale(_resolution_scale)
{
}

ScaramuzzaDistortionModel::Parameters::Parameters()
    : pol({ 0.0 }), inv_pol({ 0.0 }), xc(0.0), yc(0.0), c(0.0), d(0.0)
    , e(0.0), width(0.0), height(0.0), scale_factor(0.0), resolution_scale(0.0)
{
}

//
// 
// ScaramuzzaDistortionModel
// 
//
ScaramuzzaDistortionModel::ScaramuzzaDistortionModel(const Parameters parameters_,
                                                     bool enabled_,
                                                     CropROICb crop_roi_cb_)
    : DistortionModel(enabled_, crop_roi_cb_)
    , _parameters(parameters_)
{
}

void ScaramuzzaDistortionModel::createPerspectiveUndistortionMaps(cv::Mat &map_x,
                                                                  cv::Mat &map_y)
{
    int width = map_x.cols;
    int height = map_x.rows;
    float x_center = width / 2.0;
    float y_center = height / 2.0;
    float z = -width / _parameters.scale_factor;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            cv::Point3d world_point(x - x_center, y - y_center, z);
            cv::Point2d camera_point;
            worldToCamera(world_point, camera_point);

            map_x.at<float>(y, x) = static_cast<float>(camera_point.x);
            map_y.at<float>(y, x) = static_cast<float>(camera_point.y);
        }
    }
}

void ScaramuzzaDistortionModel::undistort(cv::Mat &image,
                                          cv::InputArray K)
{
    cv::Mat map_x(image.size().height, image.size().width, CV_32FC1);
    cv::Mat map_y(image.size().height, image.size().width, CV_32FC1);

    createPerspectiveUndistortionMaps(map_x, map_y);

    cv::Mat undistorted_image;
    cv::remap(image, undistorted_image, map_x, map_y, cv::INTER_LINEAR,
              cv::BORDER_CONSTANT, cv::Scalar::all(0));
    image = undistorted_image;
}

void ScaramuzzaDistortionModel::undistort(std::vector<cv::Mat> &images,
                                          cv::InputArray K)
{
    for (size_t i = 0; i < images.size(); ++i) {
        undistort(images[i], K);
    }
}

void ScaramuzzaDistortionModel::worldToCamera(cv::Point3d &world_point,
                                              cv::Point2d &camera_point)
{
    std::vector<double> inv_pol = _parameters.inv_pol;
    double xc = _parameters.xc * _parameters.resolution_scale;
    double yc = _parameters.yc * _parameters.resolution_scale;
    double c = _parameters.c;
    double d = _parameters.d;
    double e = _parameters.e;

    double norm = sqrt(pow(world_point.x, 2) + pow(world_point.y, 2));
    double theta = atan(world_point.z / norm);

    double inv_norm, rho, x, y, t, t_i;

    if (norm != 0) 
    {
        inv_norm = 1 / norm;
        t  = theta;
        rho = inv_pol[0];

        t_i = 1;
        for (int i = 1; i < static_cast<int>(inv_pol.size()); ++i) {
            t_i *= t;
            rho += t_i * inv_pol[i];
        }

        x = world_point.x * inv_norm * rho * _parameters.resolution_scale;
        y = world_point.y * inv_norm * rho * _parameters.resolution_scale;

        camera_point.x = (x*c + y*d) + xc;
        camera_point.y = (x*e + y) + yc;
    } else {
        camera_point.x = xc;
        camera_point.y = yc;
    }
}

} // namespace stitcher
} // namespace airmap
