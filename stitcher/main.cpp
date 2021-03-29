#include <boost/program_options.hpp>
#include <iostream>
#include <unistd.h>

#include "airmap/opencv_stitcher.h"
using namespace airmap::stitcher;
using namespace airmap::logging;

int main(int argc, char *argv[])
{
    boost::program_options::options_description desc(
            std::string(argv[0]) + " [OPTIONS] image1.jpg image2.jpg ... imageN.jpg\n"
                "stitches the given images into an equirectangular 360 panorama");
    desc.add_options()
            ("help", "show help")
            ("input", boost::program_options::value<std::vector<std::string>>(),
                            "input images")
            ("input_path", "input images will be sought for in this folder")
            ("output", boost::program_options::value<std::string>()->default_value("./panorama.jpg"),
                "path to the resulting equirectangular stitching")
            ("cubemap", "if set, also generates cubemap in <output>.<face>.jpg")
            ("ram_budget",
                boost::program_options::value<size_t>()->default_value(Panorama::Parameters::defaultMemoryBudgetMB()),
                "RAM buget (in MB) the stitcher can assume it can use")
            ("retries",
                boost::program_options::value<size_t>()->default_value(6),
                "The stitching process is non-deterministic, this increases chances to succeed")
            ("debug", "If set, debug artifacts (e.g. detected features, matches, warping, etc.) will be created in <debug_output>.")
            ("debug_path", boost::program_options::value<std::string>()->default_value("debug"),
                "Path to the debug output.")
            ("disable_opencl", "If set, OpenCL will be disabled.")
            ("elapsed_time", "Track elapsed times of the main stitch operations.  Always enabled if elapsed_time_log is.")
            ("elapsed_time_log", "Log elapsed times of stitch operations.")
            ("estimate", "Estimate time remaining during stitch.  Always enabled if elapsed_time or estimate_log are.")
            ("estimate_log", "Log estimates of remaining time.  Always enabled if elapsed_time_log is.")
            ;
    try {
        boost::program_options::positional_options_description positional;
        positional.add("input", -1);

        boost::program_options::variables_map vm;
        boost::program_options::store(
            boost::program_options::command_line_parser(argc, argv)
            .options(desc)
            .positional(positional)
            .run(),
            vm
        );
        boost::program_options::notify(vm);

        if(vm.count("help") || !vm.count("input")) {
          std::cout << desc << "\n";
          return EXIT_FAILURE;
        }

        std::list<GeoImage> input;
        for (std::string path : vm["input"].as<std::vector<std::string>>()) {
            if (vm.count("input_path")) {
                path = (boost::filesystem::path(vm["input_path"].as<std::string>()) / path).string();
            }
            input.push_back(GeoImage::fromExif(path));
        }

        std::string debugPath;
        if (vm.count("debug")) {
            debugPath = boost::filesystem::path(vm["debug_path"].as<std::string>()).string();
        }

        auto logger = std::make_shared<stdoe_logger>();
        Panorama::Parameters parameters{
            vm["ram_budget"].as<size_t>(),
            vm.count("cubemap") > 0,
            vm.count("disable_opencl") <= 0,
            vm.count("elapsed_time") > 0,
            vm.count("elapsed_time_log") > 0,
            vm.count("estimate") > 0,
            vm.count("estimate_log") > 0,
            vm["retries"].as<size_t>()
        };
        RetryingStitcher{
            std::make_shared<LowLevelOpenCVStitcher>(
                Configuration(
                    StitchType::ThreeSixty),
                Panorama{input},
                parameters,
                vm["output"].as<std::string>(),
                logger,
                vm.count("debug") > 0,
                debugPath
            ), parameters, logger
        }.stitch();
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    return EXIT_FAILURE;
}
