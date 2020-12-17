#include <boost/program_options.hpp>
#include <iostream>
#include <unistd.h>

#include "airmap/opencv_stitcher.h"
using namespace airmap::stitcher;

class stdoe_logger : public airmap::Logger {
    public:
    void log(Severity severity, const char* message, const char* component) override {
        switch (severity){
            case Severity::info:
                std::cout << "[" << component << "]>" << message << std::flush;
            break;
            case Severity::debug:
                std::cout << "[" << component << "]>" << message << std::flush;
            break;
            case Severity::error:
                std::cerr << "[" << component << "]>" << message << std::flush;
            break;
        }
    }

    bool should_log(Severity, const char*, const char*) override {
        return true;
    }
};

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
            ("cubemap", "if set, also generates cubmap in <output>.<face>.jpg")
            ("ram_budget",
                boost::program_options::value<size_t>()->default_value(Panorama::Parameters::defaultMemoryBudgetMB()),
                "RAM buget (in MB) the stitcher can assume it can use")
            ("retries",
                boost::program_options::value<size_t>()->default_value(6),
                "The stitching process is non-deterministic, this increases chances to succeed");
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
                path = (airmap::filesystem::path(vm["input_path"].as<std::string>()) / path).string();
            }
            input.push_back(GeoImage::fromExif(path));
        }

        auto logger = std::make_shared<stdoe_logger>();
        Panorama::Parameters parameters{
            vm["ram_budget"].as<size_t>(),
            vm.count("cubemap") > 0,
            vm["retries"].as<size_t>()
        };
        RetryingStitcher{
            std::make_shared<OpenCVStitcher>(
                Panorama{input},
                parameters,
                vm["output"].as<std::string>(),
                logger
            ), parameters, logger
        }.stitch();
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    return EXIT_FAILURE;
}
