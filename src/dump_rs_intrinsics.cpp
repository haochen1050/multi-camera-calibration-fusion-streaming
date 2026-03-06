// =============================================================================
//  dump_rs_intrinsics.cpp
//  Read factory intrinsics from a RealSense camera via the SDK and save them
//  to a YAML file in the same format as calibrate_intrinsics.cpp output.
//  This avoids hardcoding intrinsics while still satisfying the requirement
//  that calibrate_extrinsics_pair reads intrinsics from YAML.
//
//  Usage:
//    dump_rs_intrinsics --camera <name> --serial <serial> [--out <dir>]
//
//  Or use --config to read serial from cameras.yaml:
//    dump_rs_intrinsics --camera cam1 [--config configs/cameras.yaml] [--out dir]
// =============================================================================

#include <iostream>
#include <string>
#include <filesystem>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include "calib_common.hpp"

namespace fs = std::filesystem;

static void printHelp(const char* prog)
{
    std::cout
        << "Usage: " << prog
        << " --camera <name> [--serial <serial>] [--config <cameras.yaml>] [--out <dir>]\n"
        << "\n"
        << "  --camera   Camera logical name  (e.g. cam1, cam4)\n"
        << "  --serial   Override serial number\n"
        << "  --config   cameras.yaml path  (default: configs/cameras.yaml)\n"
        << "  --out      Output directory   (default: results/intrinsics)\n"
        << "  --width    Color stream width  (default: 1280)\n"
        << "  --height   Color stream height (default: 720)\n";
}

int main(int argc, char** argv)
{
    std::string camName, serial;
    std::string configPath = "configs/cameras.yaml";
    std::string outDir     = "results/intrinsics";
    int capWidth  = 1280;
    int capHeight = 720;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") { printHelp(argv[0]); return 0; }
        else if (a == "--camera" && i+1 < argc) camName    = argv[++i];
        else if (a == "--serial" && i+1 < argc) serial     = argv[++i];
        else if (a == "--config" && i+1 < argc) configPath = argv[++i];
        else if (a == "--out"    && i+1 < argc) outDir     = argv[++i];
        else if (a == "--width"  && i+1 < argc) capWidth   = std::stoi(argv[++i]);
        else if (a == "--height" && i+1 < argc) capHeight  = std::stoi(argv[++i]);
    }

    if (camName.empty()) { printHelp(argv[0]); return 1; }

    if (serial.empty()) {
        auto configs = loadCameraConfigs(configPath);
        auto it = configs.find(camName);
        if (it == configs.end()) {
            std::cerr << "[ERROR] Camera '" << camName << "' not in " << configPath << "\n";
            return 1;
        }
        serial = it->second.serial;
    }

    // Start pipeline briefly to read intrinsics
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR, capWidth, capHeight, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 0, 0, RS2_FORMAT_Z16, 30);

    rs2::pipeline_profile profile = pipe.start(cfg);

    auto color_intr = profile.get_stream(RS2_STREAM_COLOR)
                             .as<rs2::video_stream_profile>().get_intrinsics();
    auto depth_intr = profile.get_stream(RS2_STREAM_DEPTH)
                             .as<rs2::video_stream_profile>().get_intrinsics();

    // Also get the depth sensor scale
    float depth_scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();

    pipe.stop();

    // Build K and D from SDK intrinsics (color stream)
    cv::Mat K = (cv::Mat_<double>(3,3) <<
        color_intr.fx, 0,             color_intr.ppx,
        0,             color_intr.fy, color_intr.ppy,
        0,             0,             1.0);

    cv::Mat D = (cv::Mat_<double>(1,5) <<
        color_intr.coeffs[0], color_intr.coeffs[1],
        color_intr.coeffs[2], color_intr.coeffs[3], color_intr.coeffs[4]);

    // Also save depth intrinsics separately
    cv::Mat Kd = (cv::Mat_<double>(3,3) <<
        depth_intr.fx, 0,              depth_intr.ppx,
        0,             depth_intr.fy,  depth_intr.ppy,
        0,             0,              1.0);

    cv::Mat Dd = (cv::Mat_<double>(1,5) <<
        depth_intr.coeffs[0], depth_intr.coeffs[1],
        depth_intr.coeffs[2], depth_intr.coeffs[3], depth_intr.coeffs[4]);

    std::cout << "[" << camName << "] Color intrinsics from SDK:\n"
              << "  fx=" << color_intr.fx << " fy=" << color_intr.fy
              << " cx=" << color_intr.ppx << " cy=" << color_intr.ppy << "\n"
              << "  size: " << color_intr.width << "x" << color_intr.height << "\n"
              << "  dist: " << D << "\n";

    fs::create_directories(outDir);
    std::string outPath = outDir + "/" + camName + ".yaml";
    {
        cv::FileStorage fs(outPath, cv::FileStorage::WRITE);
        fs << "camera_name"    << camName;
        fs << "serial"         << serial;
        // Color (used for extrinsic calibration)
        fs << "image_width"    << color_intr.width;
        fs << "image_height"   << color_intr.height;
        fs << "K"              << K;
        fs << "D"              << D;
        // Depth (for reference)
        fs << "depth_width"    << depth_intr.width;
        fs << "depth_height"   << depth_intr.height;
        fs << "Kd"             << Kd;
        fs << "Dd"             << Dd;
        fs << "depth_scale"    << (double)depth_scale;
        fs << "rms"            << -1.0;   // SDK-provided, not calibrated
        fs.release();
    }
    std::cout << "[SAVE] " << outPath << "\n";
    return 0;
}
