// =============================================================================
//  chain_transforms.cpp
//  Load four pairwise extrinsic YAMLs and chain them to Camera 1's frame.
//
//  Usage:
//    chain_transforms [--extr <results/extrinsics>] [--out <results/extrinsics>]
//
//  Reads:
//    <extr>/cam1_cam2.yaml   →  T_1_2
//    <extr>/cam2_cam3.yaml   →  T_2_3
//    <extr>/cam3_cam4.yaml   →  T_3_4
//    <extr>/cam4_cam5.yaml   →  T_4_5
//
//  Writes:
//    <out>/all_to_cam1.yaml
//      T_1_1  (identity)
//      T_1_2
//      T_1_3 = T_1_2 × T_2_3
//      T_1_4 = T_1_3 × T_3_4
//      T_1_5 = T_1_4 × T_4_5
// =============================================================================

#include <iostream>
#include <string>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "calib_common.hpp"

namespace fs = std::filesystem;

static void printHelp(const char* prog)
{
    std::cout
        << "Usage: " << prog
        << " [--extr <results/extrinsics>] [--out <results/extrinsics>]\n"
        << "\n"
        << "  --extr   Directory containing pairwise YAML files (default: results/extrinsics)\n"
        << "  --out    Output directory for all_to_cam1.yaml   (default: results/extrinsics)\n";
}

static cv::Mat loadT44(const std::string& path, const std::string& key = "T_a_b")
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERROR] Cannot open: " << path << "\n";
        return cv::Mat();
    }
    cv::Mat T;
    fs[key] >> T;
    if (T.empty()) {
        // Fallback: try building from R/T
        cv::Mat R, Tvec;
        fs["R"] >> R;
        fs["T"] >> Tvec;
        if (!R.empty() && !Tvec.empty())
            T = makeT44(R, Tvec);
    }
    return T;
}

int main(int argc, char** argv)
{
    std::string extrDir = "results/extrinsics";
    std::string outDir  = "results/extrinsics";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") { printHelp(argv[0]); return 0; }
        else if (a == "--extr" && i+1 < argc) extrDir = argv[++i];
        else if (a == "--out"  && i+1 < argc) outDir  = argv[++i];
    }

    // Load pairwise transforms
    cv::Mat T_1_2 = loadT44(extrDir + "/cam1_cam2.yaml");
    cv::Mat T_2_3 = loadT44(extrDir + "/cam2_cam3.yaml");
    cv::Mat T_3_4 = loadT44(extrDir + "/cam3_cam4.yaml");
    cv::Mat T_4_5 = loadT44(extrDir + "/cam4_cam5.yaml");

    if (T_1_2.empty() || T_2_3.empty() || T_3_4.empty() || T_4_5.empty()) {
        std::cerr << "[ERROR] One or more pairwise YAML files are missing or empty.\n";
        return 1;
    }

    // Convert to CV_64F just in case
    T_1_2.convertTo(T_1_2, CV_64F);
    T_2_3.convertTo(T_2_3, CV_64F);
    T_3_4.convertTo(T_3_4, CV_64F);
    T_4_5.convertTo(T_4_5, CV_64F);

    // Chain
    cv::Mat T_1_1 = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_1_3 = T_1_2 * T_2_3;
    cv::Mat T_1_4 = T_1_3 * T_3_4;
    cv::Mat T_1_5 = T_1_4 * T_4_5;

    // Print translation vectors as sanity check
    std::cout << "Transform chain summary (translations from Camera 1 origin):\n";
    printTransform("T_1_1 (identity):", T_1_1);
    printTransform("T_1_2:", T_1_2);
    printTransform("T_1_3:", T_1_3);
    printTransform("T_1_4:", T_1_4);
    printTransform("T_1_5:", T_1_5);

    // Save
    fs::create_directories(outDir);
    std::string outPath = outDir + "/all_to_cam1.yaml";
    {
        cv::FileStorage fs(outPath, cv::FileStorage::WRITE);
        fs << "T_1_1" << T_1_1;
        fs << "T_1_2" << T_1_2;
        fs << "T_1_3" << T_1_3;
        fs << "T_1_4" << T_1_4;
        fs << "T_1_5" << T_1_5;
        fs.release();
    }
    std::cout << "[SAVE] " << outPath << "\n";
    return 0;
}
