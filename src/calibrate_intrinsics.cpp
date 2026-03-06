// =============================================================================
//  calibrate_intrinsics.cpp
//  ChArUco-based intrinsic calibration for a single camera (ZED left/right).
//  RealSense cameras do NOT use this tool — use dump_rs_intrinsics instead.
//
//  Usage:
//    calibrate_intrinsics --images <folder> --camera <name> [--out <dir>]
//
//  Output:
//    results/intrinsics/<camera_name>.yaml  (K 3×3, D distortion, image_size, rms)
// =============================================================================

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include "calib_common.hpp"

namespace fs = std::filesystem;

static void printHelp(const char* prog)
{
    std::cout
        << "Usage: " << prog
        << " --images <folder> --camera <name> [--out <results/intrinsics>]\n"
        << "\n"
        << "  --images   Folder of PNG/JPG images of the ChArUco board\n"
        << "  --camera   Camera name  (e.g. zed_left, zed_right)\n"
        << "  --out      Output directory  (default: results/intrinsics)\n"
        << "\n"
        << "  Board: " << CHARUCO_SQUARES_X << "x" << CHARUCO_SQUARES_Y
        << "  sq=" << CHARUCO_SQUARE_SIZE*1000.f << "mm"
        << "  mk=" << CHARUCO_MARKER_SIZE*1000.f << "mm  DICT_5X5_250\n";
}

int main(int argc, char** argv)
{
    std::string imagesDir, cameraName;
    std::string outDir = "results/intrinsics";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") { printHelp(argv[0]); return 0; }
        else if (a == "--images" && i+1 < argc) imagesDir  = argv[++i];
        else if (a == "--camera" && i+1 < argc) cameraName = argv[++i];
        else if (a == "--out"    && i+1 < argc) outDir     = argv[++i];
    }

    if (imagesDir.empty() || cameraName.empty()) {
        printHelp(argv[0]);
        return 1;
    }
    if (!fs::is_directory(imagesDir)) {
        std::cerr << "[ERROR] Not a directory: " << imagesDir << "\n";
        return 1;
    }

    auto board = buildCharucoBoard();

    // Collect per-image detections
    std::vector<std::vector<cv::Point2f>> allCorners;
    std::vector<std::vector<int>>         allIds;
    cv::Size imgSize;

    int total = 0, accepted = 0;
    for (const auto& entry : fs::directory_iterator(imagesDir)) {
        std::string ext = entry.path().extension().string();
        if (ext != ".png" && ext != ".jpg" && ext != ".jpeg") continue;
        ++total;

        cv::Mat img = cv::imread(entry.path().string());
        if (img.empty()) { std::cerr << "  [WARN] Cannot read: " << entry.path() << "\n"; continue; }
        imgSize = img.size();

        std::vector<int>         ids;
        std::vector<cv::Point2f> corners;
        if (detectCharuco(img, board, ids, corners, 6)) {
            allCorners.push_back(corners);
            allIds.push_back(ids);
            ++accepted;
        }
    }

    std::cout << "[INFO] Images: total=" << total << "  accepted=" << accepted << "\n";
    if (accepted < 10) {
        std::cerr << "[ERROR] Need at least 10 accepted frames (got " << accepted << ")\n";
        return 1;
    }

    // Build object / image point arrays
    std::vector<std::vector<cv::Point3f>> allObjPts;
    std::vector<std::vector<cv::Point2f>> allImgPts;
    for (size_t i = 0; i < allCorners.size(); ++i) {
        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> img2;
        getObjectPoints(board, allIds[i], allCorners[i], obj, img2);
        if (static_cast<int>(obj.size()) >= 6) {
            allObjPts.push_back(obj);
            allImgPts.push_back(img2);
        }
    }

    std::cout << "[CALIBRATE] calibrateCamera with " << allObjPts.size() << " frames...\n";

    cv::Mat K, dist;
    std::vector<cv::Mat> rvecs, tvecs;
    auto criteria = cv::TermCriteria(
        cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 200, 1e-7);

    double rms = cv::calibrateCamera(
        allObjPts, allImgPts, imgSize,
        K, dist, rvecs, tvecs,
        0,   // flags: no CALIB_RATIONAL_MODEL needed for ZED (well-behaved lens)
        criteria);

    std::cout << "  RMS reprojection error: " << rms << " px\n";
    std::cout << "  K:\n" << K << "\n";
    std::cout << "  D: " << dist << "\n";

    fs::create_directories(outDir);
    std::string outPath = outDir + "/" + cameraName + ".yaml";
    {
        cv::FileStorage fs(outPath, cv::FileStorage::WRITE);
        fs << "camera_name"  << cameraName;
        fs << "image_width"  << imgSize.width;
        fs << "image_height" << imgSize.height;
        fs << "K"            << K;
        fs << "D"            << dist;
        fs << "rms"          << rms;
        fs.release();
    }
    std::cout << "[SAVE] " << outPath << "\n";
    return 0;
}
