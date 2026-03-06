// =============================================================================
//  capture_pair.cpp
//  Synchronized image capture for any adjacent camera pair.
//  ZED camera is accessed as a V4L2 device via OpenCV (no ZED SDK required).
//  The ZED outputs side-by-side 2560×720; this tool splits it into L and R.
//
//  Usage:
//    capture_pair --pair <camA> <camB>
//                [--config <cameras.yaml>]
//                [--target <N>]
//                [--zed_dev <V4L2-device-index>]   default: 0
//
//  Supported camera names: cam1, cam2, cam3, cam4, cam5, zed_left, zed_right
//  (cam3 / zed_left / zed_right all refer to the ZED device)
//
//  Key bindings:
//    SPACE  — save synchronized pair (only when ChArUco visible in both)
//    Q      — quit
//
//  Output:
//    data/pair_X_Y/<camA>/NNNN.png
//    data/pair_X_Y/<camB>/NNNN.png
// =============================================================================

#include <iostream>
#include <string>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <memory>
#include <thread>
#include <mutex>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include "calib_common.hpp"

namespace fs = std::filesystem;

static std::string padded(int n, int w = 4)
{
    std::ostringstream ss;
    ss << std::setw(w) << std::setfill('0') << n;
    return ss.str();
}

// ── Abstract camera interface ─────────────────────────────────────────────────

struct ICamera {
    std::string name;
    virtual ~ICamera() = default;
    virtual cv::Mat grab() = 0;
    virtual void   stop() = 0;
};

// ── RealSense (L515 / D405) ───────────────────────────────────────────────────

struct RsCamera : ICamera {
    rs2::pipeline pipe;

    static std::unique_ptr<RsCamera> open(const std::string& camName,
                                           const std::string& serial)
    {
        auto c = std::make_unique<RsCamera>();
        c->name = camName;
        rs2::config cfg;
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
        c->pipe.start(cfg);
        for (int i = 0; i < 5; ++i) c->pipe.wait_for_frames();
        std::cout << "[RS] opened " << camName << " (" << serial << ")\n";
        return c;
    }

    cv::Mat grab() override {
        auto fs = pipe.wait_for_frames();
        auto f  = fs.get_color_frame();
        return cv::Mat(f.get_height(), f.get_width(), CV_8UC3,
                       const_cast<void*>(f.get_data())).clone();
    }

    void stop() override { pipe.stop(); }
};

// ── ZED via V4L2 (OpenCV VideoCapture, no ZED SDK) ───────────────────────────
//
// The ZED camera presents itself as a standard UVC device on Linux.
// In HD720 mode it outputs a 2560×720 side-by-side frame:
//   left  image : x = [0,    1279]
//   right image : x = [1280, 2559]

struct ZedV4lCamera : ICamera {
    cv::VideoCapture cap;
    bool grabLeft;   // true = left half, false = right half

    static std::unique_ptr<ZedV4lCamera> open(const std::string& camName,
                                               int devIndex, bool left)
    {
        auto c      = std::make_unique<ZedV4lCamera>();
        c->name     = camName;
        c->grabLeft = left;

        if (!c->cap.open(devIndex)) {
            std::cerr << "[ZED-V4L2] Cannot open /dev/video" << devIndex << "\n";
            return nullptr;
        }
        // Request side-by-side HD720 resolution
        c->cap.set(cv::CAP_PROP_FRAME_WIDTH,  2560);
        c->cap.set(cv::CAP_PROP_FRAME_HEIGHT,  720);
        c->cap.set(cv::CAP_PROP_FPS,           30);

        // Warm up — discard a few frames
        cv::Mat dummy;
        for (int i = 0; i < 10; ++i) c->cap.read(dummy);

        int w = static_cast<int>(c->cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int h = static_cast<int>(c->cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        std::cout << "[ZED-V4L2] opened " << camName
                  << " (/dev/video" << devIndex << ")"
                  << "  sbs=" << w << "x" << h
                  << "  lens=" << (left ? "left" : "right") << "\n";

        if (w < 2560) {
            std::cerr << "[ZED-V4L2] WARNING: got " << w << "x" << h
                      << " instead of 2560x720. Check that the correct device index "
                      << "is given with --zed_dev.\n";
        }
        return c;
    }

    cv::Mat grab() override {
        cv::Mat sbs;
        if (!cap.read(sbs) || sbs.empty()) return cv::Mat();
        int half = sbs.cols / 2;
        cv::Rect roi = grabLeft ? cv::Rect(0, 0, half, sbs.rows)
                                : cv::Rect(half, 0, half, sbs.rows);
        return sbs(roi).clone();
    }

    void stop() override { cap.release(); }
};

// ── Factory: open camera by logical name ─────────────────────────────────────

static std::unique_ptr<ICamera> openCamera(
    const std::string& name,
    const std::map<std::string, CameraConfig>& configs,
    int zedDevIndex)
{
    auto resolveSerial = [&](const std::string& n) -> std::string {
        auto it = configs.find(n);
        if (it != configs.end()) return it->second.serial;
        auto it3 = configs.find("cam3");
        return (it3 != configs.end()) ? it3->second.serial : "";
    };

    auto resolveModel = [&](const std::string& n) -> std::string {
        auto it = configs.find(n);
        if (it != configs.end()) return it->second.model;
        if (n == "zed_left" || n == "zed_right") return "ZED";
        return "";
    };

    std::string model = resolveModel(name);

    if (model == "L515" || model == "D405") {
        std::string serial = resolveSerial(name);
        if (serial.empty() || serial == "FILL_IN") {
            std::cerr << "[ERROR] Serial for '" << name << "' is not set in cameras.yaml\n";
            return nullptr;
        }
        return RsCamera::open(name, serial);
    }

    if (model == "ZED" || name == "zed_left" || name == "zed_right" || name == "cam3") {
        bool left = (name != "zed_right");
        // Prefer CLI override, then fall back to dev field in cameras.yaml
        int devIdx = (zedDevIndex >= 0) ? zedDevIndex : 0;
        auto it3 = configs.find("cam3");
        if (devIdx == 0 && it3 != configs.end() && it3->second.dev >= 0)
            devIdx = it3->second.dev;
        return ZedV4lCamera::open(name, devIdx, left);
    }

    std::cerr << "[ERROR] Unknown model '" << model << "' for '" << name << "'\n";
    return nullptr;
}

// ── CLI ───────────────────────────────────────────────────────────────────────

static void printHelp(const char* prog)
{
    std::cout
        << "Usage: " << prog
        << " --pair <camA> <camB> [--config <path>] [--target <N>] [--zed_dev <N>]\n"
        << "\n"
        << "  --pair     Two camera names (e.g. cam1 cam2, zed_left zed_right)\n"
        << "  --config   cameras.yaml     (default: configs/cameras.yaml)\n"
        << "  --target   Pairs to save    (default: 40)\n"
        << "  --zed_dev  V4L2 device index for ZED camera (default: 0)\n"
        << "\n"
        << "  SPACE=save  Q=quit\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    std::string camA, camB;
    std::string configPath = "configs/cameras.yaml";
    int target     = 40;
    int zedDevIdx  = -1;  // -1 = read from cameras.yaml dev field

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") { printHelp(argv[0]); return 0; }
        else if (a == "--pair"    && i+2 < argc) { camA = argv[++i]; camB = argv[++i]; }
        else if (a == "--config"  && i+1 < argc) configPath = argv[++i];
        else if (a == "--target"  && i+1 < argc) target     = std::stoi(argv[++i]);
        else if (a == "--zed_dev" && i+1 < argc) zedDevIdx  = std::stoi(argv[++i]);
    }

    if (camA.empty() || camB.empty()) { printHelp(argv[0]); return 1; }

    // Output directory name
    std::string pairDir;
    if ((camA == "zed_left"  && camB == "zed_right") ||
        (camA == "zed_right" && camB == "zed_left")) {
        pairDir = "data/zed_stereo";
    } else {
        auto strip = [](const std::string& s) {
            return (s.rfind("cam", 0) == 0) ? s.substr(3) : s;
        };
        pairDir = "data/pair_" + strip(camA) + "_" + strip(camB);
    }

    fs::create_directories(pairDir + "/" + camA);
    fs::create_directories(pairDir + "/" + camB);

    auto configs  = loadCameraConfigs(configPath);
    auto camObjA  = openCamera(camA, configs, zedDevIdx);
    auto camObjB  = openCamera(camB, configs, zedDevIdx);

    if (!camObjA || !camObjB) {
        if (camObjA) camObjA->stop();
        if (camObjB) camObjB->stop();
        return 1;
    }

    auto board = buildCharucoBoard();

    std::cout << "[CAPTURE] SPACE=save  Q=quit  target=" << target << "\n";
    std::cout << "[CAPTURE] Output: " << pairDir << "\n\n";

    // Returns true if >= 6 charuco corners found; draws marker boundaries + charuco corners
    auto detectAndDraw = [&](const cv::Mat& img,
                              std::vector<int>& charucoIds,
                              std::vector<cv::Point2f>& charucoCorners,
                              cv::Mat& dbg) -> bool
    {
        dbg = img.clone();
        auto dict   = board->dictionary;
        auto params = cv::aruco::DetectorParameters::create();

        std::vector<int>                      markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejected;
        cv::aruco::detectMarkers(img, dict, markerCorners, markerIds, params, rejected);

        if (!markerIds.empty()) {
            // Draw ArUco marker quad boundaries in cyan
            cv::aruco::drawDetectedMarkers(dbg, markerCorners, markerIds,
                                           cv::Scalar(255, 200, 0));
            // Interpolate ChArUco corners
            cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, img, board,
                charucoCorners, charucoIds);
            if (static_cast<int>(charucoIds.size()) >= 15) {
                // Draw ChArUco inner corners in green
                cv::aruco::drawDetectedCornersCharuco(dbg, charucoCorners, charucoIds,
                                                      cv::Scalar(0, 255, 0));
                return true;
            }
        }
        return false;
    };

    // Parallel grab: both cameras capture simultaneously so the board position
    // is consistent between the two images (sequential grab caused ~33ms offset
    // which caused the board to appear in different positions in each camera).
    auto grabBoth = [&](cv::Mat& outA, cv::Mat& outB) {
        cv::Mat tmpA, tmpB;
        std::thread tA([&]{ tmpA = camObjA->grab(); });
        std::thread tB([&]{ tmpB = camObjB->grab(); });
        tA.join();
        tB.join();
        outA = tmpA;
        outB = tmpB;
    };

    int count = 0;
    while (count < target) {
        cv::Mat imgA, imgB;
        grabBoth(imgA, imgB);
        if (imgA.empty() || imgB.empty()) continue;

        std::vector<int>         idsA, idsB;
        std::vector<cv::Point2f> cA, cB;
        cv::Mat dbgA, dbgB;
        bool okA = detectAndDraw(imgA, idsA, cA, dbgA);
        bool okB = detectAndDraw(imgB, idsB, cB, dbgB);

        std::string status =
            "[" + std::to_string(count) + "/" + std::to_string(target) + "] "
            + camA + "=" + (okA ? "OK(" + std::to_string(idsA.size()) + ")" : "--")
            + "  " + camB + "=" + (okB ? "OK(" + std::to_string(idsB.size()) + ")" : "--")
            + "  SPACE=save  Q=quit";
        cv::putText(dbgA, status, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,255}, 2);

        cv::Mat preview;
        cv::hconcat(dbgA, dbgB, preview);
        cv::imshow("capture_pair: " + camA + " | " + camB, preview);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 'Q') break;

        if (key == ' ' && okA && okB) {
            std::string fname = padded(count) + ".png";
            cv::imwrite(pairDir + "/" + camA + "/" + fname, imgA);
            cv::imwrite(pairDir + "/" + camB + "/" + fname, imgB);
            std::cout << "  Saved " << count
                      << "  A=" << idsA.size()
                      << "  B=" << idsB.size() << "\n";
            ++count;
        }
    }

    cv::destroyAllWindows();
    camObjA->stop();
    camObjB->stop();
    std::cout << "\nCaptured " << count << " pairs into " << pairDir << "\n";
    return 0;
}
