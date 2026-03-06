// =============================================================================
//  calibrate_zed_stereo.cpp
//  ZED internal stereo calibration (Left ↔ Right) using ChArUco.
//  ZED is accessed as a V4L2 device via OpenCV (no ZED SDK required).
//  The ZED outputs a side-by-side 2560×720 frame; this tool splits it.
//
//  Usage:
//    calibrate_zed_stereo [--zed_dev <V4L2-index>]
//                         [--target <N>]
//                         [--out_intr <results/intrinsics>]
//                         [--out_extr <results/extrinsics>]
//                         [--images_dir <data/zed_stereo>]  (skip live capture)
//
//  Output:
//    results/intrinsics/zed_left.yaml
//    results/intrinsics/zed_right.yaml
//    results/extrinsics/zed_left_right.yaml
// =============================================================================

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <iomanip>
#include <sstream>

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

// ── Split side-by-side ZED frame ──────────────────────────────────────────────

static void splitSBS(const cv::Mat& sbs, cv::Mat& left, cv::Mat& right)
{
    int half = sbs.cols / 2;
    left  = sbs(cv::Rect(0,    0, half, sbs.rows)).clone();
    right = sbs(cv::Rect(half, 0, half, sbs.rows)).clone();
}

// ── CLI ───────────────────────────────────────────────────────────────────────

static void printHelp(const char* prog)
{
    std::cout
        << "Usage: " << prog << "\n"
        << "  [--zed_dev <N>]              V4L2 device index (default: from cameras.yaml dev field)\n"
        << "  [--target <N>]               pairs to capture (default: 50)\n"
        << "  [--out_intr <dir>]           (default: results/intrinsics)\n"
        << "  [--out_extr <dir>]           (default: results/extrinsics)\n"
        << "  [--images_dir <dir>]         use pre-saved images instead of live capture\n"
        << "                               expects <dir>/zed_left/ and <dir>/zed_right/\n"
        << "\n"
        << "  SPACE=save  Q=quit\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    int         zedDev    = -1;  // -1 = read from cameras.yaml dev field
    int         target    = 50;
    std::string configPath  = "configs/cameras.yaml";
    std::string outIntrDir  = "results/intrinsics";
    std::string outExtrDir  = "results/extrinsics";
    std::string imagesDir;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") { printHelp(argv[0]); return 0; }
        else if (a == "--zed_dev"    && i+1 < argc) zedDev    = std::stoi(argv[++i]);
        else if (a == "--config"     && i+1 < argc) configPath = argv[++i];
        else if (a == "--target"     && i+1 < argc) target    = std::stoi(argv[++i]);
        else if (a == "--out_intr"   && i+1 < argc) outIntrDir = argv[++i];
        else if (a == "--out_extr"   && i+1 < argc) outExtrDir = argv[++i];
        else if (a == "--images_dir" && i+1 < argc) imagesDir  = argv[++i];
    }

    auto board = buildCharucoBoard();

    std::vector<std::vector<cv::Point2f>> allCornersL, allCornersR;
    std::vector<std::vector<int>>         allIdsL,     allIdsR;
    cv::Size imgSizeL, imgSizeR;

    // Resolve ZED device index: CLI overrides config, config overrides default 0
    if (zedDev < 0) {
        auto configs = loadCameraConfigs(configPath);
        auto it3 = configs.find("cam3");
        zedDev = (it3 != configs.end() && it3->second.dev >= 0) ? it3->second.dev : 0;
    }
    std::cout << "[INFO] ZED V4L2 device: /dev/video" << zedDev << "\n";

    if (!imagesDir.empty()) {
        // ── Load pre-captured images ───────────────────────────────────────────
        std::string dirL = imagesDir + "/zed_left";
        std::string dirR = imagesDir + "/zed_right";

        auto collectSorted = [](const std::string& d) {
            std::vector<fs::path> v;
            for (auto& e : fs::directory_iterator(d)) {
                std::string ext = e.path().extension().string();
                if (ext == ".png" || ext == ".jpg") v.push_back(e.path());
            }
            std::sort(v.begin(), v.end());
            return v;
        };

        auto filesL = collectSorted(dirL);
        auto filesR = collectSorted(dirR);
        size_t n = std::min(filesL.size(), filesR.size());

        for (size_t i = 0; i < n; ++i) {
            cv::Mat imgL = cv::imread(filesL[i].string());
            cv::Mat imgR = cv::imread(filesR[i].string());
            if (imgL.empty() || imgR.empty()) continue;
            imgSizeL = imgL.size();
            imgSizeR = imgR.size();

            std::vector<int>         idsL, idsR;
            std::vector<cv::Point2f> cL, cR;
            bool okL = detectCharuco(imgL, board, idsL, cL, 6);
            bool okR = detectCharuco(imgR, board, idsR, cR, 6);
            if (okL && okR) {
                allCornersL.push_back(cL); allIdsL.push_back(idsL);
                allCornersR.push_back(cR); allIdsR.push_back(idsR);
            }
        }
        std::cout << "[INFO] Loaded " << allCornersL.size()
                  << " accepted pairs from " << imagesDir << "\n";
    }
    else {
        // ── Live ZED V4L2 capture ──────────────────────────────────────────────
        cv::VideoCapture cap(zedDev);
        if (!cap.isOpened()) {
            std::cerr << "[ERROR] Cannot open /dev/video" << zedDev << "\n";
            return 1;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  2560);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT,  720);
        cap.set(cv::CAP_PROP_FPS,           30);

        int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        std::cout << "[ZED-V4L2] /dev/video" << zedDev << "  sbs=" << w << "x" << h << "\n";
        if (w < 2560)
            std::cerr << "[WARN] Expected 2560x720 but got " << w << "x" << h
                      << " — check --zed_dev index\n";

        // Warm up
        cv::Mat dummy;
        for (int i = 0; i < 10; ++i) cap.read(dummy);

        std::string saveDir = "data/zed_stereo";
        fs::create_directories(saveDir + "/zed_left");
        fs::create_directories(saveDir + "/zed_right");

        std::cout << "[CAPTURE] SPACE=save  Q=quit  target=" << target << "\n";
        int count = 0;

        while (count < target) {
            cv::Mat sbs;
            if (!cap.read(sbs) || sbs.empty()) continue;

            cv::Mat imgL, imgR;
            splitSBS(sbs, imgL, imgR);
            imgSizeL = imgL.size();
            imgSizeR = imgR.size();

            auto detectAndDraw = [&](const cv::Mat& img,
                                      std::vector<int>& charucoIds,
                                      std::vector<cv::Point2f>& charucoCorners,
                                      cv::Mat& dbg) -> bool
            {
                dbg = img.clone();
                auto dict   = board->dictionary;
                auto params = cv::aruco::DetectorParameters::create();
                std::vector<int>                        markerIds;
                std::vector<std::vector<cv::Point2f>>   markerCorners, rejected;
                cv::aruco::detectMarkers(img, dict, markerCorners, markerIds, params, rejected);
                if (!markerIds.empty()) {
                    // Cyan: ArUco marker boundary boxes
                    cv::aruco::drawDetectedMarkers(dbg, markerCorners, markerIds,
                                                   cv::Scalar(255, 200, 0));
                    cv::aruco::interpolateCornersCharuco(
                        markerCorners, markerIds, img, board,
                        charucoCorners, charucoIds);
                    if (static_cast<int>(charucoIds.size()) >= 6) {
                        // Green: ChArUco inner corners
                        cv::aruco::drawDetectedCornersCharuco(dbg, charucoCorners, charucoIds,
                                                              cv::Scalar(0, 255, 0));
                        return true;
                    }
                }
                return false;
            };

            std::vector<int>         idsL, idsR;
            std::vector<cv::Point2f> cL, cR;
            cv::Mat dbgL, dbgR;
            bool okL = detectAndDraw(imgL, idsL, cL, dbgL);
            bool okR = detectAndDraw(imgR, idsR, cR, dbgR);

            std::string status =
                "[" + std::to_string(count) + "/" + std::to_string(target) + "]"
                " L=" + (okL ? "OK(" + std::to_string(idsL.size()) + ")" : "--")
                + " R=" + (okR ? "OK(" + std::to_string(idsR.size()) + ")" : "--")
                + "  SPACE=save  Q=quit";
            cv::putText(dbgL, status, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,255}, 2);

            cv::Mat preview;
            cv::hconcat(dbgL, dbgR, preview);
            cv::imshow("ZED stereo calib: LEFT | RIGHT", preview);

            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 'Q') break;
            if (key == ' ' && okL && okR) {
                std::string fname = padded(count) + ".png";
                cv::imwrite(saveDir + "/zed_left/"  + fname, imgL);
                cv::imwrite(saveDir + "/zed_right/" + fname, imgR);
                allCornersL.push_back(cL); allIdsL.push_back(idsL);
                allCornersR.push_back(cR); allIdsR.push_back(idsR);
                std::cout << "  Saved " << count
                          << "  L=" << idsL.size() << " R=" << idsR.size() << "\n";
                ++count;
            }
        }
        cv::destroyAllWindows();
        cap.release();
    }

    if (allCornersL.size() < 10) {
        std::cerr << "[ERROR] Not enough pairs: " << allCornersL.size()
                  << " (need >= 10)\n";
        return 1;
    }

    // ── Helper: assemble obj/img point lists from charuco detections ─────────
    // Returns per-frame vectors; skips frames with < minCorners or collinear pts.
    auto assemblePoints = [&](
        const std::vector<std::vector<cv::Point2f>>& corners,
        const std::vector<std::vector<int>>&         ids,
        int minCorners,
        std::vector<std::vector<cv::Point3f>>& allObj,
        std::vector<std::vector<cv::Point2f>>& allImg)
    {
        allObj.clear(); allImg.clear();
        int nBC = static_cast<int>(board->chessboardCorners.size());
        for (size_t i = 0; i < corners.size(); ++i) {
            if (static_cast<int>(ids[i].size()) < minCorners) continue;
            std::vector<cv::Point3f> obj;
            std::vector<cv::Point2f> img;
            for (size_t j = 0; j < ids[i].size(); ++j) {
                int id = ids[i][j];
                if (id >= 0 && id < nBC) {
                    obj.push_back(board->chessboardCorners[id]);
                    img.push_back(corners[i][j]);
                }
            }
            if (static_cast<int>(obj.size()) < minCorners) continue;
            // Skip collinear frames — fisheye::calibrate requires 2D point spread.
            // Check by seeing if points span at least 2 distinct board rows AND cols.
            std::set<int> cols, rows;
            for (size_t j = 0; j < ids[i].size(); ++j) {
                int id = ids[i][j];
                if (id >= 0 && id < nBC) {
                    cols.insert(id % (CHARUCO_SQUARES_X - 1));
                    rows.insert(id / (CHARUCO_SQUARES_X - 1));
                }
            }
            if (cols.size() < 2 || rows.size() < 2) continue;
            allObj.push_back(obj);
            allImg.push_back(img);
        }
    };

    // ── Intrinsic calibration per lens (fisheye model) ────────────────────────
    // The ZED-M uses a wide-angle lens (~87° HFOV) that is much better modelled
    // by OpenCV's fisheye (equidistant) model than the standard pinhole+polynomial
    // model.  D has 4 coefficients: (k1,k2,k3,k4).
    auto calibIntr = [&](
        const std::vector<std::vector<cv::Point2f>>& corners,
        const std::vector<std::vector<int>>&         ids,
        const cv::Size& sz,
        cv::Mat& K, cv::Mat& D,
        const std::string& camName) -> double
    {
        std::vector<std::vector<cv::Point3f>> allObj;
        std::vector<std::vector<cv::Point2f>> allImg;
        assemblePoints(corners, ids, 6, allObj, allImg);
        std::cout << "[" << camName << "] using " << allObj.size() << " frames\n";

        auto criteria = cv::TermCriteria(
            cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 200, 1e-7);

        // Initial K: fx ≈ width/2 assumes ~90° HFOV (ZED-M is ~87°, close enough)
        double fx0 = sz.width / 2.0;
        K = (cv::Mat_<double>(3,3) <<
             fx0, 0, sz.width  / 2.0,
             0, fx0, sz.height / 2.0,
             0,   0, 1.0);
        D = cv::Mat::zeros(4, 1, CV_64F);  // fisheye: k1,k2,k3,k4

        std::vector<cv::Mat> rvecs, tvecs;
        // CALIB_RECOMPUTE_EXTRINSIC: re-estimate pose at each iteration (more stable)
        // CALIB_FIX_SKEW           : assume zero skew (valid for all modern cameras)
        // (No CALIB_USE_INTRINSIC_GUESS — let fisheye DLT init compute K itself)
        int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC
                  | cv::fisheye::CALIB_FIX_SKEW;
        double rms = cv::fisheye::calibrate(
            allObj, allImg, sz, K, D, rvecs, tvecs, flags, criteria);
        std::cout << "[" << camName << "] intrinsic RMS: " << rms << " px\n";
        std::cout << "[" << camName << "] K:\n" << K << "\n";
        std::cout << "[" << camName << "] D (fisheye k1-k4): " << D.t() << "\n";
        return rms;
    };

    cv::Mat KL, DL, KR, DR;
    double rmsL = calibIntr(allCornersL, allIdsL, imgSizeL, KL, DL, "zed_left");
    double rmsR = calibIntr(allCornersR, allIdsR, imgSizeR, KR, DR, "zed_right");

    // Save intrinsics
    fs::create_directories(outIntrDir);
    auto saveIntr = [&](const std::string& cname, const cv::Mat& K, const cv::Mat& D,
                         const cv::Size& sz, double rms)
    {
        std::string p = outIntrDir + "/" + cname + ".yaml";
        cv::FileStorage fs(p, cv::FileStorage::WRITE);
        fs << "camera_name"  << cname;
        fs << "image_width"  << sz.width;
        fs << "image_height" << sz.height;
        fs << "K"            << K;
        fs << "D"            << D;
        fs << "rms"          << rms;
        fs.release();
        std::cout << "[SAVE] " << p << "\n";
    };
    saveIntr("zed_left",  KL, DL, imgSizeL, rmsL);
    saveIntr("zed_right", KR, DR, imgSizeR, rmsR);

    // ── Stereo calibration (fisheye) ──────────────────────────────────────────
    // Build per-frame lists of common obj/img points (corners seen by BOTH eyes).
    std::vector<std::vector<cv::Point3f>> objPtsAll;
    std::vector<std::vector<cv::Point2f>> imgPtsL_all, imgPtsR_all;

    int nCorners = static_cast<int>(board->chessboardCorners.size());
    for (size_t f = 0; f < allCornersL.size(); ++f) {
        if (allIdsL[f].empty() || allIdsR[f].empty()) continue;

        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> ordL, ordR;

        for (size_t ia = 0; ia < allIdsL[f].size(); ++ia) {
            int id = allIdsL[f][ia];
            if (id < 0 || id >= nCorners) continue;
            auto itR = std::find(allIdsR[f].begin(), allIdsR[f].end(), id);
            if (itR == allIdsR[f].end()) continue;
            size_t ib = std::distance(allIdsR[f].begin(), itR);
            obj.push_back(board->chessboardCorners[id]);
            ordL.push_back(allCornersL[f][ia]);
            ordR.push_back(allCornersR[f][ib]);
        }
        if (static_cast<int>(obj.size()) < 6) continue;
        objPtsAll.push_back(obj);
        imgPtsL_all.push_back(ordL);
        imgPtsR_all.push_back(ordR);
    }
    std::cout << "[INFO] Stereo frames with >= 6 common corners: " << objPtsAll.size() << "\n";

    if (objPtsAll.size() < 5) { std::cerr << "[ERROR] Not enough stereo pairs.\n"; return 1; }

    cv::Mat R, T;
    auto criteria = cv::TermCriteria(
        cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 200, 1e-7);

    // cv::fisheye::stereoCalibrate with fixed intrinsics (from individual calib above)
    double rms = cv::fisheye::stereoCalibrate(
        objPtsAll, imgPtsL_all, imgPtsR_all,
        KL, DL, KR, DR,
        imgSizeL, R, T,
        cv::fisheye::CALIB_FIX_INTRINSIC, criteria);

    std::cout << "  ZED stereo RMS: " << rms << " px\n";

    cv::Mat T44 = makeT44(R, T);
    printTransform("T_zed_left_right:", T44);

    fs::create_directories(outExtrDir);
    std::string extrPath = outExtrDir + "/zed_left_right.yaml";
    {
        cv::FileStorage fs(extrPath, cv::FileStorage::WRITE);
        fs << "pair"           << "zed_left_right";
        fs << "T_a_b"          << T44;
        fs << "R"              << R;
        fs << "T"              << T;
        fs << "distortion_model" << "fisheye";
        fs << "rms"            << rms;
        fs.release();
    }
    std::cout << "[SAVE] " << extrPath << "\n";
    return 0;
}
