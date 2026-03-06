#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <string>
#include <map>
#include <iostream>

// ── ChArUco board constants ────────────────────────────────────────────────
// Physical board: ChArUco-400
static constexpr int   CHARUCO_SQUARES_X   = 9;
static constexpr int   CHARUCO_SQUARES_Y   = 12;
static constexpr float CHARUCO_SQUARE_SIZE = 0.030f;   // meters (30 mm)
static constexpr float CHARUCO_MARKER_SIZE = 0.0225f;  // meters (22.5 mm)
static constexpr auto  CHARUCO_DICT_ID     = cv::aruco::DICT_5X5_250;

// ── Camera configuration ───────────────────────────────────────────────────
struct CameraConfig {
    std::string name;    // cam1 ... cam5, zed_left, zed_right
    std::string model;   // L515, D405, ZED
    std::string serial;
    std::string role;    // "reference" (cam1 only), or empty
    int         dev = -1; // V4L2 device index (ZED only; -1 = not set)
};

// ── Load camera configs from YAML ─────────────────────────────────────────
inline std::map<std::string, CameraConfig> loadCameraConfigs(const std::string& path)
{
    std::map<std::string, CameraConfig> configs;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERROR] Cannot open camera config: " << path << "\n";
        return configs;
    }
    cv::FileNode cameras = fs["cameras"];
    for (auto it = cameras.begin(); it != cameras.end(); ++it) {
        CameraConfig cfg;
        cfg.name   = (*it).name();
        cfg.model  = (std::string)(*it)["model"];
        cfg.serial = (std::string)(*it)["serial"];
        cv::FileNode roleNode = (*it)["role"];
        if (!roleNode.empty())
            cfg.role = (std::string)roleNode;
        cv::FileNode devNode = (*it)["dev"];
        if (!devNode.empty())
            cfg.dev = (int)devNode;
        configs[cfg.name] = cfg;
    }
    fs.release();
    return configs;
}

// ── Build 4×4 homogeneous transform from R (3×3) and T (3×1 or 1×3) ──────
inline cv::Mat makeT44(const cv::Mat& R, const cv::Mat& T)
{
    cv::Mat T44 = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat R64, T64;
    R.convertTo(R64, CV_64F);
    T.convertTo(T64, CV_64F);
    R64.copyTo(T44(cv::Rect(0, 0, 3, 3)));
    cv::Mat t = T64.reshape(1, 3);
    T44.at<double>(0, 3) = t.at<double>(0);
    T44.at<double>(1, 3) = t.at<double>(1);
    T44.at<double>(2, 3) = t.at<double>(2);
    return T44;
}

// ── Save a named 4×4 transform to a YAML file ─────────────────────────────
inline void saveTransformFile(const std::string& path,
                               const std::string& key,
                               const cv::Mat& T44,
                               double rms = -1.0)
{
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    fs << key << T44;
    if (rms >= 0.0)
        fs << "rms" << rms;
    fs.release();
    std::cout << "[SAVE] " << path << "\n";
}

// ── Load a named 4×4 transform from a YAML file ───────────────────────────
inline cv::Mat loadTransform(const std::string& path, const std::string& key)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERROR] Cannot open: " << path << "\n";
        return cv::Mat();
    }
    cv::Mat T;
    fs[key] >> T;
    return T;
}

// ── Print a 4×4 transform with a label ────────────────────────────────────
inline void printTransform(const std::string& label, const cv::Mat& T44)
{
    if (T44.empty()) { std::cout << label << " : <empty>\n"; return; }
    std::cout << label << "\n";
    for (int r = 0; r < 3; ++r) {
        std::cout << "  [";
        for (int c = 0; c < 4; ++c)
            std::cout << (c ? "  " : "") << T44.at<double>(r, c);
        std::cout << "]\n";
    }
    std::cout << "  t = ["
              << T44.at<double>(0,3) << ", "
              << T44.at<double>(1,3) << ", "
              << T44.at<double>(2,3) << "] m\n";
}

// ── Build ChArUco board (Ptr, compatible with OpenCV 4.x old API) ─────────
inline cv::Ptr<cv::aruco::CharucoBoard> buildCharucoBoard()
{
    auto dict = cv::aruco::getPredefinedDictionary(CHARUCO_DICT_ID);
    return cv::aruco::CharucoBoard::create(
        CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y,
        CHARUCO_SQUARE_SIZE, CHARUCO_MARKER_SIZE,
        dict);
}

// ── Detect ChArUco corners in one image (OpenCV 4.x old ArUco API) ────────
// Returns true if at least minCorners were detected.
// Optionally draws on debugImg (pass an empty Mat to skip).
inline bool detectCharuco(const cv::Mat& img,
                           const cv::Ptr<cv::aruco::CharucoBoard>& board,
                           std::vector<int>& charucoIds,
                           std::vector<cv::Point2f>& charucoCorners,
                           int minCorners = 6,
                           const cv::Mat& K = cv::Mat(),
                           const cv::Mat& D = cv::Mat())
{
    auto dict   = board->dictionary;
    auto params = cv::aruco::DetectorParameters::create();

    std::vector<int>                        markerIds;
    std::vector<std::vector<cv::Point2f>>   markerCorners, rejected;

    cv::aruco::detectMarkers(img, dict, markerCorners, markerIds, params, rejected);
    if (markerIds.empty()) return false;

    int N;
    if (!K.empty() && !D.empty())
        N = cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, img, board,
                charucoCorners, charucoIds, K, D);
    else
        N = cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, img, board,
                charucoCorners, charucoIds);

    return N >= minCorners;
}

// ── Get 3D object points for a set of ChArUco corner IDs ──────────────────
// board->chessboardCorners is indexed by charuco corner ID.
inline void getObjectPoints(const cv::Ptr<cv::aruco::CharucoBoard>& board,
                             const std::vector<int>& ids,
                             const std::vector<cv::Point2f>& corners,
                             std::vector<cv::Point3f>& objPts,
                             std::vector<cv::Point2f>& imgPts)
{
    objPts.clear();
    imgPts.clear();
    for (size_t i = 0; i < ids.size(); ++i) {
        int id = ids[i];
        if (id >= 0 && id < static_cast<int>(board->chessboardCorners.size())) {
            objPts.push_back(board->chessboardCorners[id]);
            imgPts.push_back(corners[i]);
        }
    }
}
