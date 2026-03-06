// =============================================================================
//  fuse_stream.cpp
//  Real-time multi-camera point cloud streaming into Camera 1's frame.
//  Opens selected RealSense cameras, projects every depth frame into
//  cam1 coordinates and renders the merged cloud live with Open3D.
//
//  No ZED SDK required.
//
//  Usage:
//    fuse_stream [--config     <cameras.yaml>]
//               [--transforms  <results/extrinsics/all_to_cam1.yaml>]
//               [--cameras     cam1,cam2,cam4,cam5]   default: all
//               [--min_depth   <m>]  default 0.1
//               [--max_depth   <m>]  default 4.0
//               [--stride      <px>] default 4  (skip pixels; higher = faster)
//               [--voxel       <m>]  default 0  (0 = no voxel downsampling)
//
//  Examples:
//    fuse_stream --cameras cam1           # only cam1
//    fuse_stream --cameras cam1,cam2      # cam1 + cam2 fused
//    fuse_stream --cameras cam1,cam2,cam4 # three cameras
//    fuse_stream --stride 8               # very lightweight preview
//
//  Per-camera colors:
//    cam1 = red   cam2 = green   cam4 = yellow   cam5 = magenta
// =============================================================================

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <array>
#include <cstring>
#include <filesystem>
#include <set>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>

#include "calib_common.hpp"

namespace o3d = open3d;
namespace fs  = std::filesystem;

// ── Per-camera state ──────────────────────────────────────────────────────────

struct RsCam {
    std::string    name;
    rs2::pipeline  pipe;
    rs2::align     align_to_color{RS2_STREAM_COLOR};
    rs2_intrinsics color_intr{};
    float          depth_scale = 0.f;
    double         T[4][4]{};             // transform to cam1 frame
    Eigen::Vector3d color;                // per-camera display color
};

static std::unique_ptr<RsCam> openCam(const std::string& name,
                                       const std::string& serial,
                                       const Eigen::Vector3d& displayColor)
{
    auto c = std::make_unique<RsCam>();
    c->name  = name;
    c->color = displayColor;

    try {
        rs2::config cfg;
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_DEPTH, 0, 0, RS2_FORMAT_Z16,  30);
        cfg.enable_stream(RS2_STREAM_COLOR, 0, 0, RS2_FORMAT_RGB8, 30);

        auto profile      = c->pipe.start(cfg);
        c->depth_scale    = profile.get_device()
                                   .first<rs2::depth_sensor>()
                                   .get_depth_scale();
        c->color_intr     = profile.get_stream(RS2_STREAM_COLOR)
                                   .as<rs2::video_stream_profile>().get_intrinsics();

        // Warm-up: discard early frames
        for (int i = 0; i < 10; ++i) c->pipe.wait_for_frames();

        std::cout << " ok  color=" << c->color_intr.width << "x" << c->color_intr.height
                  << "  scale=" << c->depth_scale << "\n";
    } catch (const std::exception& e) {
        std::cerr << "[SKIP] " << name << " (" << serial << "): " << e.what() << "\n";
        return nullptr;
    }
    return c;
}

// ── Load 4×4 OpenCV matrix from YAML ─────────────────────────────────────────

static bool loadT44(const std::string& path, const std::string& key,
                    double T[4][4])
{
    cv::FileStorage fss(path, cv::FileStorage::READ);
    if (!fss.isOpened()) {
        std::cerr << "[ERROR] Cannot open: " << path << "\n";
        return false;
    }
    cv::Mat m;
    fss[key] >> m;
    if (m.empty()) {
        std::cerr << "[ERROR] Key '" << key << "' not found in " << path << "\n";
        return false;
    }
    m.convertTo(m, CV_64F);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T[r][c] = m.at<double>(r, c);
    return true;
}

// ── Unproject one camera's aligned depth+color → append to cloud ──────────────

static void appendCam(RsCam& cam, float minDep, float maxDep, int stride,
                      std::vector<Eigen::Vector3d>& pts,
                      std::vector<Eigen::Vector3d>& cols)
{
    rs2::frameset frames;
    try {
        frames = cam.pipe.wait_for_frames(200);   // 200ms timeout
    } catch (...) {
        std::cerr << "[WARN] " << cam.name << " frame timeout\n";
        return;
    }
    frames = cam.align_to_color.process(frames);

    auto df = frames.get_depth_frame();
    auto cf = frames.get_color_frame();
    if (!df || !cf) return;

    int W = df.get_width();
    int H = df.get_height();
    const uint16_t* raw = reinterpret_cast<const uint16_t*>(df.get_data());
    const uint8_t*  rgb = reinterpret_cast<const uint8_t*> (cf.get_data());

    const double* T = &cam.T[0][0];

    for (int v = 0; v < H; v += stride) {
        for (int u = 0; u < W; u += stride) {
            uint16_t d = raw[v * W + u];
            if (d == 0) continue;
            float dm = d * cam.depth_scale;
            if (dm < minDep || dm > maxDep) continue;

            float pixel[2] = {(float)u, (float)v};
            float pt[3];
            rs2_deproject_pixel_to_point(pt, &cam.color_intr, pixel, dm);

            // Transform into cam1 frame
            double xw = T[0]*pt[0] + T[1]*pt[1] + T[2]*pt[2]  + T[3];
            double yw = T[4]*pt[0] + T[5]*pt[1] + T[6]*pt[2]  + T[7];
            double zw = T[8]*pt[0] + T[9]*pt[1] + T[10]*pt[2] + T[11];

            pts.emplace_back(xw, yw, zw);

            // Use actual RGB color from the aligned color frame
            int ci = (v * W + u) * 3;
            cols.emplace_back(rgb[ci] / 255.0, rgb[ci+1] / 255.0, rgb[ci+2] / 255.0);
        }
    }
}

// ── CLI ───────────────────────────────────────────────────────────────────────

static void printHelp(const char* prog)
{
    std::cout
        << "Usage: " << prog
        << " [--cameras cam1,cam2,cam4,cam5]\n"
        << "       [--config <cameras.yaml>] [--transforms <all_to_cam1.yaml>]\n"
        << "       [--min_depth <m>] [--max_depth <m>]\n"
        << "       [--stride <px>]   (pixel step, default 4 — higher = faster)\n"
        << "       [--voxel <m>]     (voxel size for downsampling, default 0 = off)\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    std::string configPath     = "configs/cameras.yaml";
    std::string transformsPath = "results/extrinsics/all_to_cam1.yaml";
    std::string camerasArg     = "cam1,cam2,cam4,cam5";   // default: all
    float minDep  = 0.1f;
    float maxDep  = 4.0f;
    int   stride  = 4;      // default 4 — lighter workload
    double voxel  = 0.0;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") { printHelp(argv[0]); return 0; }
        else if (a == "--config"     && i+1 < argc) configPath     = argv[++i];
        else if (a == "--transforms" && i+1 < argc) transformsPath = argv[++i];
        else if (a == "--cameras"    && i+1 < argc) camerasArg     = argv[++i];
        else if (a == "--min_depth"  && i+1 < argc) minDep         = std::stof(argv[++i]);
        else if (a == "--max_depth"  && i+1 < argc) maxDep         = std::stof(argv[++i]);
        else if (a == "--stride"     && i+1 < argc) stride         = std::stoi(argv[++i]);
        else if (a == "--voxel"      && i+1 < argc) voxel          = std::stod(argv[++i]);
    }

    // Parse comma-separated camera list
    std::set<std::string> wantCams;
    {
        std::istringstream ss(camerasArg);
        std::string tok;
        while (std::getline(ss, tok, ','))
            if (!tok.empty()) wantCams.insert(tok);
    }

    // Load camera configs
    auto configs = loadCameraConfigs(configPath);
    if (configs.empty()) {
        std::cerr << "[ERROR] No camera configs loaded from " << configPath << "\n";
        return 1;
    }

    // Load transforms
    double T_1_1[4][4], T_1_2[4][4], T_1_4[4][4], T_1_5[4][4];
    if (!loadT44(transformsPath, "T_1_1", T_1_1) ||
        !loadT44(transformsPath, "T_1_2", T_1_2) ||
        !loadT44(transformsPath, "T_1_4", T_1_4) ||
        !loadT44(transformsPath, "T_1_5", T_1_5)) {
        return 1;
    }

    // Per-camera display colors (RGB 0-1)
    const Eigen::Vector3d kRed     = {1.0, 0.2, 0.2};
    const Eigen::Vector3d kGreen   = {0.2, 1.0, 0.2};
    const Eigen::Vector3d kYellow  = {1.0, 1.0, 0.2};
    const Eigen::Vector3d kMagenta = {1.0, 0.2, 1.0};

    // Enumerate connected RealSense devices once (avoids interfering contexts later)
    std::set<std::string> connected;
    {
        rs2::context ctx;
        for (auto&& dev : ctx.query_devices())
            connected.insert(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    }
    std::cout << "[INFO] Connected serials:";
    for (auto& s : connected) std::cout << " " << s;
    std::cout << "\n";

    // Try opening a camera with a 6-second timeout.
    // Uses a detached thread so pipe.start() hanging doesn't block the caller.
    auto tryOpen = [&](const std::string& name, const std::string& serial,
                       const Eigen::Vector3d& clr) -> std::unique_ptr<RsCam> {
        if (!connected.count(serial)) {
            std::cerr << "[SKIP] " << name << " (" << serial << ") not found.\n";
            return nullptr;
        }
        std::cout << "[INFO] Opening " << name << "..." << std::flush;

        // Shared state between caller and detached thread
        struct Slot {
            std::unique_ptr<RsCam> cam;
            bool done = false;
            std::mutex mtx;
            std::condition_variable cv;
        };
        auto slot = std::make_shared<Slot>();

        std::thread([slot, name, serial, clr]() mutable {
            auto c = openCam(name, serial, clr);
            std::lock_guard<std::mutex> lk(slot->mtx);
            slot->cam  = std::move(c);
            slot->done = true;
            slot->cv.notify_one();
        }).detach();   // detach immediately — won't block on destructor

        std::unique_lock<std::mutex> lk(slot->mtx);
        bool ok = slot->cv.wait_for(lk, std::chrono::seconds(6),
                                    [&]{ return slot->done; });
        if (!ok) {
            std::cerr << "\n[SKIP] " << name << " timed out (USB bandwidth?). Continuing.\n";
            return nullptr;   // slot kept alive by detached thread's shared_ptr copy
        }
        return std::move(slot->cam);
    };

    // Open only the cameras in wantCams
    auto maybeOpen = [&](const std::string& name, const Eigen::Vector3d& clr,
                         double T[4][4]) -> std::unique_ptr<RsCam> {
        if (!wantCams.count(name)) return nullptr;
        auto cam = tryOpen(name, configs.at(name).serial, clr);
        if (cam) memcpy(cam->T, T, sizeof(double[4][4]));
        return cam;
    };

    std::cout << "[INFO] Opening cameras: " << camerasArg << "\n";
    auto cam1 = maybeOpen("cam1", kRed,     T_1_1);
    auto cam2 = maybeOpen("cam2", kGreen,   T_1_2);
    auto cam4 = maybeOpen("cam4", kYellow,  T_1_4);
    auto cam5 = maybeOpen("cam5", kMagenta, T_1_5);

    int activeCams = (cam1 ? 1 : 0) + (cam2 ? 1 : 0) + (cam4 ? 1 : 0) + (cam5 ? 1 : 0);
    if (activeCams == 0) {
        std::cerr << "[ERROR] No cameras available.\n";
        return 1;
    }
    std::cout << "[INFO] " << activeCams << " camera(s) active.\n";

    std::cout << "[INFO] stride=" << stride
              << "  depth=[" << minDep << ", " << maxDep << "]m"
              << "  voxel=" << (voxel > 0 ? std::to_string(voxel) + "m" : "off")
              << "\n";

    // ── Open3D visualizer setup ───────────────────────────────────────────────

    auto cloud = std::make_shared<o3d::geometry::PointCloud>();
    auto frame = o3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);

    o3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("Point Cloud Stream — " + camerasArg, 1280, 720);
    vis.AddGeometry(cloud);
    vis.AddGeometry(frame);

    // Set a reasonable initial viewpoint (look from +Z toward origin)
    auto& vc = vis.GetViewControl();
    vc.SetZoom(0.6);

    std::cout << "[INFO] Streaming — close window or press Q to stop.\n\n";

    int frameIdx = 0;
    bool viewReset = false;
    while (vis.PollEvents()) {
        std::vector<Eigen::Vector3d> pts, cols;
        pts.reserve(4 * (640 / stride) * (480 / stride));
        cols.reserve(pts.capacity());

        if (cam1) appendCam(*cam1, minDep, maxDep, stride, pts, cols);
        if (cam2) appendCam(*cam2, minDep, maxDep, stride, pts, cols);
        if (cam4) appendCam(*cam4, minDep, maxDep, stride, pts, cols);
        if (cam5) appendCam(*cam5, minDep, maxDep, stride, pts, cols);

        cloud->points_ = std::move(pts);
        cloud->colors_ = std::move(cols);

        if (voxel > 0.0) {
            auto down = cloud->VoxelDownSample(voxel);
            cloud->points_ = down->points_;
            cloud->colors_ = down->colors_;
        }

        vis.UpdateGeometry(cloud);

        // Reset viewpoint once after the first real frame so the camera
        // fits to the actual point cloud (not the initial empty bounding box)
        if (!viewReset && !cloud->points_.empty()) {
            vis.ResetViewPoint(true);
            viewReset = true;
        }

        vis.UpdateRender();

        if (++frameIdx % 30 == 0)
            std::cout << "\r  pts=" << cloud->points_.size()
                      << "  frame=" << frameIdx << std::flush;
    }

    std::cout << "\n[INFO] Window closed — stopping cameras.\n";
    vis.DestroyVisualizerWindow();

    if (cam1) cam1->pipe.stop();
    if (cam2) cam2->pipe.stop();
    if (cam4) cam4->pipe.stop();
    if (cam5) cam5->pipe.stop();

    return 0;
}
