// =============================================================================
//  fuse_and_view.cpp
//  Open all 5 cameras, capture one synchronized frame set, unproject to 3D,
//  transform every point cloud into Camera 1's coordinate frame, and display
//  them fused in a single Open3D viewer window.
//
//  Each camera's cloud is given a distinct color:
//    cam1 = red  cam2 = green  cam3 = blue  cam4 = yellow  cam5 = magenta
//  A coordinate frame axis is drawn at the Camera 1 origin.
//
//  Usage:
//    fuse_and_view [--config <cameras.yaml>]
//                 [--transforms <results/extrinsics/all_to_cam1.yaml>]
// =============================================================================

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <array>
#include <cmath>

#include <librealsense2/rs.hpp>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

#include <open3d/Open3D.h>

#include "calib_common.hpp"

// ── RealSense session ─────────────────────────────────────────────────────────

struct RsSession {
    rs2::pipeline  pipe;
    rs2::align     aligner{RS2_STREAM_COLOR};
    rs2_intrinsics depth_intr;
    float          depth_scale = 0.f;
    std::string    name;
};

static std::unique_ptr<RsSession> openRs(const std::string& camName,
                                          const std::string& serial)
{
    auto s = std::make_unique<RsSession>();
    s->name = camName;

    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_DEPTH, 0, 0, RS2_FORMAT_Z16,  30);
    cfg.enable_stream(RS2_STREAM_COLOR, 0, 0, RS2_FORMAT_BGR8, 30);

    auto profile = s->pipe.start(cfg);
    s->depth_scale  = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
    s->depth_intr   = profile.get_stream(RS2_STREAM_DEPTH)
                             .as<rs2::video_stream_profile>().get_intrinsics();

    for (int i = 0; i < 10; ++i) s->pipe.wait_for_frames();   // warm up
    std::cout << "[RS] opened " << camName << " (" << serial << ")"
              << "  depth=" << s->depth_intr.width << "x" << s->depth_intr.height
              << "  scale=" << s->depth_scale << "\n";
    return s;
}

// ── Unproject RealSense depth → colored Open3D cloud (in cam1 frame) ─────────

static std::shared_ptr<open3d::geometry::PointCloud>
rsToCloud(RsSession& cam,
          const Eigen::Matrix4d& T_to_cam1,
          const std::array<double,3>& color)
{
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();

    rs2::frameset frames = cam.pipe.wait_for_frames();
    frames = cam.aligner.process(frames);

    auto df = frames.get_depth_frame();
    int W = df.get_width(), H = df.get_height();
    const uint16_t* raw = reinterpret_cast<const uint16_t*>(df.get_data());

    pcd->points_.reserve(static_cast<size_t>(W * H));
    pcd->colors_.reserve(static_cast<size_t>(W * H));

    for (int v = 0; v < H; ++v) {
        for (int u = 0; u < W; ++u) {
            uint16_t d = raw[v * W + u];
            if (d == 0) continue;
            float dm = d * cam.depth_scale;
            if (dm < 0.1f || dm > 6.0f) continue;

            float pixel[2] = {(float)u, (float)v};
            float pt[3];
            rs2_deproject_pixel_to_point(pt, &cam.depth_intr, pixel, dm);

            Eigen::Vector4d p(pt[0], pt[1], pt[2], 1.0);
            Eigen::Vector4d pw = T_to_cam1 * p;
            pcd->points_.emplace_back(pw[0], pw[1], pw[2]);
            pcd->colors_.emplace_back(color[0], color[1], color[2]);
        }
    }
    return pcd;
}

// ── ZED depth → colored Open3D cloud (in cam1 frame) ─────────────────────────

static std::shared_ptr<open3d::geometry::PointCloud>
zedToCloud(sl::Camera& zed,
           const Eigen::Matrix4d& T_to_cam1,
           const std::array<double,3>& color)
{
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();

    zed.grab();
    sl::Mat zed_pc;
    zed.retrieveMeasure(zed_pc, sl::MEASURE::XYZ, sl::MEM::CPU);

    int W = static_cast<int>(zed_pc.getWidth());
    int H = static_cast<int>(zed_pc.getHeight());

    pcd->points_.reserve(static_cast<size_t>(W * H));
    pcd->colors_.reserve(static_cast<size_t>(W * H));

    for (int v = 0; v < H; ++v) {
        for (int u = 0; u < W; ++u) {
            sl::float4 pt;
            zed_pc.getValue(u, v, &pt);
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;
            Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4d pw = T_to_cam1 * p;
            pcd->points_.emplace_back(pw[0], pw[1], pw[2]);
            pcd->colors_.emplace_back(color[0], color[1], color[2]);
        }
    }
    return pcd;
}

// ── Load 4×4 transform from YAML → Eigen ─────────────────────────────────────

static Eigen::Matrix4d loadEigenT(const std::string& yamlPath, const std::string& key)
{
    cv::FileStorage fs(yamlPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERROR] Cannot open: " << yamlPath << "\n";
        return Eigen::Matrix4d::Identity();
    }
    cv::Mat m;
    fs[key] >> m;
    if (m.empty()) {
        std::cerr << "[WARN] Key '" << key << "' not found in " << yamlPath << "\n";
        return Eigen::Matrix4d::Identity();
    }
    m.convertTo(m, CV_64F);
    Eigen::Matrix4d e = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            e(r, c) = m.at<double>(r, c);
    return e;
}

// ── CLI ───────────────────────────────────────────────────────────────────────

static void printHelp(const char* prog)
{
    std::cout
        << "Usage: " << prog
        << " [--config <cameras.yaml>] [--transforms <all_to_cam1.yaml>]\n"
        << "\n"
        << "  --config      Serial numbers  (default: configs/cameras.yaml)\n"
        << "  --transforms  Extrinsic chain (default: results/extrinsics/all_to_cam1.yaml)\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    std::string configPath     = "configs/cameras.yaml";
    std::string transformsPath = "results/extrinsics/all_to_cam1.yaml";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") { printHelp(argv[0]); return 0; }
        else if (a == "--config"     && i+1 < argc) configPath     = argv[++i];
        else if (a == "--transforms" && i+1 < argc) transformsPath = argv[++i];
    }

    auto configs = loadCameraConfigs(configPath);

    // Load extrinsic chain
    Eigen::Matrix4d T_1_1 = loadEigenT(transformsPath, "T_1_1");
    Eigen::Matrix4d T_1_2 = loadEigenT(transformsPath, "T_1_2");
    Eigen::Matrix4d T_1_3 = loadEigenT(transformsPath, "T_1_3");
    Eigen::Matrix4d T_1_4 = loadEigenT(transformsPath, "T_1_4");
    Eigen::Matrix4d T_1_5 = loadEigenT(transformsPath, "T_1_5");

    // Open RealSense cameras
    auto rs1 = openRs("cam1", configs.at("cam1").serial);
    auto rs2 = openRs("cam2", configs.at("cam2").serial);
    auto rs4 = openRs("cam4", configs.at("cam4").serial);
    auto rs5 = openRs("cam5", configs.at("cam5").serial);

    // Open ZED (cam3) for depth
    sl::Camera zed;
    {
        sl::InitParameters init;
        init.camera_resolution = sl::RESOLUTION::HD720;
        init.camera_fps        = 30;
        init.depth_mode        = sl::DEPTH_MODE::PERFORMANCE;
        init.sdk_gpu_id        = -1;   // CPU-only
        init.sdk_verbose       = false;

        auto err = zed.open(init);
        if (err != sl::ERROR_CODE::SUCCESS) {
            std::cerr << "[ZED] open failed: " << sl::toString(err) << "\n";
            return 1;
        }
        for (int i = 0; i < 10; ++i) zed.grab();
        std::cout << "[ZED] cam3 ready\n";
    }

    // Per-camera RGB colors (0..1)
    const std::array<double,3> c1 = {1.0, 0.2, 0.2};  // red
    const std::array<double,3> c2 = {0.2, 1.0, 0.2};  // green
    const std::array<double,3> c3 = {0.2, 0.4, 1.0};  // blue
    const std::array<double,3> c4 = {1.0, 1.0, 0.2};  // yellow
    const std::array<double,3> c5 = {1.0, 0.2, 1.0};  // magenta

    std::cout << "[INFO] Grabbing synchronized frames from all 5 cameras...\n";

    auto pcd1 = rsToCloud(*rs1, T_1_1, c1);
    auto pcd2 = rsToCloud(*rs2, T_1_2, c2);
    auto pcd3 = zedToCloud(zed, T_1_3, c3);
    auto pcd4 = rsToCloud(*rs4, T_1_4, c4);
    auto pcd5 = rsToCloud(*rs5, T_1_5, c5);

    std::cout << "  cam1: " << pcd1->points_.size() << " pts\n";
    std::cout << "  cam2: " << pcd2->points_.size() << " pts\n";
    std::cout << "  cam3: " << pcd3->points_.size() << " pts\n";
    std::cout << "  cam4: " << pcd4->points_.size() << " pts\n";
    std::cout << "  cam5: " << pcd5->points_.size() << " pts\n";

    // Stop cameras
    rs1->pipe.stop();
    rs2->pipe.stop();
    rs4->pipe.stop();
    rs5->pipe.stop();
    zed.close();

    // Merge clouds
    auto merged = std::make_shared<open3d::geometry::PointCloud>();
    *merged += *pcd1;
    *merged += *pcd2;
    *merged += *pcd3;
    *merged += *pcd4;
    *merged += *pcd5;
    std::cout << "[INFO] Total merged points: " << merged->points_.size() << "\n";

    // Coordinate frame at Camera 1 origin (0.1 m axis length)
    auto frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);

    std::cout << "[INFO] Launching Open3D viewer (close window to exit)...\n";
    open3d::visualization::DrawGeometries(
        {merged, frame},
        "Fused 5-Camera Point Cloud — Cam1 Frame",
        1280, 720);

    return 0;
}
