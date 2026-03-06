// =============================================================================
//  fuse_save_ply.cpp
//  Capture one frame set from cam1/cam2/cam4/cam5 (RealSense only),
//  unproject each depth map, transform every point into Camera 1's frame,
//  and save a single colored PLY file.
//
//  No Open3D or ZED SDK required.
//
//  Usage:
//    fuse_save_ply [--config <cameras.yaml>]
//                 [--transforms <results/extrinsics/all_to_cam1.yaml>]
//                 [--out <fused.ply>]
//                 [--min_depth <m>]  default 0.1
//                 [--max_depth <m>]  default 4.0
// =============================================================================

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <array>
#include <cmath>
#include <filesystem>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>

#include "calib_common.hpp"

// ── Colored 3-D point ─────────────────────────────────────────────────────────

struct Point3C {
    float x, y, z;
    uint8_t r, g, b;
};

// ── Per-camera context ────────────────────────────────────────────────────────

struct RsCam {
    std::string    name;
    rs2::pipeline  pipe;
    rs2::align     align_to_color{RS2_STREAM_COLOR};
    rs2_intrinsics color_intr;
    rs2_intrinsics depth_intr;
    rs2_extrinsics depth_to_color;   // for pixel mapping
    float          depth_scale = 0.f;

    // 4x4 transform to cam1 frame (row-major, loaded from YAML)
    double T[4][4];
};

static std::unique_ptr<RsCam> openCam(const std::string& name,
                                       const std::string& serial)
{
    auto c = std::make_unique<RsCam>();
    c->name = name;

    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_DEPTH, 0, 0, RS2_FORMAT_Z16,  30);
    cfg.enable_stream(RS2_STREAM_COLOR, 0, 0, RS2_FORMAT_RGB8, 30);

    auto profile = c->pipe.start(cfg);
    c->depth_scale = profile.get_device()
                            .first<rs2::depth_sensor>()
                            .get_depth_scale();
    c->color_intr  = profile.get_stream(RS2_STREAM_COLOR)
                            .as<rs2::video_stream_profile>().get_intrinsics();
    c->depth_intr  = profile.get_stream(RS2_STREAM_DEPTH)
                            .as<rs2::video_stream_profile>().get_intrinsics();
    c->depth_to_color = profile.get_stream(RS2_STREAM_DEPTH)
                               .get_extrinsics_to(profile.get_stream(RS2_STREAM_COLOR));

    // Warm-up
    for (int i = 0; i < 10; ++i) c->pipe.wait_for_frames();

    std::cout << "[RS] " << name << " (" << serial << ")"
              << "  depth=" << c->depth_intr.width << "x" << c->depth_intr.height
              << "  color=" << c->color_intr.width << "x" << c->color_intr.height
              << "  scale=" << c->depth_scale << "\n";
    return c;
}

// ── Load 4×4 OpenCV matrix from YAML into a plain double[4][4] ────────────────

static bool loadT44(const std::string& path, const std::string& key,
                    double T[4][4])
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERROR] Cannot open: " << path << "\n";
        return false;
    }
    cv::Mat m;
    fs[key] >> m;
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

// ── Unproject depth frame + color frame → colored 3-D points in cam1 frame ────

static void grabAndUnproject(RsCam& cam,
                              float minDep, float maxDep,
                              std::vector<Point3C>& out)
{
    rs2::frameset frames = cam.pipe.wait_for_frames();
    frames = cam.align_to_color.process(frames);

    auto df = frames.get_depth_frame();
    auto cf = frames.get_color_frame();

    int W = df.get_width();
    int H = df.get_height();
    const uint16_t* raw = reinterpret_cast<const uint16_t*>(df.get_data());
    const uint8_t*  rgb = reinterpret_cast<const uint8_t*> (cf.get_data());
    int cW = cf.get_width();

    // After align_to_color the depth and color share the same pixel grid.
    // color_intr is used for deprojection of the aligned depth.
    for (int v = 0; v < H; ++v) {
        for (int u = 0; u < W; ++u) {
            uint16_t d = raw[v * W + u];
            if (d == 0) continue;
            float dm = d * cam.depth_scale;
            if (dm < minDep || dm > maxDep) continue;

            float pixel[2] = {(float)u, (float)v};
            float pt[3];
            rs2_deproject_pixel_to_point(pt, &cam.color_intr, pixel, dm);

            // Transform to cam1 frame
            const double* T = &cam.T[0][0];
            float xw = (float)(T[0]*pt[0] + T[1]*pt[1] + T[2]*pt[2]  + T[3]);
            float yw = (float)(T[4]*pt[0] + T[5]*pt[1] + T[6]*pt[2]  + T[7]);
            float zw = (float)(T[8]*pt[0] + T[9]*pt[1] + T[10]*pt[2] + T[11]);

            // Color from aligned color frame (RGB8)
            int ci = (v * cW + u) * 3;
            out.push_back({xw, yw, zw,
                           rgb[ci], rgb[ci+1], rgb[ci+2]});
        }
    }
}

// ── Write ASCII PLY ───────────────────────────────────────────────────────────

static void savePLY(const std::string& path, const std::vector<Point3C>& pts)
{
    std::ofstream f(path);
    if (!f) { std::cerr << "[ERROR] Cannot write: " << path << "\n"; return; }

    f << "ply\n"
      << "format ascii 1.0\n"
      << "element vertex " << pts.size() << "\n"
      << "property float x\n"
      << "property float y\n"
      << "property float z\n"
      << "property uchar red\n"
      << "property uchar green\n"
      << "property uchar blue\n"
      << "end_header\n";

    for (const auto& p : pts)
        f << p.x << " " << p.y << " " << p.z
          << " " << (int)p.r << " " << (int)p.g << " " << (int)p.b << "\n";

    std::cout << "[SAVE] " << path
              << "  (" << pts.size() << " points)\n";
}

// ── CLI ───────────────────────────────────────────────────────────────────────

static void printHelp(const char* prog)
{
    std::cout
        << "Usage: " << prog
        << " [--config <cameras.yaml>] [--transforms <all_to_cam1.yaml>]\n"
        << "       [--out <fused.ply>] [--min_depth <m>] [--max_depth <m>]\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    std::string configPath     = "configs/cameras.yaml";
    std::string transformsPath = "results/extrinsics/all_to_cam1.yaml";
    std::string outPath        = "results/fused.ply";
    float minDep = 0.1f;
    float maxDep = 4.0f;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") { printHelp(argv[0]); return 0; }
        else if (a == "--config"     && i+1 < argc) configPath     = argv[++i];
        else if (a == "--transforms" && i+1 < argc) transformsPath = argv[++i];
        else if (a == "--out"        && i+1 < argc) outPath        = argv[++i];
        else if (a == "--min_depth"  && i+1 < argc) minDep         = std::stof(argv[++i]);
        else if (a == "--max_depth"  && i+1 < argc) maxDep         = std::stof(argv[++i]);
    }

    auto configs = loadCameraConfigs(configPath);

    // Load transforms
    double T_1_1[4][4], T_1_2[4][4], T_1_4[4][4], T_1_5[4][4];
    if (!loadT44(transformsPath, "T_1_1", T_1_1) ||
        !loadT44(transformsPath, "T_1_2", T_1_2) ||
        !loadT44(transformsPath, "T_1_4", T_1_4) ||
        !loadT44(transformsPath, "T_1_5", T_1_5)) {
        return 1;
    }

    // Open cameras
    auto cam1 = openCam("cam1", configs.at("cam1").serial);
    auto cam2 = openCam("cam2", configs.at("cam2").serial);
    auto cam4 = openCam("cam4", configs.at("cam4").serial);
    auto cam5 = openCam("cam5", configs.at("cam5").serial);

    // Assign transforms
    memcpy(cam1->T, T_1_1, sizeof(T_1_1));
    memcpy(cam2->T, T_1_2, sizeof(T_1_2));
    memcpy(cam4->T, T_1_4, sizeof(T_1_4));
    memcpy(cam5->T, T_1_5, sizeof(T_1_5));

    // Capture and unproject
    std::vector<Point3C> cloud;
    cloud.reserve(4 * 1280 * 720);

    size_t before;
    before = cloud.size(); grabAndUnproject(*cam1, minDep, maxDep, cloud);
    std::cout << "  cam1: " << (cloud.size() - before) << " pts\n";
    before = cloud.size(); grabAndUnproject(*cam2, minDep, maxDep, cloud);
    std::cout << "  cam2: " << (cloud.size() - before) << " pts\n";
    before = cloud.size(); grabAndUnproject(*cam4, minDep, maxDep, cloud);
    std::cout << "  cam4: " << (cloud.size() - before) << " pts\n";
    before = cloud.size(); grabAndUnproject(*cam5, minDep, maxDep, cloud);
    std::cout << "  cam5: " << (cloud.size() - before) << " pts\n";

    cam1->pipe.stop();
    cam2->pipe.stop();
    cam4->pipe.stop();
    cam5->pipe.stop();

    std::cout << "[INFO] Total: " << cloud.size() << " points\n";

    // Ensure output directory exists
    {
        std::string dir = outPath.substr(0, outPath.rfind('/'));
        if (!dir.empty())
            std::filesystem::create_directories(dir);
    }

    savePLY(outPath, cloud);
    return 0;
}
