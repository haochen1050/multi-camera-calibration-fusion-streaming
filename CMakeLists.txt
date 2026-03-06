cmake_minimum_required(VERSION 3.16)
project(multi_cam_calib CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Core dependencies ─────────────────────────────────────────────────────────
find_package(OpenCV     REQUIRED)
find_package(realsense2 REQUIRED)

# ── Optional: Open3D (only needed for fuse_and_view) ─────────────────────────
# ZED is accessed via V4L2/OpenCV for all calibration tools.
# Only fuse_and_view needs the ZED SDK (for depth).
# Build Open3D from source with -DBUILD_CUDA_MODULE=OFF for CPU-only use.
find_package(Open3D QUIET)
if(Open3D_FOUND)
    message(STATUS "Open3D found: ${Open3D_VERSION}")
else()
    message(WARNING "Open3D NOT found — fuse_and_view will be skipped.")
endif()

find_package(ZED 4 QUIET)
if(ZED_FOUND)
    message(STATUS "ZED SDK found: ${ZED_VERSION}")
else()
    message(STATUS "ZED SDK not found — fuse_and_view will be skipped.")
    message(STATUS "  Calibration tools use V4L2/OpenCV for ZED (no SDK needed).")
endif()

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

# ── Targets that need only OpenCV ─────────────────────────────────────────────

add_executable(calibrate_intrinsics src/calibrate_intrinsics.cpp)
target_link_libraries(calibrate_intrinsics PRIVATE ${OpenCV_LIBS})

add_executable(calibrate_extrinsics_pair src/calibrate_extrinsics_pair.cpp)
target_link_libraries(calibrate_extrinsics_pair PRIVATE ${OpenCV_LIBS})

add_executable(chain_transforms src/chain_transforms.cpp)
target_link_libraries(chain_transforms PRIVATE ${OpenCV_LIBS})

# calibrate_zed_stereo accesses ZED via V4L2 — no ZED SDK required
add_executable(calibrate_zed_stereo src/calibrate_zed_stereo.cpp)
target_link_libraries(calibrate_zed_stereo PRIVATE ${OpenCV_LIBS})

# ── Targets that need RealSense + OpenCV ──────────────────────────────────────

add_executable(dump_rs_intrinsics src/dump_rs_intrinsics.cpp)
target_link_libraries(dump_rs_intrinsics PRIVATE realsense2::realsense2 ${OpenCV_LIBS})

# capture_pair uses V4L2/OpenCV for ZED — no ZED SDK required
add_executable(capture_pair src/capture_pair.cpp)
target_link_libraries(capture_pair PRIVATE realsense2::realsense2 ${OpenCV_LIBS})

# ── fuse_save_ply: RealSense-only PLY writer (no ZED SDK, no Open3D) ─────────

add_executable(fuse_save_ply src/fuse_save_ply.cpp)
target_link_libraries(fuse_save_ply PRIVATE realsense2::realsense2 ${OpenCV_LIBS})

# ── fuse_stream: real-time streaming viewer (RealSense + Open3D, no ZED SDK) ──

if(Open3D_FOUND)
    add_executable(fuse_stream src/fuse_stream.cpp)
    target_link_libraries(fuse_stream PRIVATE
        realsense2::realsense2 ${OpenCV_LIBS} Open3D::Open3D)
    target_include_directories(fuse_stream PRIVATE ${Open3D_INCLUDE_DIRS})
else()
    message(STATUS "  Skipping fuse_stream (needs Open3D).")
endif()

# ── fuse_and_view needs ZED SDK + Open3D (optional) ──────────────────────────

if(ZED_FOUND AND Open3D_FOUND)
    include_directories(${ZED_INCLUDE_DIRS})
    add_executable(fuse_and_view src/fuse_and_view.cpp)
    target_link_libraries(fuse_and_view PRIVATE
        realsense2::realsense2 ${ZED_LIBRARIES} ${OpenCV_LIBS} Open3D::Open3D)
else()
    message(STATUS "  Skipping fuse_and_view (needs ZED SDK + Open3D).")
endif()
