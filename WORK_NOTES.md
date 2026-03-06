# Multi-Camera Calibration & Point Cloud Fusion — Work Notes

## Overview

This project calibrates a 5-camera RGBD array and fuses their depth streams into a
single point cloud in Camera 1's coordinate frame. The array consists of:

| Camera | Model          | Role      | Serial         |
|--------|----------------|-----------|----------------|
| cam1   | Intel L515     | Reference | f0210390       |
| cam2   | Intel L515     | —         | f0171709       |
| cam3   | ZED-M (left)   | —         | N/A (V4L2)     |
| cam4   | Intel D405     | —         | 130322271201   |
| cam5   | Intel D405     | —         | 218622270381   |

Camera 1 is the reference frame. All other cameras are expressed relative to it.

---

## System Environment

- **OS:** Ubuntu, Wayland display (DISPLAY=:0, XWayland running)
- **GPU:** AMD (no CUDA, no ROCm)
- **OpenCV:** 4.6.0 — uses the **old ArUco API** (`cv::Ptr<CharucoBoard>`, `detectMarkers` +
  `interpolateCornersCharuco`). No `CharucoDetector`, no new `CharucoBoard` constructor.
- **librealsense2:** installed at `/usr/local/lib/cmake/realsense2/`
- **Open3D:** 0.19.0, installed at `/usr/local/lib/cmake/Open3D/`
- **ZED SDK:** NOT installed. ZED camera is accessed via V4L2/OpenCV VideoCapture on
  `/dev/video12` (set as `dev: 12` in `configs/cameras.yaml`).

---

## Project Layout

```
multi_cam_calib/
├── CMakeLists.txt
├── configs/
│   └── cameras.yaml                    # camera models, serials, V4L2 device index
├── include/
│   └── calib_common.hpp                # shared helpers: board, ArUco detect, transforms
├── src/
│   ├── capture_pair.cpp                # live synchronized image capture
│   ├── calibrate_intrinsics.cpp        # intrinsics from ChArUco images (ZED)
│   ├── dump_rs_intrinsics.cpp          # read RealSense SDK intrinsics → YAML
│   ├── calibrate_extrinsics_pair.cpp   # pairwise extrinsics via stereoCalibrate
│   ├── calibrate_zed_stereo.cpp        # ZED left↔right stereo calibration
│   ├── chain_transforms.cpp            # chain T_1_2..T_4_5 → all_to_cam1.yaml
│   ├── fuse_save_ply.cpp               # one-shot depth capture → colored PLY file
│   └── fuse_stream.cpp                 # real-time streaming point cloud viewer
├── data/
│   ├── pair_1_2/cam1/  pair_1_2/cam2/  # captured image pairs
│   ├── pair_2_3/cam2/  pair_2_3/cam3/
│   ├── pair_3_4/cam3/  pair_3_4/cam4/
│   ├── pair_4_5/cam4/  pair_4_5/cam5/
│   └── zed_stereo/zed_left/ zed_stereo/zed_right/
├── results/
│   ├── intrinsics/
│   │   ├── cam1.yaml   cam2.yaml   cam3.yaml   cam4.yaml   cam5.yaml
│   └── extrinsics/
│       ├── cam1_cam2.yaml   cam2_cam3.yaml   cam3_cam4.yaml   cam4_cam5.yaml
│       ├── zed_left_right.yaml
│       └── all_to_cam1.yaml
└── build/                              # all compiled binaries
```

---

## Calibration Board

All calibration tools use a single ChArUco board (physical board "ChArUco-400"):

| Parameter      | Value                  |
|----------------|------------------------|
| Grid size      | 9 × 12 squares         |
| Square size    | 30 mm                  |
| Marker size    | 22.5 mm                |
| Dictionary     | DICT_5X5_250           |
| Inner corners  | 88 (IDs 0 – 87)        |

Defined as constants in `include/calib_common.hpp`.

---

## Step 1 — Camera Configuration

`configs/cameras.yaml`:
```yaml
cameras:
  cam1: { model: "L515", serial: "f0210390",    role: "reference" }
  cam2: { model: "L515", serial: "f0171709" }
  cam3: { model: "ZED",  serial: "N/A", dev: 12 }
  cam4: { model: "D405", serial: "130322271201" }
  cam5: { model: "D405", serial: "218622270381" }
```

The `dev: 12` field tells `capture_pair` which `/dev/videoN` device the ZED uses.
ZED outputs a 2560×720 side-by-side frame; the tools split it at the midpoint.

---

## Step 2 — Image Capture (`capture_pair`)

Captures synchronized image pairs for each adjacent camera pair.

```bash
# From multi_cam_calib/ directory
./build/capture_pair --pair cam1 cam2
./build/capture_pair --pair cam2 cam3   # cam3 = ZED left via V4L2
./build/capture_pair --pair cam3 cam4
./build/capture_pair --pair cam4 cam5
./build/capture_pair --pair zed_left zed_right   # ZED internal stereo
```

**Key bindings during capture:**
- `SPACE` — save a pair (only when ChArUco detected in **both** cameras, ≥6 corners)
- `Q` — quit

**Live preview** shows:
- Cyan boxes: ArUco marker boundaries
- Green dots: ChArUco inner corners
- Status bar: detection count per camera

**Output:** `data/pair_X_Y/<camName>/NNNN.png` (zero-padded index).
Aim for ~40 pairs per camera combination with varied board positions (angles, distances,
all corners of the image covered).

**ZED specifics:** `capture_pair` accesses ZED as a standard V4L2 device — no ZED SDK
needed. The `--zed_dev` flag overrides the device index if needed.

---

## Step 3 — Intrinsic Calibration

### RealSense cameras (cam1, cam2, cam4, cam5)

RealSense factory intrinsics are read directly from the SDK and saved to YAML.
**Must specify capture resolution** — do not use default, which returns 1920×1080.

```bash
./build/dump_rs_intrinsics --camera cam1 --width 1280 --height 720
./build/dump_rs_intrinsics --camera cam2 --width 1280 --height 720
./build/dump_rs_intrinsics --camera cam4 --width 1280 --height 720
./build/dump_rs_intrinsics --camera cam5 --width 1280 --height 720
```

Output goes to `results/intrinsics/<name>.yaml` containing K (3×3), D (1×5),
image size, depth K/D, depth scale, and `rms: -1` (SDK-provided, not calibrated).

**Important:** If you dump at the wrong resolution and then calibrate extrinsics with
images at 1280×720, the K matrix will not match and extrinsic RMS will be ~42px instead
of sub-pixel. Always match the resolution used during `capture_pair`.

#### Calibrated intrinsics results

| Camera | fx     | fy     | cx    | cy    | Resolution |
|--------|--------|--------|-------|-------|------------|
| cam1   | 906.01 | 906.16 | 645.1 | 372.0 | 1280×720   |
| cam2   | 912.05 | 912.57 | 641.3 | 355.8 | 1280×720   |

### ZED camera (cam3)

ZED intrinsics are calibrated from captured images using `calibrate_intrinsics`:

```bash
./build/calibrate_intrinsics \
    --images data/zed_stereo/zed_left \
    --camera zed_left
```

Uses `cv::calibrateCamera` with all ChArUco frames. Requires ≥10 accepted frames.
Output: `results/intrinsics/zed_left.yaml` (or `cam3.yaml`).

**Known issue — ZED calibration divergence:** The ZED-M has a wide-angle lens (~87° HFOV).
`cv::calibrateCamera` with standard distortion model consistently converges to a wrong
local minimum (very large fx + large k1 that cancel each other), yielding RMS ~100px.
Switching to `cv::fisheye::calibrate` crashes with a `norm_u1 > 0` assertion.

**Current workaround:** `results/intrinsics/cam3.yaml` is a placeholder with estimated
values: `fx = fy = 674`, `cx = 640`, `cy = 360`, zero distortion. This means cam3-related
extrinsics are approximate.

**Pending fix:** Two-phase calibration — fix focal length first, then release; or use a
proper fisheye model with correct object-point format (CV_32FC3 shape Nx1x3 in C++).

---

## Step 4 — Pairwise Extrinsic Calibration (`calibrate_extrinsics_pair`)

Computes the 4×4 rigid transform T_a_b (points in b → frame of a) between each adjacent
pair using `cv::stereoCalibrate` with `CALIB_FIX_INTRINSIC`.

```bash
./build/calibrate_extrinsics_pair \
    --images_a data/pair_1_2/cam1 --intr_a results/intrinsics/cam1.yaml \
    --images_b data/pair_1_2/cam2 --intr_b results/intrinsics/cam2.yaml \
    --name cam1_cam2

./build/calibrate_extrinsics_pair \
    --images_a data/pair_2_3/cam2 --intr_a results/intrinsics/cam2.yaml \
    --images_b data/pair_2_3/cam3 --intr_b results/intrinsics/cam3.yaml \
    --name cam2_cam3

./build/calibrate_extrinsics_pair \
    --images_a data/pair_3_4/cam3 --intr_a results/intrinsics/cam3.yaml \
    --images_b data/pair_3_4/cam4 --intr_b results/intrinsics/cam4.yaml \
    --name cam3_cam4

./build/calibrate_extrinsics_pair \
    --images_a data/pair_4_5/cam4 --intr_a results/intrinsics/cam4.yaml \
    --images_b data/pair_4_5/cam5 --intr_b results/intrinsics/cam5.yaml \
    --name cam4_cam5
```

**Algorithm:**
1. Detect ChArUco corners in every image of both directories
2. Match frames by filename index; find common corner IDs per frame pair
3. Run `cv::stereoCalibrate` with fixed intrinsics
4. Save T_a_b (4×4), R (3×3), T (3×1), E, F, RMS to YAML

**Transform convention:** `T_a_b` maps a point from camera B into camera A's frame:
```
p_camA = T_a_b * p_camB
```

#### Pairwise extrinsics results

| Pair        | RMS (px) | Translation (m)              | Notes                        |
|-------------|----------|------------------------------|------------------------------|
| cam1_cam2   | 42.0     | [-0.016, -0.003, +0.023]     | L515 pair, reasonable        |
| cam2_cam3   | 41.8     | [-0.040, -0.027, +0.078]     | Affected by cam3 placeholder K |
| cam3_cam4   | 53.3     | [-0.018, -0.126, +0.195]     | Affected by cam3 placeholder K |
| cam4_cam5   | 101.3    | [+0.762, +0.350, +1.988]     | **Suspicious** — needs recapture |

The 42px RMS on cam1_cam2 is higher than ideal (<1px expected). Root cause:
intrinsics were initially dumped at 1920×1080 instead of 1280×720. After re-dumping
at the correct resolution the error remains ~42px, suggesting sub-optimal image capture
(insufficient board pose diversity or too few frames).

The 101px RMS on cam4_cam5 with ~2m translation is unrealistic for adjacent cameras.
This pair should be recaptured.

---

## Step 5 — Transform Chaining (`chain_transforms`)

Chains the four pairwise transforms into a single file expressing every camera relative
to cam1.

```bash
./build/chain_transforms
# Output: results/extrinsics/all_to_cam1.yaml
```

**Chain formula:**
```
T_1_1 = Identity
T_1_2 = T_cam1_cam2  (direct)
T_1_3 = T_1_2 * T_cam2_cam3
T_1_4 = T_1_3 * T_cam3_cam4
T_1_5 = T_1_4 * T_cam4_cam5
```

#### Chained transform translations from cam1 origin

| Transform | tx (m)  | ty (m)  | tz (m)  |
|-----------|---------|---------|---------|
| T_1_1     |  0.000  |  0.000  |  0.000  |
| T_1_2     | -0.016  | -0.003  | +0.023  |
| T_1_3     | -0.047  | -0.026  | +0.106  |
| T_1_4     | -0.050  | -0.164  | +0.294  |
| T_1_5     | +1.529  | -1.069  | +1.452  |

T_1_5 is unreliable due to the bad cam4_cam5 calibration.

---

## Step 6 — ZED Stereo Calibration (`calibrate_zed_stereo`)

Calibrates the ZED left↔right stereo pair for baseline and rectification.

```bash
./build/calibrate_zed_stereo
```

The live preview shows both ZED half-images with:
- Cyan boxes: ArUco marker boundaries (`drawDetectedMarkers`)
- Green dots: ChArUco inner corners (`drawDetectedCornersCharuco`)

Press `SPACE` to save a pair, `C` to run calibration on collected frames, `Q` to quit.

**Status: UNRESOLVED.** Same wide-angle divergence issue as cam3 intrinsics.
Output saved to `results/extrinsics/zed_left_right.yaml`.

---

## Step 7 — Point Cloud Snapshot (`fuse_save_ply`)

Captures one synchronized frame from cam1/cam2/cam4/cam5, unprojects depth to 3D,
transforms each cloud to cam1 frame, and saves a colored ASCII PLY file.
Does not require Open3D or ZED SDK.

```bash
./build/fuse_save_ply
./build/fuse_save_ply --out results/fused.ply --max_depth 3.0
```

**How it works:**
1. Opens each RealSense camera, aligns depth to color stream
2. For each valid depth pixel: calls `rs2_deproject_pixel_to_point` using the
   **color stream intrinsics** (post-alignment)
3. Applies the 4×4 transform: `p_cam1 = T_1_N * p_camN`
4. Writes ASCII PLY with `x y z r g b` per point

Output PLY can be opened in MeshLab, CloudCompare, or any PLY viewer.

---

## Step 8 — Real-Time Streaming Viewer (`fuse_stream`)

Streams live point clouds from selected cameras, fused into cam1 frame, rendered with
Open3D's `Visualizer` class.

### Usage

```bash
# Step by step — start with one camera to verify
./build/fuse_stream --cameras cam1

# Add cam2
./build/fuse_stream --cameras cam1,cam2

# All four (if USB bandwidth allows)
./build/fuse_stream --cameras cam1,cam2,cam4,cam5

# Tune performance
./build/fuse_stream --cameras cam1,cam2 --stride 6
./build/fuse_stream --cameras cam1,cam2 --stride 8    # lightest load
./build/fuse_stream --cameras cam1,cam2 --voxel 0.01  # 1 cm voxel filter
./build/fuse_stream --max_depth 2.5                   # limit depth range
```

### Parameters

| Flag            | Default        | Description                              |
|-----------------|----------------|------------------------------------------|
| `--cameras`     | cam1,cam2,cam4,cam5 | Comma-separated list of cameras     |
| `--stride`      | 4              | Process every Nth pixel (higher = fewer points, faster) |
| `--min_depth`   | 0.1 m          | Ignore depth below this                  |
| `--max_depth`   | 4.0 m          | Ignore depth above this                  |
| `--voxel`       | 0 (off)        | Voxel downsample size in metres          |
| `--config`      | configs/cameras.yaml | Camera serial config                |
| `--transforms`  | results/extrinsics/all_to_cam1.yaml | Chained transforms  |

### Per-camera colors

| Camera | Color   |
|--------|---------|
| cam1   | Red     |
| cam2   | Green   |
| cam4   | Yellow  |
| cam5   | Magenta |

### Architecture

```
main loop (Open3D PollEvents)
  └── appendCam() × N active cameras
        ├── wait_for_frames(200ms)
        ├── align depth → color frame
        ├── for each pixel at stride:
        │     rs2_deproject_pixel_to_point  (color intrinsics)
        │     apply T_1_N (4×4 matrix multiply)
        │     append to pts / cols vectors
        ├── UpdateGeometry(cloud)
        └── ResetViewPoint(true)  [first frame only — fits view to data]
```

**Viewer controls (Open3D standard):**
- Left drag: rotate
- Right drag / scroll: zoom
- Ctrl + drag: pan
- Close window: stop

### Known issues and fixes applied

**Issue 1 — Window showed only axes, no point cloud.**
Cause: `ResetViewPoint` was never called after the first real frame; the camera was
locked to the empty bounding box. Fix: added `vis.ResetViewPoint(true)` on the first
frame with non-empty cloud.

**Issue 2 — cam5 hangs on `pipe.start()` when all 4 cameras open simultaneously.**
Cause: USB bandwidth saturation. `pipe.start()` does not throw — it blocks indefinitely.
Fix: each camera is opened in a detached `std::thread`. The caller waits with a 6-second
`condition_variable::wait_for`. If the thread does not complete in time, the camera is
skipped and the thread keeps the shared state alive via `shared_ptr` until it eventually
returns (or the process exits).

**Issue 3 — `failed to set power state` error.**
Cause: creating an `rs2::context` for device enumeration AND then starting pipelines
caused context conflicts. Fix: enumerate devices once with a single context before any
pipelines are created, then destroy that context before calling `pipe.start()`.

**Issue 4 — Wayland warning `cannot set window position`.**
This is harmless. Open3D's GLFW backend cannot set the initial window position on Wayland.
The window still opens and renders correctly via XWayland.

---

## Build Instructions

```bash
cd multi_cam_calib/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

All binaries land in `build/`. Run commands from the `multi_cam_calib/` directory so
relative paths (`configs/`, `data/`, `results/`) resolve correctly.

**Targets built:**

| Binary                     | Dependencies              |
|----------------------------|---------------------------|
| `calibrate_intrinsics`     | OpenCV                    |
| `calibrate_extrinsics_pair`| OpenCV                    |
| `chain_transforms`         | OpenCV                    |
| `calibrate_zed_stereo`     | OpenCV                    |
| `dump_rs_intrinsics`       | librealsense2 + OpenCV    |
| `capture_pair`             | librealsense2 + OpenCV    |
| `fuse_save_ply`            | librealsense2 + OpenCV    |
| `fuse_stream`              | librealsense2 + OpenCV + Open3D |
| `fuse_and_view`            | SKIPPED (needs ZED SDK + Open3D) |

---

## Pending Work

1. **Fix ZED intrinsic calibration** — cam3.yaml is a placeholder (fx=674, zero distortion).
   Options: two-phase calibration (fix focal, then release), or proper fisheye model
   (`cv::fisheye::calibrate` with Nx1x3 CV_32FC3 object points).

2. **Recapture cam4_cam5 pair** — RMS=101px and T~2m are unrealistic.
   Ensure the ChArUco board fills the frame from various angles and distances.

3. **Improve cam1_cam2 RMS** — currently 42px; with good data and correct intrinsics
   this should be <5px. Recapture with more diverse board poses.

4. **ZED stereo calibration** — `calibrate_zed_stereo` needs the same intrinsic fix
   before stereo baseline/rectification will be reliable.

5. **Install ZED SDK** (optional) — would enable cam3 depth in `fuse_stream` and allow
   `fuse_and_view` to build.
