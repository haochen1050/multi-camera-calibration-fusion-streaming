#!/usr/bin/env python3
"""
Diagnose cam1+cam2 stereo calibration quality.
 - Runs fresh stereoCalibrate on new captures
 - Shows per-frame RMS with new T
 - Filters outlier frames and shows improved RMS
"""
import cv2, numpy as np, glob, os, sys

# ── Board ──────────────────────────────────────────────────────────────────────
SQUARES_X, SQUARES_Y = 20, 20
SQUARE_LEN  = 0.030   # m
MARKER_LEN  = 0.0225  # m
DICT_ID     = cv2.aruco.DICT_5X5_250

aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)
board = cv2.aruco.CharucoBoard_create(SQUARES_X, SQUARES_Y,
                                      SQUARE_LEN, MARKER_LEN, aruco_dict)
params = cv2.aruco.DetectorParameters_create()

def detect(img_path, K, D):
    img = cv2.imread(img_path)
    if img is None:
        return [], []
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape)==3 else img
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    if ids is None or len(ids) == 0:
        return [], []
    _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board, cameraMatrix=K, distCoeffs=D)
    if ch_ids is None or len(ch_ids) < 6:
        return [], []
    return ch_corners.reshape(-1,2), ch_ids.flatten()

# ── Intrinsics ─────────────────────────────────────────────────────────────────
def load_intr(path):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    K = fs.getNode("K").mat().astype(np.float64)
    D = fs.getNode("D").mat().astype(np.float64)
    w = int(fs.getNode("image_width").real())
    h = int(fs.getNode("image_height").real())
    fs.release()
    return K, D, (w, h)

base = "/home/aup/determ/Calibration/multi_cam_calib"
KA, DA, szA = load_intr(f"{base}/results/intrinsics/cam1.yaml")
KB, DB, szB = load_intr(f"{base}/results/intrinsics/cam2.yaml")
print(f"Intrinsics loaded: cam1 {szA}, cam2 {szB}")

# ── Load images ────────────────────────────────────────────────────────────────
dir_a = f"{base}/data/pair_1_2/cam1"
dir_b = f"{base}/data/pair_1_2/cam2"

files_a = sorted(glob.glob(f"{dir_a}/*.png"))
files_b = sorted(glob.glob(f"{dir_b}/*.png"))
print(f"Images: cam1={len(files_a)}  cam2={len(files_b)}")

# ── Detect corners ─────────────────────────────────────────────────────────────
obj_pts, img_a, img_b, fnames = [], [], [], []
skipped = 0
for fa, fb in zip(files_a, files_b):
    assert os.path.basename(fa) == os.path.basename(fb), "filename mismatch"
    cA, iA = detect(fa, KA, DA)
    cB, iB = detect(fb, KB, DB)
    if len(cA) == 0 or len(cB) == 0:
        skipped += 1
        continue
    # common IDs
    common = np.intersect1d(iA, iB)
    if len(common) < 15:
        skipped += 1
        continue
    idxA = [np.where(iA==c)[0][0] for c in common]
    idxB = [np.where(iB==c)[0][0] for c in common]
    ptsA = cA[idxA]
    ptsB = cB[idxB]
    obj3d = np.array([board.chessboardCorners[c] for c in common], dtype=np.float32)
    obj_pts.append(obj3d)
    img_a.append(ptsA.astype(np.float32))
    img_b.append(ptsB.astype(np.float32))
    fnames.append(os.path.basename(fa))

print(f"Valid pairs: {len(fnames)}  skipped: {skipped}")
if len(fnames) < 5:
    print("Not enough frames!")
    sys.exit(1)

# ── stereoCalibrate ────────────────────────────────────────────────────────────
crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7)
rms, KA_, DA_, KB_, DB_, R, T, E, F = cv2.stereoCalibrate(
    obj_pts, img_a, img_b,
    KA.copy(), DA.copy(), KB.copy(), DB.copy(),
    szA, flags=cv2.CALIB_FIX_INTRINSIC, criteria=crit)
print(f"\n=== stereoCalibrate RMS: {rms:.3f} px ===")
print(f"T (m): [{T[0,0]:.4f}, {T[1,0]:.4f}, {T[2,0]:.4f}]")

# ── Per-frame RMS with new R,T ─────────────────────────────────────────────────
print(f"\n{'file':>12}  {'RMS_A':>7}  {'RMS_B':>8}  {'pts':>4}  {'dist':>6}  flag")
print("-" * 60)

per_frame_rms = []
for i, fname in enumerate(fnames):
    pts3 = obj_pts[i]
    pA   = img_a[i]
    pB   = img_b[i]

    # solvePnP in cam1
    ok, rvec, tvec = cv2.solvePnP(pts3, pA, KA, DA)
    if not ok:
        print(f"{fname:>12}  solvePnP failed")
        continue

    # cam1 reprojection RMS
    proj_A, _ = cv2.projectPoints(pts3, rvec, tvec, KA, DA)
    rms_a = float(np.sqrt(np.mean((pA - proj_A.reshape(-1,2))**2)))

    # Project into cam2
    R_mat, _ = cv2.Rodrigues(rvec)
    R2 = R @ R_mat
    T2 = R @ tvec + T
    proj_B, _ = cv2.projectPoints(pts3, cv2.Rodrigues(R2)[0], T2, KB, DB)
    rms_b = float(np.sqrt(np.mean((pB - proj_B.reshape(-1,2))**2)))

    dist = float(np.linalg.norm(tvec))
    flag = " *** OUTLIER" if rms_b > 15 else ""
    per_frame_rms.append((fname, rms_a, rms_b, len(pts3), dist))
    print(f"{fname:>12}  {rms_a:7.2f}  {rms_b:8.2f}  {len(pts3):4d}  {dist:.2f}m{flag}")

# ── Summary ────────────────────────────────────────────────────────────────────
rms_bs = [x[2] for x in per_frame_rms]
rms_as = [x[1] for x in per_frame_rms]
print(f"\nMean RMS_A: {np.mean(rms_as):.2f}px  (single-cam reprojection floor)")
print(f"Mean RMS_B: {np.mean(rms_bs):.2f}px  (stereo projection error)")
print(f"Median RMS_B: {np.median(rms_bs):.2f}px")

# Filter and rerun
THRESH = 15.0
good = [(o,a,b,n) for (fname,ra,rb,n,d),(o,a,b,nn) in
        zip(per_frame_rms, zip(obj_pts,img_a,img_b,[x[3] for x in per_frame_rms]))
        if rb < THRESH]
if good:
    obj2,ia2,ib2,_ = zip(*[(obj_pts[i],img_a[i],img_b[i],0)
                            for i,(fn,ra,rb,n,d) in enumerate(per_frame_rms) if rb < THRESH])
    rms2, *_ = cv2.stereoCalibrate(
        list(obj2), list(ia2), list(ib2),
        KA.copy(), DA.copy(), KB.copy(), DB.copy(),
        szA, flags=cv2.CALIB_FIX_INTRINSIC, criteria=crit)
    n_good = len(obj2)
    n_out  = len(per_frame_rms) - n_good
    print(f"\nAfter removing {n_out} outlier frames (RMS_B > {THRESH}px):")
    print(f"  {n_good} frames -> stereoCalibrate RMS: {rms2:.3f} px")
