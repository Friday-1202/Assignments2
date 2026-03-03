# AAE5303 Assignment: Visual Odometry with ORB-SLAM3

<div align="center">

![ORB-SLAM3](https://img.shields.io/badge/SLAM-ORB--SLAM3-blue?style=for-the-badge)
![VO](https://img.shields.io/badge/Mode-Visual_Odometry-green?style=for-the-badge)
![Dataset](https://img.shields.io/badge/Dataset-HKisland03-orange?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Completed-success?style=for-the-badge)

**Monocular Visual Odometry Evaluation on UAV Aerial Imagery**

*HKisland03 Dataset*

</div>

---

## 📋 Table of Contents

1. [Executive Summary](#-executive-summary)
2. [Introduction](#-introduction)
3. [Methodology](#-methodology)
4. [Dataset Description](#-dataset-description)
5. [Implementation Details](#-implementation-details)
6. [My Steps: Running ORB-SLAM3](#-my-steps-running-orb-slam3)
7. [Results and Analysis](#-results-and-analysis)
8. [Visualizations](#-visualizations)
9. [Discussion](#-discussion)
10. [Conclusions](#-conclusions)
11. [References](#-references)
12. [Appendix](#-appendix)

---

## 📊 Executive Summary

This report presents the implementation and evaluation of **Monocular Visual Odometry (VO)** using **ORB-SLAM3** on the **HKisland03** sequence (downsampled to 1224×1024). The trajectory is evaluated against RTK ground truth using **four parallel, monocular-appropriate metrics** computed with the `evo` toolkit via the project evaluation scripts.

### Key Results

| Metric | Value | Description |
|--------|-------|-------------|
| **ATE RMSE** | **33.52 m** | Global accuracy after Sim(3) alignment (scale corrected) |
| **RPE Trans Drift** | **1.40 m/m** | Translation drift rate (mean error per meter, delta=10 m) |
| **RPE Rot Drift** | **107.37 deg/100m** | Rotation drift rate (mean angle per 100 m, delta=10 m) |
| **Completeness** | **94.27%** | Matched poses / total images (3687 / 3911) |
| **Estimated poses** | 3,687 | Trajectory poses in `CameraTrajectory.txt` |

---

## 📖 Introduction

### Background

ORB-SLAM3 is a state-of-the-art visual SLAM system capable of performing:

- **Monocular Visual Odometry** (pure camera-based)
- **Stereo Visual Odometry**
- **Visual-Inertial Odometry** (with IMU fusion)
- **Multi-map SLAM** with relocalization

This assignment focuses on **Monocular VO mode**, which:

- Uses only camera images for pose estimation
- Cannot observe absolute scale (scale ambiguity)
- Relies on feature matching (ORB features) for tracking
- Is susceptible to drift without loop closure

### Objectives

1. Run monocular Visual Odometry using ORB-SLAM3 on the HKisland03 dataset
2. Obtain full-frame trajectory (`CameraTrajectory.txt`) and keyframe trajectory (`KeyFrameTrajectory.txt`)
3. Evaluate trajectory accuracy against provided ground truth using four parallel metrics
4. Document the complete workflow for reproducibility

### Scope

This assignment evaluates:

- **ATE (Absolute Trajectory Error)**: Global trajectory accuracy after Sim(3) alignment (monocular-friendly)
- **RPE drift rates (translation + rotation)**: Local consistency (drift per traveled distance)
- **Completeness**: Robustness / coverage (how much of the sequence is successfully tracked and evaluated)

---

## 🔬 Methodology

### ORB-SLAM3 Visual Odometry Overview

ORB-SLAM3 performs visual odometry through the following pipeline:

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Input Image    │────▶│   ORB Feature   │────▶│   Feature       │
│  Sequence       │     │   Extraction    │     │   Matching      │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                         │
┌─────────────────┐     ┌─────────────────┐     ┌────────▼────────┐
│   Trajectory    │◀────│   Pose          │◀────│   Motion        │
│   Output        │     │   Estimation    │     │   Model         │
└─────────────────┘     └────────┬────────┘     └─────────────────┘
                                 │
                        ┌────────▼────────┐
                        │   Local Map     │
                        │   Optimization  │
                        └─────────────────┘
```

### Evaluation Metrics

#### 1. ATE (Absolute Trajectory Error)

Measures the RMSE of the aligned trajectory after Sim(3) alignment:

$$ATE_{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}\|\mathbf{p}_{est}^i - \mathbf{p}_{gt}^i\|^2}$$

**Reference**: Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012

#### 2. RPE (Relative Pose Error) – Drift Rates

Measures local consistency by comparing relative transformations. We report drift as **rates**:

- **Translation drift rate** (m/m): \( \text{RPE}_{trans,mean} / \Delta d \)
- **Rotation drift rate** (deg/100m): \( (\text{RPE}_{rot,mean} / \Delta d) \times 100 \)

where \(\Delta d = 10\) m.

**Reference**: Geiger et al., "Vision meets Robotics: The KITTI Dataset", IJRR 2013

#### 3. Completeness

$$Completeness = \frac{N_{matched}}{N_{gt}} \times 100\%$$

#### Why Sim(3) alignment?

Monocular VO has **scale ambiguity**. All metrics are computed after Sim(3) alignment (rotation + translation + scale) so that accuracy reflects **trajectory shape** and **drift**, not an arbitrary global scale.

### Evaluation Protocol

- **Trajectory format**: TUM (`t tx ty tz qx qy qz qw`)
- **Timestamp association**: `t_max_diff = 0.1 s`
- **Alignment**: Sim(3) with scale correction
- **RPE delta**: `delta = 10 m` (distance domain)

---

## 📁 Dataset Description

### HKisland03 Dataset

The dataset consists of a monocular sequence (`HKisland03`) with images downsampled to **1224 × 1024** and synchronized RTK ground truth.

| Property | Value |
|----------|-------|
| **Dataset Name** | HKisland03 |
| **Total Images** | 3,911 (images listed in `data/HKisland03/rgb.txt`) |
| **Image Resolution** | 1224 × 1024 pixels (after 0.5× downsampling) |
| **Frame Rate** | ~20 Hz (per `HKisland_Mono_1224x1024.yaml`) |

### Ground Truth

- **RTK trajectory**: `docs/ground_truth_HKisland03.txt` (TUM format, `t tx ty tz qx qy qz qw`)

---

## ⚙️ Implementation Details

### System Configuration

| Component | Specification |
|-----------|---------------|
| **Framework** | ORB-SLAM3 (C++, Monocular TUM mode) |
| **Vocabulary** | ORBvoc.txt (pre-trained) |
| **Dataset** | HKisland03 (downsampled to 1224 × 1024, 3,911 images) |
| **Trajectory Output** | `CameraTrajectory.txt` (full-frame) + `KeyFrameTrajectory.txt` (keyframes) |
| **Ground Truth** | `docs/ground_truth_HKisland03.txt` (RTK, TUM format) |
| **Operating System** | Linux (Ubuntu / ROS2 Humble container) |

### Camera Calibration (HKisland03)

From `docs/HKisland_Mono_1224x1024.yaml`:

```yaml
Camera.type: "PinHole"

# Intrinsics for 1224×1024 (0.5x downsampled)
Camera.fx: 726.86
Camera.fy: 726.64
Camera.cx: 586.09
Camera.cy: 520.89

Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

Camera.width: 1224
Camera.height: 1024
Camera.fps: 20.0
Camera.RGB: 0        # BGR (OpenCV default)
```

### ORB Feature Extraction Parameters (HKisland03)

```yaml
ORBextractor.nFeatures: 2000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 6
```

---

## 🚀 My Steps: Running ORB-SLAM3

### Prerequisites

1. **ORB-SLAM3** installed and built (e.g. at `/root/ORB_SLAM3` or set `ORB_SLAM3_ROOT`).
2. **Vocabulary**: `ORBvoc.txt` (e.g. in `ORB_SLAM3/Vocabulary/`).
3. **Dataset**: HKisland03 images in `data/HKisland03/rgb/` and `data/HKisland03/rgb.txt` (TUM-style: timestamp and filename per line, 3911 images).

### Step 1: Dataset layout

```text
data/HKisland03/
├── rgb.txt              # 3911 valid image lines (+1 header)
└── rgb/                 # 3911 PNG images
    ├── 1698132948.799810.png
    ├── 1698132948.899896.png
    └── ...
```

`rgb.txt` follows the TUM RGB-D format:

```text
# timestamp filename
1698132948.799810 rgb/1698132948.799810.png
...
```

### Step 2: Build and configure ORB-SLAM3

```bash
# Build ORB-SLAM3 (one-time)
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
./build.sh

# Place ORBvoc.txt at:
#   ORB_SLAM3/Vocabulary/ORBvoc.txt
```

Environment variables (run from project root `/home/workspace`):

```bash
export ORB_SLAM3_ROOT=/root/ORB_SLAM3
export ORB_SLAM3_VOCAB=/root/ORB_SLAM3/Vocabulary/ORBvoc.txt

# Optional: slow down playback in Viewer (2 = half speed)
export ORB_SLAM3_PLAYBACK_SCALE=2
```

### Step 3: Run ORB-SLAM3 (Monocular TUM, HKisland03)

From the project root:

```bash
cd /home/workspace

./scripts/run_orb_slam3_mono.sh \
  data/HKisland03 \
  docs/HKisland_Mono_1224x1024.yaml
```

After the sequence finishes, the script produces:

```text
data/
├── CameraTrajectory.txt     # full-frame trajectory (used for evaluation)
├── KeyFrameTrajectory.txt   # keyframe trajectory
└── HKisland03/...
```

### Step 4: Evaluate trajectory and generate figures (HKisland03)

I use a dedicated helper script `scripts/run_evaluate_HKisland03.sh` that:

- Applies a **timestamp offset** `t_offset = -28430400 s` so that estimated timestamps line up with RTK ground truth.
- Sets the **completeness denominator to 3911**, i.e. the total number of images in `rgb.txt`, so that  
  completeness = `matched_poses / 3911` instead of `/ #groundtruth_poses`.

From the project root:

```bash
cd /home/workspace
./scripts/run_evaluate_HKisland03.sh
```

Internally this expands to:

```bash
# 1) Metrics (ATE / RPE / Completeness), with evo
python3 scripts/evaluate_vo_accuracy.py \
  --groundtruth docs/ground_truth_HKisland03.txt \
  --estimated data/CameraTrajectory.txt \
  --t-max-diff 0.1 \
  --t-offset -28430400 \
  --delta-m 10 \
  --completeness-denominator 3911 \
  --workdir evaluation_results \
  --json-out evaluation_results/metrics.json

# 2) 2×2 report figure (pre/post alignment + ATE plots)
python3 scripts/generate_report_figures.py \
  --gt docs/ground_truth_HKisland03.txt \
  --est data/CameraTrajectory.txt \
  --evo-ape-zip evaluation_results/ate.zip \
  --out figures_HKisland03/trajectory_evaluation.png \
  --t-max-diff 0.1 \
  --t-offset -28430400

# 3) 3D / 2D trajectory views (Sim(3) aligned)
python3 scripts/visualize_trajectory_3d.py \
  --gt docs/ground_truth_HKisland03.txt \
  --est data/CameraTrajectory.txt \
  --evo-ate-zip evaluation_results/ate.zip \
  --out figures_HKisland03/trajectory_3d \
  --t-max-diff 0.1 \
  --t-offset -28430400

# 4) Merge figures into a single vertical panel
python3 scripts/merge_figures.py \
  --input-dir figures_HKisland03 \
  --output figures_HKisland03/HKisland03_trajectory_combined.png \
  --layout vertical
```

---

## 📈 Results and Analysis

### Evaluation Results (HKisland03)

```
================================================================================
VISUAL ODOMETRY EVALUATION RESULTS (HKisland03)
================================================================================

Ground Truth:  docs/ground_truth_HKisland03.txt
Estimated:     data/CameraTrajectory.txt (3,687 poses)
Matched Poses: 3,687 / 3,911 (94.27%)  ← Completeness

METRIC 1: ATE (Absolute Trajectory Error)
────────────────────────────────────────
RMSE:   33.5171 m
Mean:   29.1782 m
Std:    16.4931 m

METRIC 2: RPE Translation Drift (distance-based, delta=10 m)
────────────────────────────────────────
Mean translational RPE over 10 m: 13.9557 m
Translation drift rate:           1.3956 m/m

METRIC 3: RPE Rotation Drift (distance-based, delta=10 m)
────────────────────────────────────────
Mean rotational RPE over 10 m: 10.7373 deg
Rotation drift rate:            107.3734 deg/100m

================================================================================
```

### Trajectory Alignment

- **Sim(3)** alignment with scale correction is applied before computing ATE/RPE.
- **Association**: `t_max_diff = 0.1 s`, **RPE delta** = 10 m.

### Performance Summary

| Metric | Value | Interpretation |
|--------|-------|----------------|
| **ATE RMSE** | 33.52 m | Global error after alignment |
| **RPE Trans Drift** | 1.40 m/m | Local translation drift rate |
| **RPE Rot Drift** | 107.37 deg/100m | Local rotation drift rate |
| **Completeness** | 94.27% | Evaluated frames / total images (3687 / 3911) |

---

## 📊 Visualizations

### Trajectory comparison

Figures are under `figures_HKisland03/`:

- **trajectory_evaluation.png** — 2×2: trajectory before/after alignment, ATE histogram, ATE along trajectory.
- **trajectory_3d_3d.png** — 3D trajectory comparison (GT vs estimated).
- **trajectory_3d_2d.png** — 2D top-down view.
- **HKisland03_trajectory_combined.png** — All of the above merged into one image (vertical layout).

![Trajectory Evaluation](figures_HKisland03/trajectory_evaluation.png)

*(Generated from `ground_truth_HKisland03.txt` and `CameraTrajectory.txt` with the project evaluation script and `evo`.)*

---

## 💭 Discussion

### Strengths

1. **Full-frame trajectory**: Using `CameraTrajectory.txt` (after enabling it in ORB-SLAM3 for monocular) achieves high completeness (94.27%) compared to keyframe-only evaluation.
2. **Tuned front-end**: Lower FAST thresholds and appropriate ORB settings help stabilize tracking on downsampled aerial images (1224×1024).
3. **Reproducible pipeline**: A single bash script (`run_evaluate_HKisland03.sh`) drives evaluation and figure generation from a finished ORB-SLAM3 run.

### Limitations

1. **High drift**: Despite good completeness, ATE and drift metrics are relatively large on HKisland03, reflecting challenging motion and limited observability (pure monocular VO, no loop closure).
2. **Scale and drift**: Monocular VO has scale ambiguity; drift rates reflect local consistency after Sim(3) alignment rather than absolute navigation accuracy.
3. **No loop closure / IMU**: The experiment uses pure monocular VO without loop closure or IMU fusion, which limits long-term consistency.

### Error sources

- **Tracking failures / weak features**: Low-texture regions, motion blur, and large parallax can still cause temporary tracking degradation and outliers.
- **Calibration and resolution**: Intrinsics must match the downsampled 1224×1024 images (`HKisland_Mono_1224x1024.yaml`); any mismatch directly impacts accuracy.
- **RTK alignment and timestamps**: A large constant time offset between images and RTK ground truth must be compensated (`t_offset = -28430400 s`); residual timing errors will influence evaluation.

---

## 🎯 Conclusions

1. ✅ **ORB-SLAM3** was run successfully on **HKisland03** in monocular TUM mode, producing both **CameraTrajectory.txt** (full-frame) and **KeyFrameTrajectory.txt**.
2. ✅ **Evaluation** was performed with the project scripts and **evo**, reporting **ATE RMSE 33.52 m**, **RPE trans drift 1.40 m/m**, **RPE rot drift 107.37 deg/100m**, and **completeness 94.27% (3687 / 3911)**.
3. ✅ **Trajectory figures** and a **combined figure** were generated under `figures_HKisland03/`.
4. ⚠️ **Long-term drift** remains significant due to the challenging monocular aerial setting and lack of loop closure/IMU, leaving room for future improvements.

### Recommendations

| Priority | Action |
|----------|--------|
| High | Use `CameraTrajectory.txt` (full-frame) for evaluation rather than keyframes only |
| High | Ensure calibration (`HKisland_Mono_1224x1024.yaml`) matches the downsampled image resolution and lens characteristics |
| Medium | Further tune ORB parameters (e.g. feature count, FAST thresholds) and consider robust outlier handling for HKisland03 |
| Low | Explore adding IMU fusion or loop closure to reduce long-term drift on this sequence |

---

## 📚 References

1. Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M., & Tardós, J. D. (2021). **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**. *IEEE Transactions on Robotics*, 37(6), 1874-1890.

2. Sturm, J., et al. (2012). **A Benchmark for the Evaluation of RGB-D SLAM Systems**. IROS.

3. Geiger, A., et al. (2012). **Are we ready for Autonomous Driving? The KITTI Dataset**. CVPR.

4. ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAMLab/ORB_SLAM3

---

## 📎 Appendix

### A. Repository structure (this project)

```
.
├── README.md                           # This report
├── requirements.txt
├── data/
│   ├── HKisland03/
│   │   ├── rgb.txt
│   │   └── rgb/                        # (images; not in repo if too large)
│   ├── CameraTrajectory.txt           # Full-frame trajectory (HKisland03)
│   └── KeyFrameTrajectory.txt
├── docs/
│   ├── HKisland_Mono_1224x1024.yaml   # ORB-SLAM3 camera + ORB settings for HKisland03
│   ├── ground_truth_HKisland03.txt    # RTK ground truth (TUM format)
├── figures_HKisland03/
│   ├── trajectory_evaluation.png
│   ├── trajectory_3d_3d.png
│   ├── trajectory_3d_2d.png
│   └── HKisland03_trajectory_combined.png
├── evaluation_results/
│   └── metrics.json
├── scripts/
│   ├── run_orb_slam3_mono.sh           # Run ORB-SLAM3 monocular TUM
│   ├── run_evaluate_HKisland03.sh      # Evaluate + plot for HKisland03
│   ├── evaluate_vo_accuracy.py
│   ├── generate_report_figures.py
│   ├── visualize_trajectory_3d.py
│   └── merge_figures.py
└── leaderboard/
    ├── README.md
    └── submission_template.json
```

### B. Running commands (summary)

```bash
# 1. Run ORB-SLAM3 on HKisland03
export ORB_SLAM3_ROOT=/root/ORB_SLAM3
export ORB_SLAM3_VOCAB=/root/ORB_SLAM3/Vocabulary/ORBvoc.txt
./scripts/run_orb_slam3_mono.sh data/HKisland03 docs/HKisland_Mono_1224x1024.yaml

# 2. Evaluate and generate figures for HKisland03
./scripts/run_evaluate_HKisland03.sh
```

### C. Output trajectory format (TUM)

```
# timestamp x y z qx qy qz qw
1658132909.697110 0.0000000 0.0000000 0.0000000 -0.0000000 -0.0000000 -0.0000000 1.0000000
1658132909.797510 -0.0011234 -0.0010792 -0.0482122 0.0054858 -0.0078534 0.0011552 0.9999534
...
```

---

<div align="center">

**AAE5303 - Robust Control Technology in Low-Altitude Aerial Vehicle**

*Department of Aeronautical and Aviation Engineering*

*The Hong Kong Polytechnic University*

Mar 2026

</div>
