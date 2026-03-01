# AAE5303 Assignment: Visual Odometry with ORB-SLAM3

<div align="center">

![ORB-SLAM3](https://img.shields.io/badge/SLAM-ORB--SLAM3-blue?style=for-the-badge)
![VO](https://img.shields.io/badge/Mode-Visual_Odometry-green?style=for-the-badge)
![Dataset](https://img.shields.io/badge/Dataset-AMtown02-orange?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Completed-success?style=for-the-badge)

**Monocular Visual Odometry Evaluation on UAV Aerial Imagery**

*AMtown02 Dataset - MARS-LVIG*

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

This report presents the implementation and evaluation of **Monocular Visual Odometry (VO)** using **ORB-SLAM3** on the **AMtown02** UAV aerial imagery dataset (MARS-LVIG). The trajectory is evaluated against RTK ground truth using **four parallel, monocular-appropriate metrics** computed with the `evo` toolkit via the project evaluation scripts.

### Key Results

| Metric | Value | Description |
|--------|-------|-------------|
| **ATE RMSE** | **1.330 m** | Global accuracy after Sim(3) alignment (scale corrected) |
| **RPE Trans Drift** | **1.582 m/m** | Translation drift rate (mean error per meter, delta=10 m) |
| **RPE Rot Drift** | **12.20 deg/100m** | Rotation drift rate (mean angle per 100 m, delta=10 m) |
| **Completeness** | **61.30%** | Matched poses / total ground-truth poses (217 / 354) |
| **Estimated poses** | 2,014 | Trajectory poses in `CameraTrajectory.txt` |

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

1. Run monocular Visual Odometry using ORB-SLAM3 on the AMtown02 dataset
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

### AMtown02 Dataset

The dataset is from the **MARS-LVIG** UAV dataset (AMtown02 sequence).

| Property | Value |
|----------|-------|
| **Dataset Name** | AMtown02 |
| **Source** | MARS-LVIG / UAVScenes |
| **Total Images** | 2,149 (used in this run; dataset may contain more) |
| **Image Resolution** | 1280 × 720 pixels (as configured in this project) |
| **Frame Rate** | ~10 Hz |
| **Ground truth (by images)** | 354 poses (subsampled to image timestamps) |

### Data Sources

| Resource | Link |
|----------|------|
| MARS-LVIG Dataset | https://mars.hku.hk/dataset.html |
| UAVScenes GitHub | https://github.com/sijieaaa/UAVScenes |

### Ground Truth

- **Full RTK trajectory**: `docs/ground_truth_AMtown02.txt`
- **Subsampled by image timestamps** (for evaluation): `docs/ground_truth_AMtown02_by_images.txt`  
  Used so that completeness is computed as matched poses over the number of image timestamps in the evaluation set.

---

## ⚙️ Implementation Details

### System Configuration

| Component | Specification |
|-----------|---------------|
| **Framework** | ORB-SLAM3 (C++) |
| **Mode** | Monocular Visual Odometry (TUM format) |
| **Vocabulary** | ORBvoc.txt (pre-trained) |
| **Operating System** | Linux (Ubuntu) |

### Camera Calibration (AMtown02)

From `docs/AMtown_Mono.yaml`:

```yaml
Camera.type: "PinHole"
Camera.fx: 726.86
Camera.fy: 726.64
Camera.cx: 586.09
Camera.cy: 520.89

Camera.k1: -0.1210
Camera.k2: 0.1113
Camera.p1: 0.0016
Camera.p2: 0.00013
Camera.k3: -0.062353

Camera.width: 1280
Camera.height: 720
Camera.fps: 10.0
Camera.RGB: 0
```

### ORB Feature Extraction Parameters (Tuned for AMtown02)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `nFeatures` | 3000 | More features per frame for stable tracking |
| `scaleFactor` | 1.2 | Pyramid scale factor |
| `nLevels` | 10 | More levels for scale robustness in aerial footage |
| `iniThFAST` | 10 | Lower threshold for weak texture / blur |
| `minThFAST` | 4 | Minimum FAST threshold |

---

## 🚀 My Steps: Running ORB-SLAM3

### Prerequisites

1. **ORB-SLAM3** installed and built (e.g. at `/root/ORB_SLAM3` or set `ORB_SLAM3_ROOT`).
2. **Vocabulary**: `ORBvoc.txt` (e.g. in `ORB_SLAM3/Vocabulary/`).
3. **Dataset**: AMtown02 images in `data/AMtown02/rgb/` and `data/AMtown02/rgb.txt` (TUM-style: timestamp and filename per line).

### Step 1: Prepare environment and dataset

Ensure the dataset directory contains:

- `data/AMtown02/rgb.txt` — list of image timestamps and paths (e.g. `rgb/xxx.png`).
- `data/AMtown02/rgb/` — directory of images.

If `rgb.txt` is missing or incomplete, generate it (see `docs/AMtown02_跑全图说明.md` for full-sequence options).

### Step 2: Run ORB-SLAM3 (Monocular TUM)

From the project root:

```bash
cd /home/workspace

export ORB_SLAM3_ROOT=/root/ORB_SLAM3
export ORB_SLAM3_VOCAB=/root/ORB_SLAM3/Vocabulary/ORBvoc.txt
export TRAJ_OUT_DIR=/home/workspace/data

./scripts/run_orb_slam3_mono.sh \
  /home/workspace/data/AMtown02 \
  /home/workspace/docs/AMtown_Mono.yaml
```

- **Dataset directory**: `data/AMtown02` (must contain `rgb.txt` and `rgb/`).
- **Camera and ORB settings**: `docs/AMtown_Mono.yaml`.
- **Trajectory output**: Written to `TRAJ_OUT_DIR` (here `data/`), i.e. `data/CameraTrajectory.txt` and `data/KeyFrameTrajectory.txt`.

Run until the sequence finishes or you stop with `Ctrl+C`. Ensure a display is available (e.g. X11); for headless runs, use a virtual display (e.g. Xvfb) or see project docs.

### Step 3: Evaluate trajectory and generate figures

Using the project script (prefer **CameraTrajectory.txt** for higher completeness):

```bash
cd /home/workspace

./scripts/evaluate_AMtown02_by_images.sh data/CameraTrajectory.txt
```

This script:

- Uses ground truth: `docs/ground_truth_AMtown02_by_images.txt`
- Runs `scripts/evaluate_vo_accuracy.py` (ATE, RPE, completeness) and writes `evaluation_results_AMtown02/metrics.json`
- Generates trajectory figures in `figures_AMtown02/` (e.g. `trajectory_evaluation.png`, `trajectory_3d_3d.png`, `trajectory_3d_2d.png`)

### Step 4 (Optional): Merge figures into one image

```bash
python3 scripts/merge_figures.py \
  --input-dir figures_AMtown02 \
  --output figures_AMtown02/AMtown02_trajectory_combined.png \
  --layout vertical
```

---

## 📈 Results and Analysis

### Evaluation Results

```
================================================================================
VISUAL ODOMETRY EVALUATION RESULTS (AMtown02)
================================================================================

Ground Truth:  docs/ground_truth_AMtown02_by_images.txt (354 poses)
Estimated:     data/CameraTrajectory.txt (2,014 poses)
Matched Poses: 217 / 354 (61.30%)  ← Completeness

METRIC 1: ATE (Absolute Trajectory Error)
────────────────────────────────────────
RMSE:   1.3302 m
Mean:   0.9552 m
Std:    0.9257 m

METRIC 2: RPE Translation Drift (distance-based, delta=10 m)
────────────────────────────────────────
Mean translational RPE over 10 m: 15.8196 m
Translation drift rate:           1.5820 m/m

METRIC 3: RPE Rotation Drift (distance-based, delta=10 m)
────────────────────────────────────────
Mean rotational RPE over 10 m: 1.2198 deg
Rotation drift rate:              12.1977 deg/100m

================================================================================
```

### Trajectory Alignment

- **Sim(3)** alignment with scale correction is applied before computing ATE/RPE.
- **Association**: `t_max_diff = 0.1 s`, **RPE delta** = 10 m.

### Performance Summary

| Metric | Value | Interpretation |
|--------|-------|----------------|
| **ATE RMSE** | 1.33 m | Global error after alignment |
| **RPE Trans Drift** | 1.58 m/m | Local translation drift rate |
| **RPE Rot Drift** | 12.20 deg/100m | Local rotation drift rate |
| **Completeness** | 61.30% | Share of ground-truth poses matched and evaluated |

---

## 📊 Visualizations

### Trajectory comparison

Figures are under `figures_AMtown02/`:

- **trajectory_evaluation.png** — 2×2: trajectory before/after alignment, ATE histogram, ATE along trajectory.
- **trajectory_3d_3d.png** — 3D trajectory comparison (GT vs estimated).
- **trajectory_3d_2d.png** — 2D top-down view.
- **AMtown02_trajectory_combined.png** — All of the above merged into one image (vertical layout).

<img width="1200" height="3360" alt="image" src="https://github.com/user-attachments/assets/ad19424a-ac23-4ded-a0b3-75a72a555ba1" />


*(Generated from `ground_truth_AMtown02_by_images.txt` and `CameraTrajectory.txt` with the project evaluation script and `evo`.)*

---

## 💭 Discussion

### Strengths

1. **Full-frame trajectory**: Using `CameraTrajectory.txt` (after enabling it in ORB-SLAM3 for monocular) improves completeness (61.30%) compared to keyframe-only.
2. **Tuned ORB parameters**: Higher `nFeatures` (3000), more pyramid levels (10), and lower FAST thresholds help tracking on aerial sequences.
3. **Reproducible pipeline**: Single script for running SLAM and one for evaluation and figures.

### Limitations

1. **Completeness**: Only 61.30% of ground-truth poses are matched; tracking loss or timestamp mismatch can reduce this.
2. **Scale and drift**: Monocular VO has scale ambiguity; drift rates reflect local consistency after Sim(3) alignment.
3. **No loop closure**: Pure VO mode; no loop closure or relocalization.

### Error sources

- **Tracking failures**: Occasional lost tracking reduces the number of estimated poses that can be aligned to ground truth.
- **Calibration and resolution**: Camera intrinsics and resolution (1280×720) must match the actual images.
- **Motion and texture**: Fast motion or weak texture can degrade feature matching and pose estimation.

---

## 🎯 Conclusions

1. ✅ **ORB-SLAM3** was run successfully on **AMtown02** in monocular TUM mode, producing **CameraTrajectory.txt** and **KeyFrameTrajectory.txt**.
2. ✅ **Evaluation** was performed with the project script and **evo**, reporting **ATE RMSE 1.33 m**, **RPE trans drift 1.58 m/m**, **RPE rot drift 12.20 deg/100m**, and **completeness 61.30%**.
3. ✅ **Trajectory figures** and a **combined figure** were generated under `figures_AMtown02/`.
4. ⚠️ **Completeness** can be improved by ensuring full-sequence tracking and using full-frame trajectory; parameter tuning (e.g. in `docs/AMtown_Mono.yaml`) may help.

### Recommendations

| Priority | Action |
|----------|--------|
| High | Use `CameraTrajectory.txt` for evaluation (not only keyframes) |
| High | Match camera resolution and intrinsics in the YAML to the actual images |
| Medium | Tune ORB parameters (e.g. `nFeatures`, FAST thresholds) for your sequence |
| Low | Consider VIO or loop closure if available for better scale and drift |

---

## 📚 References

1. Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M., & Tardós, J. D. (2021). **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**. *IEEE Transactions on Robotics*, 37(6), 1874-1890.

2. Sturm, J., et al. (2012). **A Benchmark for the Evaluation of RGB-D SLAM Systems**. IROS.

3. Geiger, A., et al. (2012). **Are we ready for Autonomous Driving? The KITTI Dataset**. CVPR.

4. MARS-LVIG Dataset: https://mars.hku.hk/dataset.html

5. ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3

---

## 📎 Appendix

### A. Repository structure (this project)

```
.
├── README.md                           # This report
├── requirements.txt
├── AMtown.yaml
├── data/
│   ├── AMtown02/
│   │   ├── rgb.txt
│   │   └── rgb/                        # (images; not in repo if too large)
│   ├── CameraTrajectory.txt           # Full-frame trajectory
│   └── KeyFrameTrajectory.txt
├── docs/
│   ├── AMtown_Mono.yaml                # ORB-SLAM3 camera + ORB settings
│   ├── ground_truth_AMtown02.txt
│   ├── ground_truth_AMtown02_by_images.txt
│   └── AMtown02_跑全图说明.md
├── figures_AMtown02/
│   ├── trajectory_evaluation.png
│   ├── trajectory_3d_3d.png
│   ├── trajectory_3d_2d.png
│   └── AMtown02_trajectory_combined.png
├── evaluation_results_AMtown02/
│   └── metrics.json
├── output/
│   └── evaluation_report.json
├── scripts/
│   ├── run_orb_slam3_mono.sh           # Run ORB-SLAM3 monocular TUM
│   ├── evaluate_AMtown02_by_images.sh  # Evaluate + plot
│   ├── evaluate_vo_accuracy.py
│   ├── generate_report_figures.py
│   ├── visualize_trajectory_3d.py
│   └── merge_figures.py
└── leaderboard/
    ├── README.md
    ├── AMtown02_leaderboard.json
    └── submission_template.json
```

### B. Running commands (summary)

```bash
# 1. Run ORB-SLAM3 on AMtown02
export ORB_SLAM3_ROOT=/root/ORB_SLAM3
export ORB_SLAM3_VOCAB=/root/ORB_SLAM3/Vocabulary/ORBvoc.txt
export TRAJ_OUT_DIR=/home/workspace/data
./scripts/run_orb_slam3_mono.sh data/AMtown02 docs/AMtown_Mono.yaml

# 2. Evaluate and generate figures
./scripts/evaluate_AMtown02_by_images.sh data/CameraTrajectory.txt

# 3. (Optional) Merge figures
python3 scripts/merge_figures.py --input-dir figures_AMtown02 \
  --output figures_AMtown02/AMtown02_trajectory_combined.png --layout vertical
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
