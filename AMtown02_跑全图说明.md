# AMtown02 Full-Sequence Run Guide

## Objective

Run ORB-SLAM3 with **minimal tracking loss** over **as many images as possible (e.g. full ~7498 frames)**, to obtain a longer trajectory (more keyframes and/or full-frame trajectory).

## Parameter Tuning Applied (`docs/AMtown_Mono.yaml`)

| Parameter | Purpose |
|-----------|---------|
| **nFeatures: 3000** | More features per frame for more stable tracking and fewer "Tracking lost" events due to insufficient features |
| **nLevels: 10** | Extra pyramid levels for better scale handling in aerial footage and longer trajectories |
| **iniThFAST: 10, minThFAST: 4** | Lower corner thresholds so enough keypoints are extracted in blur, weak texture, and over/under-exposed frames, reducing frame drops |

## If Tracking Is Still Lost Mid-Run

1. **Check resolution**  
   `Camera.width` and `Camera.height` in the YAML must match the actual image size in `data/AMtown02/rgb/`.  
   - If images are 2448×2048 but the YAML has 1280×720, update to 2448×2048 and scale intrinsics fx, fy, cx, cy accordingly (or use calibration for that resolution).

2. **Frame rate**  
   Set `Camera.fps` close to the actual capture rate (e.g. 10) to avoid large motion model errors.

3. **Re-run the sequence**  
   Sometimes the first run loses tracking in the second half due to initialization or occasional drops; run 1–2 more times with the same parameters and check if the trajectory length improves.

## Ensure rgb.txt Covers All Frames (e.g. 7498)

If `data/AMtown02/rgb.txt` has only a few hundred lines, it lists only a subset of images. To run the full sequence, regenerate `rgb.txt`:

- **If images are in the dataset root** (`data/AMtown02/*.png`) and `times.txt` exists:
  ```bash
  python3 scripts/generate_rgb_txt_from_times.py data/AMtown02
  ```
- **If images are in the root but there is no times.txt**:
  ```bash
  python3 scripts/generate_rgb_txt.py data/AMtown02 --image-subdir .
  ```

Then run ORB-SLAM3 again to process the full frame list.

## How to Run

```bash
cd /home/workspace
export ORB_SLAM3_ROOT=/root/ORB_SLAM3
export ORB_SLAM3_VOCAB=/root/ORB_SLAM3/Vocabulary/ORBvoc.txt

/home/workspace/scripts/run_orb_slam3_mono.sh \
  /home/workspace/data/AMtown02 \
  /home/workspace/docs/AMtown_Mono.yaml
```

Exit with `Ctrl+C` when done. Output trajectories are written to `data/KeyFrameTrajectory.txt` and, if enabled, `data/CameraTrajectory.txt`.  
Compare **trajectory line count / rgb.txt line count (e.g. ~7498)** to see what fraction of the sequence was processed.
