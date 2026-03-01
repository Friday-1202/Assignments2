# AMtown02 尽量跑完全部图像、拉长轨迹 说明

## 目标

让 ORB-SLAM3 尽可能**不中途丢跟踪**，跑完**更多/全部约 7498 张图像**，得到更长的飞行轨迹（更多关键帧或全帧轨迹）。

## 已做的参数调整（`docs/AMtown_Mono.yaml`）

| 参数 | 作用 |
|------|------|
| **nFeatures: 3000** | 每帧提取更多特征，跟踪更稳，减少因特征不足导致的 Tracking lost |
| **nLevels: 10** | 金字塔多两层，对尺度变化大的航拍更友好，长轨迹更稳 |
| **iniThFAST: 10, minThFAST: 4** | 降低角点阈值，在模糊、弱纹理、过曝/欠曝帧也能提出足够点，减少丢帧 |

## 若仍中途丢跟踪，可再排查

1. **分辨率是否一致**  
   `Camera.width` / `Camera.height` 必须和 `data/AMtown02/rgb/` 里实际图像尺寸一致。  
   - 若图像是 2448×2048，而 yaml 里写 1280×720，需要改成 2448/2048，并把内参 fx,fy,cx,cy 按同比例缩放（或使用该分辨率下的标定值）。

2. **帧率**  
   `Camera.fps` 建议和实际采集频率接近（如 10），避免运动模型偏差过大。

3. **重新跑一遍**  
   有时第一次跑会因初始化或偶然丢帧导致后半段全丢；用当前参数多跑 1～2 次，看轨迹是否明显变长。

## 确保 rgb.txt 包含全部 7498 帧

若当前 `data/AMtown02/rgb.txt` 只有三百多行，说明只列了部分图像（例如只扫了 `rgb/` 子目录）。要跑满 7498 帧，需重新生成 rgb.txt：

- **若图像在数据集根目录**（`data/AMtown02/*.png`，且存在 `times.txt`）：
  ```bash
  python3 scripts/generate_rgb_txt_from_times.py data/AMtown02
  ```
- **若图像在根目录但无 times.txt**：
  ```bash
  python3 scripts/generate_rgb_txt.py data/AMtown02 --image-subdir .
  ```

生成后再跑 ORB-SLAM3，即可按 7498 帧序列运行。

## 运行方式

```bash
cd /home/workspace
export ORB_SLAM3_ROOT=/root/ORB_SLAM3
export ORB_SLAM3_VOCAB=/root/ORB_SLAM3/Vocabulary/ORBvoc.txt

/home/workspace/scripts/run_orb_slam3_mono.sh \
  /home/workspace/data/AMtown02 \
  /home/workspace/docs/AMtown_Mono.yaml
```

跑完后用 `Ctrl+C` 正常退出。轨迹在 `data/KeyFrameTrajectory.txt`（若未写全则可能还有 `data/CameraTrajectory.txt`）。  
比较「轨迹行数 / rgb.txt 行数（约 7498）」可知当前大约跑过了多少比例的图像。
