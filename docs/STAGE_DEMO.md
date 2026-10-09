# SO-101 真机物块抓取阶段演示

[![真机阶段演示封面](media/qianli-so101-stage-demo-20261009.jpg)](https://github.com/Suerzong/QianLi/releases/tag/stage-so101-grasp-20261009)

[阶段成果 Release](https://github.com/Suerzong/QianLi/releases/tag/stage-so101-grasp-20261009) · [1080p 播放版](https://github.com/Suerzong/QianLi/releases/download/stage-so101-grasp-20261009/qianli_so101_grasp_stage_demo_20261009_1080p.mp4) · [4K 原片](https://github.com/Suerzong/QianLi/releases/download/stage-so101-grasp-20261009/qianli_so101_grasp_stage_demo_20261009_original.mp4)

## 展示内容与范围

用户提供的实拍视频展示 SO-101 在棋盘上的物块操作：人工摆放物块，机械臂接近、夹持、抬升，以及松爪和退回。视频于 **2026-10-09 归档**，文件名日期表示归档日期。

这是机械臂研发的阶段成果展示。拍摄时的操作系统、ROS 发行版、代码提交、控制方式和标定版本未核实；不能将视频计入 Ubuntu 22.04/Humble 的真机迁移验收，也不由此推算自动抓取成功率、定位精度或长期可靠性。当前迁移验收仍按 [MIGRATION_VALIDATION.md](MIGRATION_VALIDATION.md) 记录。

## 发布文件

| 文件 | 规格与用途 |
|---|---|
| `qianli_so101_grasp_stage_demo_20261009_original.mp4` | 原始 `IMG_1987.MP4` 的内容一致副本；3840×2160、120 fps、HEVC、93.425 秒、1,150,347,093 字节 |
| `qianli_so101_grasp_stage_demo_20261009_1080p.mp4` | 网页播放版；1920×1080、30 fps、H.264 / yuv420p、AAC 双声道、93.438 秒、42,069,501 字节 |
| `SHA256SUMS.txt` | 两个视频的 SHA256 校验值 |

播放版由本地 FFmpeg 对整段原片转码，保留完整画面顺序和原 AAC 音轨，采用 MP4 faststart；未增加字幕、配乐或生成画面。0.013 秒的容器时长差来自帧率/音频封装取整。原片完整保留，不覆盖源文件。

视频存放于 GitHub Release，Git 仓库保存说明和封面，避免将大型视频加入代码历史。

## 校验与来源

```text
39f4dd5a55b14eed6b079e98036236e572e30de3bf3ad2b9383613e471c5c2cd  qianli_so101_grasp_stage_demo_20261009_original.mp4
c59b62742d88c37ab3a9cd08d2c10f6ed0d0701bda1a3a445739044a58eaf99d  qianli_so101_grasp_stage_demo_20261009_1080p.mp4
```

原始视频由项目用户提供；播放版已核对开头、中段、结尾画面、媒体参数及完整解码。Release 的源码标签对应归档文档提交，不代表视频拍摄时的运行版本。
