# firmware/stm32 — STM32 底层固件

电机实时控制、Encoder、PID、底层传感器、安全保护与急停。

- **职责边界**：只做实时底层；SLAM / 导航 / 感知由 Linux 上位机负责
- **建议工具链**：STM32CubeIDE / STM32CubeMX / STM32CubeProgrammer / OpenOCD（当前开发机已安装）
- **通信**：与上位机通过串口 / CAN 解耦，帧协议与 ROS 驱动封装在 qianli_base / qianli_arm
- 当前状态：⏳ 待底盘与控制器到位后启动
