#!/usr/bin/env python3
"""Windows 宿主机侧的孪生环境适配层（为了用上宿主机显卡）。

背景 / 为什么需要这个文件：
  Ubuntu 虚拟机里**没有 GPU**（`lspci` 只有 VMware SVGA II 虚拟适配器，
  torch.cuda.is_available()=False），所以 RL 训练只能吃 12 个 CPU 核。
  宿主机 Windows 上有 RTX 5070 Ti Laptop，但 VM 没有做 GPU 直通。
  于是把仿真环境原样搬到宿主机：MuJoCo 仍然是 CPU 物理，
  神经网络部分用 GPU。

  原脚本 sim_grasp.py / sim_mesh_gripper.py 里的资源路径写死在
  `~/legacy/...` 和 `~/mj_parts`，在 Windows 上不存在。这个模块只做
  **路径改写**，不改任何物理参数/关节增益/碰撞分组 —— 保证宿主机上的
  孪生与虚拟机上那份逐位一致（同样的 URDF、同样的 CoACD 凸块）。

用法：
  import win_twin            # 必须在 import rl_env 之前
  import rl_env
"""


from twin_runtime import HERE, DUAL, LOCAL_SCRIPTS, URDF_PATH, PARTS_DIR, report
import sim_grasp as S
import sim_mesh_gripper as MG

if __name__ == "__main__":
    report()
