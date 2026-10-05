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

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DUAL = os.path.dirname(HERE)                       # ...\dual_twin
LOCAL_SCRIPTS = os.path.join(
    os.path.dirname(DUAL), 'qianli_ws', 'src', 'qianli_vision', 'scripts')

# 让 import sim_grasp / rl_env 找得到本地镜像里的脚本
for p in (LOCAL_SCRIPTS, HERE):
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

URDF_PATH = os.path.join(DUAL, 'urdf', 'so101.urdf')
PARTS_DIR = os.path.join(DUAL, 'mj_parts')

# 关键：用**环境变量**把路径传下去，而不是 import 后再 monkeypatch。
# 原因：Windows 上 SubprocVecEnv 用 spawn 起子进程，子进程里
# `import win_twin` 不一定会被重新执行（取决于 __main__ 怎么被复现），
# monkeypatch 就丢了 → 子进程又去找 /home/ros/... 然后崩掉（实测 EOFError）。
# 环境变量是进程创建时继承的，一定能传到子进程。
os.environ['QI_SO101_PKG'] = DUAL          # → <DUAL>/urdf/so101.urdf
os.environ['QI_PARTS_DIR'] = PARTS_DIR

import sim_grasp as S            # noqa: E402
import sim_mesh_gripper as MG    # noqa: E402


def report():
    print(f'win_twin: URDF = {S.URDF}')
    print(f'win_twin: 凸分解零件 = {MG.PARTS_DIR} '
          f'({len(os.listdir(MG.PARTS_DIR))} 个文件)')
    print(f'win_twin: 本地脚本目录 = {LOCAL_SCRIPTS}')


if __name__ == '__main__':
    report()
