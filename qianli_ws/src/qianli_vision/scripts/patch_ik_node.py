#!/usr/bin/env python3
"""给 so101_bringup/ik_node.py 打补丁：加数值雅可比 DLS IK

问题：驱动只用 ikpy 的 inverse_kinematics_frame，且以上一次的 q_target 为种子。
      目标稍远就卡在局部极小，残差几百毫米 → 直接判"不可达"拒绝
      （实测 425mm 处残差 386.9mm，而同一目标 DLS IK 只要 0.09mm）。

方案（不引入新依赖，驱动侧没有 mujoco）：
  1. 仍用 ikpy 解一次，作为"姿态合理"的初值
  2. 用 ikpy 自己的 forward_kinematics 做**数值雅可比**，跑阻尼最小二乘(DLS)
     只优化位置（姿态沿用 ikpy 初值 —— 5 自由度臂本就无法同时满足 6 维位姿）
  3. 若仍不收敛，换多个初值（上次解 / 当前关节 / 零位 / 限位中点）重试
  4. 仍然超过 max_ik_residual 才拒绝（保持原有安全语义）

用法：
  python3 patch_ik_node.py            # 打补丁（自动备份 .bak_dlsik）
  python3 patch_ik_node.py --revert   # 撤销
"""

from project_paths import arm_source_path

import argparse
import os
import shutil
import sys

TARGETS = [
    os.path.expanduser(arm_source_path('so101_bringup/ik_node.py')),
]

MARK = '# ---- DLS IK patch ----'

HELPERS = '''
    # ---- DLS IK patch ----
    def _tip_matrix(self, q):
        """链末端位姿矩阵（用 ikpy 自己的 FK）。"""
        return self.chain.forward_kinematics(np.asarray(q, dtype=float),
                                             full_kinematics=True)[-1]

    def _numeric_jacobian(self, q, eps=2e-4):
        """数值雅可比（3×n，仅位置）。固定关节列为 0，无副作用。"""
        q = np.asarray(q, dtype=float)
        p0 = self._tip_matrix(q)[:3, 3]
        J = np.zeros((3, len(q)))
        for i, act in enumerate(self.chain.active_links_mask):
            if not act or i >= len(q):
                continue
            qp = q.copy()
            qp[i] += eps
            J[:, i] = (self._tip_matrix(qp)[:3, 3] - p0) / eps
        return J

    def _clip_bounds(self, q):
        """按链上各 link 的 bounds 夹紧。"""
        for i, link in enumerate(self.chain.links):
            if i >= len(q):
                break
            b = getattr(link, 'bounds', None)
            if b is not None and b[0] is not None and b[1] is not None:
                q[i] = min(max(q[i], float(b[0])), float(b[1]))
        return q

    def _dls_refine(self, q0, target_pos, iters=300, lam=1e-2, step=1.0):
        """阻尼最小二乘精修位置。返回 (q, 残差)。"""
        q = self._clip_bounds(np.asarray(q0, dtype=float).copy())
        n = len(q)
        for _ in range(iters):
            p = self._tip_matrix(q)[:3, 3]
            e = np.asarray(target_pos, dtype=float) - p
            if np.linalg.norm(e) < 5e-5:
                break
            J = self._numeric_jacobian(q)
            A = J @ J.T + lam * np.eye(3)
            try:
                dq = J.T @ np.linalg.solve(A, e)
            except np.linalg.LinAlgError:
                break
            dq = np.clip(dq * step, -0.1, 0.1)
            for i, act in enumerate(self.chain.active_links_mask):
                if i < n and not act:
                    dq[i] = 0.0
            q = self._clip_bounds(q + dq)
        err = float(np.linalg.norm(self._tip_matrix(q)[:3, 3] - target_pos))
        return q, err

    def _seed_list(self, ikpy_sol):
        """多个初值：ikpy 解 / 上次目标 / 当前关节 / 零位 / 限位中点。"""
        n = len(self.chain.links)
        seeds = []
        if ikpy_sol is not None:
            seeds.append(np.asarray(ikpy_sol, dtype=float))
        seeds.append(np.asarray(self.q_target, dtype=float))
        seeds.append(np.asarray(self.q_current, dtype=float))
        seeds.append(np.zeros(n))
        mid = np.zeros(n)
        for i, link in enumerate(self.chain.links):
            b = getattr(link, 'bounds', None)
            if b is not None and b[0] is not None and b[1] is not None:
                mid[i] = 0.5 * (float(b[0]) + float(b[1]))
        seeds.append(mid)
        return seeds

    def _solve_dls(self, target_mat):
        """ikpy 初值 + DLS 精修 + 多初值重试。"""
        target_pos = np.asarray(target_mat[:3, 3], dtype=float)
        orientation_mode = (
            None if self.orientation_mode == 'none' else self.orientation_mode)
        ikpy_sol = None
        try:
            ikpy_sol = self.chain.inverse_kinematics_frame(
                target_mat, initial_position=self.q_target,
                orientation_mode=orientation_mode)
        except Exception as exc:  # noqa: BLE001
            self._warn_throttled(f'ikpy seed failed: {exc}')
        best_q, best_err = None, float('inf')
        for s in self._seed_list(ikpy_sol):
            q, err = self._dls_refine(s, target_pos)
            if err < best_err:
                best_q, best_err = q, err
            if best_err <= self.max_ik_residual:
                break
        return best_q, best_err

    def _solve_ik(self):
        sol, err = self._solve_dls(self.target_mat)
        if sol is None or err > self.max_ik_residual:
            self._warn_throttled(
                f'Rejected unreachable target: residual {err * 1000:.1f} mm'
                ' (DLS+multi-seed)')
            return
        if err > self.reach_warn:
            self._warn_throttled(
                f'Target out of reach: residual {err * 1000:.0f} mm')
        self.q_target = np.asarray(sol, dtype=float)
    # ---- DLS IK patch end ----
'''


OLD_SOLVE_START = '    def _solve_ik(self):'


def patch(path):
    with open(path, 'r', encoding='utf-8') as fh:
        src = fh.read()
    if MARK in src:
        print(f'  已打过补丁，跳过: {path}')
        return False
    i = src.find(OLD_SOLVE_START)
    if i < 0:
        print(f'  ❌ 找不到 _solve_ik: {path}')
        return False
    # 找到该函数结束（下一个顶格 def/class 之前）
    j = src.find('\n    def ', i + len(OLD_SOLVE_START))
    if j < 0:
        print(f'  ❌ 找不到 _solve_ik 的结尾: {path}')
        return False
    shutil.copy2(path, path + '.bak_dlsik')
    new = src[:i] + HELPERS.strip('\n') + '\n' + src[j + 1:]
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(new)
    print(f'  ✅ 已打补丁（备份 {os.path.basename(path)}.bak_dlsik）')
    return True


def revert(path):
    bak = path + '.bak_dlsik'
    if os.path.exists(bak):
        shutil.copy2(bak, path)
        print(f'  ✅ 已还原 {path}')
    else:
        print(f'  ❌ 没有备份: {bak}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--revert', action='store_true')
    a = ap.parse_args()
    for p in TARGETS:
        if not os.path.exists(p):
            print(f'  ❌ 不存在: {p}')
            continue
        if a.revert:
            revert(p)
        else:
            patch(p)
            # 顺带把 install 目录下的拷贝也同步（之前踩过"只改 src 不生效"的坑）
            inst = p.replace('/src/', '/install/')
            if os.path.exists(inst):
                shutil.copy2(p, inst)
                print(f'  ✅ 同步到 install: {inst}')
            checks = [p + '.bak_dlsik']
            for c in checks:
                if os.path.exists(c):
                    print(f'     备份: {c}')


if __name__ == '__main__':
    main()
