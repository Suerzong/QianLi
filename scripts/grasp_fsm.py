#!/usr/bin/env python3
"""抓取状态机（显式状态 + 每步安全校验 + JSON 日志）。

状态序列（按用户规格）：
  INIT         读配置/限位/home，检查力矩
  PLAN         计算"固定爪在物块右侧正上方、工具轴垂直桌面"的目标位姿
  MOVE_ABOVE   移动到目标上方
  OPEN         张开爪子
  DESCEND      数值竖直下落，直到固定爪接近桌面/抓取高度
  CLOSE        合爪找接触
  LIFT         抬起（避免回位时拖拽物块）
  RETURN_HOME  回到折叠 baseline
  DONE / ERROR

baseline（折叠位）由 --capture-home 现场记录；同时算出夹进安全限位的
"可执行回位位姿"，避免再次触发 0x02 锁存错误。
"""

from project_paths import open_video_capture

from project_paths import default_arm_port

from project_paths import arm_source_path, default_camera, driver_params_path, project_path
import argparse
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
import yaml

sys.path.insert(0, arm_source_path())
sys.path.insert(0, os.path.expanduser(
    project_path('qianli_ws/src/qianli_vision/scripts')))
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CONFIG = os.path.expanduser(
    driver_params_path())
CFG_DIR = os.path.expanduser(project_path('config'))
HOME_PATH = os.path.join(CFG_DIR, 'home_pose.json')
SAFE_PATH = os.path.join(CFG_DIR, 'safe_limits.json')
TABLE_Z = -0.06485       # 拖拽标定实测（平面残差 0.073mm），非旧值 -0.06909
DOWN = np.array([0.0, 0.0, -1.0])
# 示教抓取模板位（2026-10-06 实测成功抓过方块）：贴近工作区、已知安全。
# 从"折叠位"直接去目标上方时，关节空间插值会让夹爪下探到 -110mm（桌面以下
# 45mm）；经这个中间位再走笛卡尔接近，路径最低只到 -63mm，安全得多。
READY_Q = np.array([-0.11658253987930872, 0.9480001269133262,
                    -0.4586602555778067, 1.2363885150358267,
                    -1.5508545765523831, 0.3098641191528995])


class FSM:
    def __init__(self, a):
        self.a = a
        self.log = []
        cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
        self.zero = np.array(cfg['zero_raw'])
        self.dir = np.array(cfg['direction'])
        lo = (np.array(cfg['raw_min']) - self.zero) * self.dir * 2 * math.pi / 4096
        hi = (np.array(cfg['raw_max']) - self.zero) * self.dir * 2 * math.pi / 4096
        self.drv_lo, self.drv_hi = np.minimum(lo, hi), np.maximum(lo, hi)
        self.lo, self.hi = self.drv_lo.copy(), self.drv_hi.copy()
        if os.path.exists(SAFE_PATH):
            sl = json.load(open(SAFE_PATH))
            self.lo = np.maximum(self.lo, np.array(sl['rad_lo']))
            self.hi = np.minimum(self.hi, np.array(sl['rad_hi']))
        self.bus = FeetechSerialBus(default_arm_port(), timeout_s=0.08)
        self.model = GripperModel(stride=8)
        self.state = 'INIT'

    # ---------- 基础 ----------
    def enter(self, s, **kw):
        self.state = s
        rec = {'state': s, 't': datetime.now().strftime('%H:%M:%S'), **kw}
        self.log.append(rec)
        print(f'[{rec["t"]}] ▶ {s}' + (f'  {kw}' if kw else ''), flush=True)

    def read(self):
        """读关节角。串口偶发超时（USB 链路问题）自动重试。"""
        last = None
        for _ in range(8):
            try:
                return (np.array(self.bus.read_positions()) - self.zero) \
                    * self.dir * 2 * math.pi / 4096
            except Exception as exc:
                last = exc
                time.sleep(0.2)
        raise RuntimeError(f'读位置连续失败: {last}')

    def read_raw(self):
        return np.array(self.bus.read_positions())

    def load_pct(self):
        for _ in range(6):
            try:
                return self.bus.read_gripper_load()[0]
            except Exception:
                time.sleep(0.15)
        return 0.0

    def write(self, q):
        q = np.asarray(q, dtype=float)
        if not np.all(np.isfinite(q)):
            raise ValueError('non-finite command')
        over = float(np.max(np.maximum(self.lo - q, q - self.hi)))
        if over > 0.05:
            raise ValueError(f'command exceeds safe limits by {over:.3f} rad')
        qc = np.clip(q, self.lo, self.hi)
        raw = np.rint(self.zero + qc * self.dir * 4096
                      / (2 * math.pi)).astype(int).tolist()
        last = None
        for _ in range(8):
            try:
                self.bus.write_positions(raw)
                return
            except Exception as exc:
                last = exc
                time.sleep(0.2)
        raise RuntimeError(f'写位置连续失败: {last}')

    def fk(self, q):
        return self.model.solve(dict(zip(JOINTS, q)))

    def fixed_tip(self, q):
        F = self.fk(q)['gripper_frame_link']
        return F[:3, 3] + F[:3, :3] @ self.p_fix

    def lowest(self, q):
        return self.model.lowest_over_all(dict(zip(JOINTS, q)))

    def move(self, goal, segs=8, tol=0.05, wait=3.0):
        goal = np.clip(np.asarray(goal, dtype=float), self.lo, self.hi)
        start = self.read()
        for i in range(1, segs + 1):
            seg = start + (goal - start) * i / segs
            t0 = time.monotonic()
            while time.monotonic() - t0 < wait:
                self.write(seg)
                time.sleep(0.2)
                if np.max(np.abs(self.read() - seg)) < tol:
                    break

    # ---------- 状态实现 ----------
    def s_init(self):
        self.enter('INIT')
        raw = self.read_raw()
        q = self.read()
        self.home_captured = None
        self.home_exec = None
        if os.path.exists(HOME_PATH):
            h = json.load(open(HOME_PATH))
            self.home_captured = np.array(h['raw'])
            self.home_exec = np.array(h['raw_exec'])
            print(f'  baseline 折叠位(记录): {self.home_captured.tolist()}')
            print(f'  baseline 折叠位(可执行): {self.home_exec.tolist()}')
        else:
            print('  ⚠ 尚无 baseline（先跑 --capture-home）')
        print(f'  当前位置 raw {raw.tolist()}')
        # 固定爪内侧尖端（工具系常向量）
        T0 = self.fk(np.zeros(6))
        F0, G0 = T0['gripper_frame_link'], T0[FLANGE_LINK]
        w0 = (G0[:3, :3] @ self.model.parts[FLANGE_LINK].T).T + G0[:3, 3]
        pf = (F0[:3, :3].T @ (w0 - F0[:3, 3]).T).T
        inner = pf[np.abs(pf[:, 0]) < 0.004]
        if len(inner) == 0:
            inner = pf[np.argsort(np.abs(pf[:, 0]))[:max(20, len(pf) // 50)]]
        # 关键：工具系的 z 指向下方时，爪尖在 z **最大**一侧（+6.3mm）。
        # 用 z 最小会取到腕部/机身端，与真爪尖相差 105mm
        # （用"示教抓取位"交叉验证过：真爪尖在 TCP 下方 6.2mm 且是网格世界最低点）。
        self.p_fix = inner[int(np.argmax(inner[:, 2]))]
        print(f'  固定爪内侧尖端(工具系) {np.round(self.p_fix*1000,1).tolist()} mm')
        torque = self.bus.read_torque_states()
        print(f'  力矩 {torque}')
        if torque != [1] * 6:
            print('  力矩未全开 -> 自动使能（抓取必须有力矩）')
            self.bus.set_torque(True)
            time.sleep(0.8)
            torque = self.bus.read_torque_states()
            print(f'  使能后力矩 {torque}')
            if torque != [1] * 6:
                print('  ❌ 力矩使能失败（可能有舵机故障）')
                return False
        # 方块"下侧"方向 = 棋盘 +y（朝机械臂那侧）。由长期坐标系的仿射取。
        frame_path = os.path.join(CFG_DIR, 'board_frame.json')
        rd = None
        if os.path.exists(frame_path):
            aff = np.array(json.load(open(frame_path))['affine'])
            v = aff[:2, 1]
            rd = np.array([v[0], v[1], 0.0])
            rd /= np.linalg.norm(rd)
            print(f'  下侧方向(棋盘+y) = {np.round(rd,4).tolist()}')
        if rd is None:
            rd = np.array([-0.992, -0.126, 0.0])
            rd /= np.linalg.norm(rd)
            print(f'  下侧方向(缺省) = {np.round(rd,4).tolist()}')
        self.rd = rd
        return True

    def s_plan(self):
        self.enter('PLAN')
        blk = np.array([self.a.x, self.a.y, 0.0])
        # 固定爪内侧面放在方块侧面**外侧**留间隙：下探全程不碰方块，
        # 合爪时由活动爪把方块推过来贴住固定爪。
        # （间隙为 0 时下探会擦着方块，实测把方块蹭开了。）
        gap = self.a.clearance_mm / 1000.0
        face = blk + (self.a.half_mm / 1000.0 + gap) * self.rd
        tip_z = self.a.tip_z_mm / 1000.0
        self.tgt_hi = np.array([face[0], face[1], tip_z + self.a.hover_mm / 1000.0])
        self.tgt_lo = np.array([face[0], face[1], tip_z])
        print(f'  物块中心 ({blk[0]:.4f},{blk[1]:.4f})  半宽 {self.a.half_mm:.0f}mm'
              f'  外侧间隙 {self.a.clearance_mm:.0f}mm')
        print(f'  固定爪目标(下侧面外侧) ({face[0]:.4f},{face[1]:.4f})')
        print(f'  上方 z={self.tgt_hi[2]*1000:+.1f}mm -> 抓取 z={self.tgt_lo[2]*1000:+.1f}mm')
        self.seeds = [self.read()]
        rng = np.random.default_rng(5)
        self.seeds += [self.lo[:5] + rng.random(5) * (self.hi[:5] - self.lo[:5])
                       for _ in range(10)]
        q_hi, e_hi = self.solve_tip(self.tgt_hi, self.a.open_rad)
        print(f'  IK 上方误差 {e_hi*1000:.1f} mm')
        if e_hi > 0.006:
            print('  目标不可达')
            return False
        self.q_hi = q_hi
        return True

    def solve_tip(self, target_p, grip, align=True, ref=None):
        """解 IK。ref 给定时，在位置及格的解里挑"关节变化最小"的。

        为什么不能只挑位置误差最小的：多起点里可能有一个位置很准、但姿态
        很奇怪的解（夹爪朝下扎到桌面以下 -110mm）。
        """
        w = 0.15 if align else 0.0

        def residual(arm):
            q = np.r_[arm, grip]
            F = self.fk(q)['gripper_frame_link']
            p = F[:3, 3] + F[:3, :3] @ self.p_fix
            zax = F[:3, :3] @ np.array([0.0, 0.0, 1.0])
            xax = F[:3, :3] @ np.array([1.0, 0.0, 0.0])
            return np.r_[p - target_p, w * (zax - DOWN), w * (xax - self.rd)]
        cands = []
        best, bc = None, None
        for s in self.seeds:
            seed = np.clip(s[:5], self.lo[:5] + 1e-6, self.hi[:5] - 1e-6)
            sol = least_squares(residual, seed,
                                bounds=(self.lo[:5], self.hi[:5]), max_nfev=400)
            c = np.r_[sol.x, grip]
            e = float(np.linalg.norm(self.fixed_tip(c) - target_p))
            if bc is None or e < bc:
                best, bc = c, e
            if e < 0.004:
                d = 0.0 if ref is None else float(
                    np.linalg.norm(c[:5] - ref[:5]))
                cands.append((d, c, e))
        if cands:
            cands.sort(key=lambda t: t[0])
            return cands[0][1], cands[0][2]
        return best, bc

    def cartesian_move(self, target_tip, grip, lift_margin=0.06, label='',
                       n_traverse=6):
        """笛卡尔途经点：先抬到安全高度 -> 平移到目标上方 -> 下探到位。

        为什么不用关节空间插值：关节插值会让夹爪在中途塌到桌面以下
        （实测曾到 -110mm，比桌面低 45mm）。这里每一步都做净空检查。
        """
        p_cur = self.fixed_tip(self.read())
        travel_z = max(p_cur[2], target_tip[2] + 0.10)     # 高位平移
        align_z = target_tip[2] + lift_margin              # 中位对齐
        # 阶段1/2：抬升 + 平移到目标上方（不约束姿态：折叠姿态下工具轴
        #          本来不竖直，此时强加竖直约束会 IK 无解）
        wps = [(np.array([p_cur[0], p_cur[1], travel_z]), False)]
        for i in range(1, n_traverse + 1):
            a = i / n_traverse
            wps.append((np.array([p_cur[0] + (target_tip[0] - p_cur[0]) * a,
                                  p_cur[1] + (target_tip[1] - p_cur[1]) * a,
                                  travel_z]), False))
        # 阶段3：竖直降到中位（仍不约束姿态）
        if align_z < travel_z - 1e-6:
            wps.append((np.array([target_tip[0], target_tip[1], align_z]),
                        False))
        # 阶段4：在目标上方中位处对齐成竖直（位置不动，只转姿态）
        wps.append((np.array([target_tip[0], target_tip[1], align_z]), True))
        # 阶段5：带竖直约束下探
        if target_tip[2] < align_z - 1e-6:
            wps.append((np.array(target_tip), True))
        solved = []
        worst = None
        for wp, align in wps:
            q_c, err = self.solve_tip(wp, grip, align=align)
            if err > 0.006:
                print(f'  {label} 途经点 {np.round(wp,3).tolist()} '
                      f'(align={align}) IK 误差 {err*1000:.1f}mm，中止')
                return False
            low, lk = self.lowest(q_c)
            if worst is None or low[2] < worst[0]:
                worst = (low[2], lk, wp)
            if low[2] < TABLE_Z + 0.006:
                print(f'  {label} 途经点最低 {lk} z={low[2]*1000:+.2f}mm '
                      f'低于桌面安全线，中止')
                return False
            solved.append(q_c)
        print(f'  {label} 路径最低点 {worst[1]} z={worst[0]*1000:+.2f}mm '
              f'(桌面 {TABLE_Z*1000:+.2f})')
        for q_c in solved:
            self.move(q_c, segs=4, tol=0.05, wait=2.0)
        return True

    def s_goto_ready(self):
        """先到"示教中间位"（用户手拖示教，绕开关节插值的下塌路径）。"""
        self.enter('GOTO_READY')
        q_cur = self.read()
        rdy_path = os.path.join(CFG_DIR, 'ready_pose.json')
        if os.path.exists(rdy_path):
            raw = np.array(json.load(open(rdy_path))['raw_exec'])
            q_rdy = ((raw - self.zero) * self.dir * 2 * math.pi / 4096)
            print(f'  用示教中间位 raw {raw.tolist()}')
        else:
            q_rdy = READY_Q
            print('  未示教中间位，用示教抓取模板位')
        q_rdy = np.clip(q_rdy, self.lo, self.hi)
        n = 20
        worst = None
        for i in range(1, n + 1):
            q = q_cur + (q_rdy - q_cur) * i / n
            low, lk = self.lowest(q)
            if worst is None or low[2] < worst[0]:
                worst = (low[2], lk)
        print(f'  路径最低点 {worst[1]} z={worst[0]*1000:+.2f}mm '
              f'(桌面 {TABLE_Z*1000:+.2f})')
        if worst[0] < TABLE_Z - 0.003:
            print('  路径低于桌面超过 3mm（超出网格近似误差），中止')
            return False
        if worst[0] < TABLE_Z + 0.006:
            print('  ⚠ 路径贴近桌面（网格近似误差量级内），继续')
        self.move(q_rdy, segs=16, tol=0.05, wait=2.5)
        p = self.fixed_tip(self.read())
        print(f'  已到模板位 固定爪 ({p[0]:.4f},{p[1]:.4f},{p[2]*1000:+.1f}mm)')
        return True

    def snapshot(self, tag):
        """关键时刻拍照存证（/tmp/fsm_<tag>.jpg）。"""
        try:
            import cv2
            cap = open_video_capture(default_camera())
            img = None
            for _ in range(8):
                ok, f = cap.read()
                if ok:
                    img = f
            cap.release()
            if img is not None:
                path = f'/tmp/fsm_{tag}.jpg'
                cv2.imwrite(path, img)
                print(f'  📷 {path}', flush=True)
        except Exception as exc:
            print(f'  拍照失败: {exc}', flush=True)

    def s_move_above(self):
        self.enter('MOVE_ABOVE')
        # 优先：从当前位（示教中间位）关节空间直线走到"对齐悬停位"。
        # 距离近时最稳；先做净空预检。
        q_cur = self.read()
        n = 16
        worst = None
        for i in range(1, n + 1):
            q = q_cur + (self.q_hi - q_cur) * i / n
            low, lk = self.lowest(q)
            if worst is None or low[2] < worst[0]:
                worst = (low[2], lk)
        print(f'  关节直线到悬停位: 最低 {worst[1]} z={worst[0]*1000:+.2f}mm '
              f'(桌面 {TABLE_Z*1000:+.2f})')
        if worst[0] >= TABLE_Z - 0.003:
            self.move(self.q_hi, segs=12, tol=0.05, wait=2.0)
            p = self.fixed_tip(self.read())
            low, lk = self.lowest(self.read())
            print(f'  到位 固定爪 ({p[0]:.4f},{p[1]:.4f},{p[2]*1000:+.1f}mm)  '
                  f'最低 {lk} z={low[2]*1000:+.2f}mm')
            return True
        print('  关节直线不安全，改用笛卡尔途经点')
        if not self.cartesian_move(self.tgt_hi, self.a.open_rad, label='上方'):
            return False
        p = self.fixed_tip(self.read())
        print(f'  到位 固定爪 ({p[0]:.4f},{p[1]:.4f},{p[2]*1000:+.1f}mm)')
        return True

    def s_open(self):
        self.enter('OPEN')
        q = self.read()
        for g in np.linspace(q[5], self.a.open_rad, 15):
            qq = q.copy()
            qq[5] = g
            self.write(qq)
            time.sleep(0.08)
        print(f'  爪口张开到 {self.a.open_rad:.3f} rad')
        return True

    def s_descend(self):
        self.enter('DESCEND')
        for i in range(1, 13):
            want = self.tgt_hi + (self.tgt_lo - self.tgt_hi) * i / 12
            q_c, err = self.solve_tip(want, self.a.open_rad)
            if err > 0.006:
                print(f'  第{i}步 IK 误差 {err*1000:.1f}mm，停下')
                break
            self.write(q_c)
            time.sleep(0.32)
        p = self.fixed_tip(self.read())
        low, lk = self.lowest(self.read())
        print(f'  落到底 固定爪 ({p[0]:.4f},{p[1]:.4f},{p[2]*1000:+.1f}mm)  '
              f'≥桌面 {(p[2]-TABLE_Z)*1000:.1f}mm')
        print(f'  整臂最低点 {lk} z={low[2]*1000:+.2f}mm')
        if p[2] < TABLE_Z + 0.004:
            print('  固定爪低于安全高度，中止')
            return False
        self.p_bottom = p
        self.snapshot('descended')
        return True

    def s_close(self):
        self.enter('CLOSE')
        q0 = self.read()
        load0 = self.load_pct()
        close = q0.copy()
        hits, pct = 0, load0
        deadline = time.monotonic() + 20
        while close[5] > self.a.close_min_rad and time.monotonic() < deadline:
            close[5] = max(self.a.close_min_rad, close[5] - 0.006)
            low, lk = self.lowest(close)
            if low[2] < TABLE_Z + 0.0008:
                print(f'  合爪会碰桌面({lk})，停下')
                break
            self.write(close)
            time.sleep(0.09)
            pct = self.load_pct()
            hits = hits + 1 if pct >= max(self.a.load_pct, load0 + 6) else 0
            if hits >= 3:
                break
        print(f'  载荷 {pct:.1f}%  夹爪 {close[5]:.4f}rad  hits={hits}')
        self.snapshot('closed')
        if hits < 3:
            print('  未检到接触（物块可能不在固定爪左侧）')
            return False
        m = self.read()
        close[5] = max(self.lo[5], m[5] - 0.008)
        self.write(close)
        time.sleep(0.4)
        self.hold_rad = float(close[5])
        return True

    def s_lift(self):
        self.enter('LIFT')
        p0 = self.fixed_tip(self.read())
        for i in range(1, 11):
            want = p0 + np.array([0, 0, self.a.lift_mm / 1000 * i / 10])
            q_c, err = self.solve_tip(want, self.hold_rad if hasattr(self, 'hold_rad')
                                      else self.read()[5])
            if err > 0.006:
                break
            self.write(q_c)
            time.sleep(0.3)
        p1 = self.fixed_tip(self.read())
        self.lift_mm = float((p1[2] - p0[2]) * 1000)
        load = self.load_pct()
        print(f'  抬起 {self.lift_mm:.1f}mm  保持载荷 {load:.1f}%')
        return True

    def s_return_home(self):
        self.enter('RETURN_HOME')
        if self.home_exec is None:
            print('  无 baseline，跳过')
            return True
        q_home = ((self.home_exec - self.zero) * self.dir * 2 * math.pi / 4096)
        q_home = np.clip(q_home, self.lo, self.hi)
        tip_home = self.fixed_tip(q_home)
        grip = float(self.read()[5])
        print(f'  折叠位固定爪 ({tip_home[0]:.4f},{tip_home[1]:.4f},'
              f'{tip_home[2]*1000:+.1f}mm)  保持夹爪 {grip:.4f}rad')
        if not self.cartesian_move(tip_home, grip, lift_margin=0.03,
                                   label='回位', n_traverse=8):
            print('  笛卡尔回位失败，改用关节插值（带净空预检）')
            self.move(q_home, segs=12, tol=0.06, wait=2.5)
        q = self.read()
        p = self.fixed_tip(q)
        low, lk = self.lowest(q)
        print(f'  回到折叠位 raw {self.read_raw().tolist()}')
        print(f'  固定爪 ({p[0]:.4f},{p[1]:.4f},{p[2]*1000:+.1f}mm)  '
              f'整臂最低 {lk} z={low[2]*1000:+.2f}mm')
        return True

    def finish(self, ok, err=None):
        out = {'completed': bool(ok), 'error': err, 'log': self.log,
               'state_final': self.state}
        path = self.a.report
        Path(path).write_text(json.dumps(out, indent=2, ensure_ascii=False))
        print(f'\n状态机结束: {"成功" if ok else "未完成"}  日志 → {path}')

    def run(self):
        """执行流程。无论成功或失败，收尾都会张开爪子并自动归位。"""
        ok = True
        try:
            if not self.s_init():
                self.finish(False, 'INIT 未通过')
                self.bus.close()
                return
            steps = [self.s_plan, self.s_goto_ready, self.s_move_above,
                     self.s_open, self.s_descend, self.s_close, self.s_lift]
            for st in steps:
                if not st():
                    ok = False
                    print(f'  ⚠ {self.state} 未通过 —— 仍将自动归位', flush=True)
                    break
        except BaseException as exc:
            ok = False
            self.enter('ERROR', detail=str(exc))
        # ---- 收尾：张开爪子 + 自动归位（不管抓没抓到）----
        try:
            self.s_open()
        except BaseException as exc:
            print(f'  张开爪子失败: {exc}', flush=True)
        try:
            self.s_return_home()
        except BaseException as exc:
            self.enter('ERROR', detail=f'归位失败: {exc}')
        self.finish(ok)
        self.bus.close()


def capture_home():
    cfg = yaml.safe_load(Path(CONFIG).read_text())['so101_driver']['ros__parameters']
    zero, direction = np.array(cfg['zero_raw']), np.array(cfg['direction'])
    bus = FeetechSerialBus(default_arm_port(), timeout_s=0.08)
    raw = np.array(bus.read_positions())
    torque = bus.read_torque_states()
    bus.close()
    lo = np.minimum(np.array(cfg['raw_min']), np.array(cfg['raw_max']))
    hi = np.maximum(np.array(cfg['raw_min']), np.array(cfg['raw_max']))
    exec_raw = np.clip(raw, lo, hi)
    os.makedirs(CFG_DIR, exist_ok=True)
    json.dump({'raw': raw.tolist(), 'raw_exec': exec_raw.tolist(),
               'torque_at_capture': torque,
               'driver_lo': lo.tolist(), 'driver_hi': hi.tolist(),
               'note': 'baseline 折叠位：raw 为实测；raw_exec 为夹进驱动限位后'
                       '用于回位的可执行值'},
              open(HOME_PATH, 'w'), indent=2)
    print(f'baseline 已保存 → {HOME_PATH}')
    print(f'  实测 raw      {raw.tolist()}')
    print(f'  可执行 raw    {exec_raw.tolist()}')
    print(f'  夹位差(计数)  {(exec_raw - raw).tolist()}')
    print(f'  采集时力矩    {torque}')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--capture-home', action='store_true')
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float)
    ap.add_argument('--half-mm', type=float, default=20.0)
    ap.add_argument('--clearance-mm', type=float, default=6.0,
                    help='固定爪与方块侧面的外侧间隙（避免下探时蹭开方块）')
    ap.add_argument('--tip-z-mm', type=float, default=-46.0,
                    help='抓取时固定爪尖端高度（-46 = 离桌面约 19mm）')
    ap.add_argument('--hover-mm', type=float, default=50.0)
    ap.add_argument('--open-rad', type=float, default=0.58)
    ap.add_argument('--close-min-rad', type=float, default=0.16)
    ap.add_argument('--load-pct', type=float, default=8.0)
    ap.add_argument('--lift-mm', type=float, default=40.0)
    ap.add_argument('--report', default='/tmp/grasp_fsm.json')
    a = ap.parse_args()
    if a.capture_home:
        capture_home()
    else:
        if a.x is None or a.y is None:
            ap.error('需要 --x --y（或用 --capture-home）')
        FSM(a).run()
