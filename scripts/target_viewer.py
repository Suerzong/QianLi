#!/usr/bin/env python3
"""目标点可视化前端（网页）。

画面叠加三样东西，用于核对"图像定位 → 机械臂目标点"是否正确：
  1. 相机检测到的色块（框 + 棋盘坐标 + base 坐标）
  2. 算出的**目标点**（固定爪应去的位置）—— 反投影回画面画绿十字
  3. 机械臂**实际爪尖**（读舵机 → 正运动学）—— 反投影画红十字

若绿十字（目标）与红十字（实际）不重合，差值就是"图像↔机械臂"换算误差。

用法: ~/mj/bin/python target_viewer.py --port 8099
     浏览器打开 http://<VM_IP>:8099
"""

from project_paths import default_arm_port

from project_paths import arm_source_path, default_camera, driver_params_path, project_path
import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

sys.path.insert(0, arm_source_path())
from so101_bringup.servo_protocol import FeetechSerialBus
from gripper_model import GripperModel, JOINTS, FLANGE_LINK

CFG_DIR = os.path.expanduser(project_path('config'))
FRAME_JSON = os.path.join(CFG_DIR, 'board_frame.json')
DRIVER = os.path.expanduser(
    driver_params_path())
TABLE_Z = -0.06485

SPEC = {'blue': [((95, 90, 60), (135, 255, 255))],
        'green': [((35, 80, 60), (85, 255, 255))],
        'yellow': [((20, 90, 90), (34, 255, 255))],
        'red': [((0, 100, 70), (8, 255, 255)), ((170, 100, 70), (179, 255, 255))],
        'purple': [((136, 60, 60), (168, 255, 255))]}
BGR = {'blue': (255, 80, 0), 'green': (0, 200, 0), 'yellow': (0, 215, 255),
       'red': (0, 0, 255), 'purple': (200, 0, 200)}

STATE = {'jpeg': None, 'info': {}, 'lock': threading.Lock()}


class Vision:
    def __init__(self, half_mm, clearance_mm, color):
        self.half = half_mm / 1000.0
        self.gap = clearance_mm / 1000.0
        self.color = color
        fr = json.load(open(FRAME_JSON))
        self.H = np.array(fr['H'])
        self.Hi = np.linalg.inv(self.H)
        if fr.get('affine'):
            A = np.array(fr['affine'])
            self.A = A[:2].T
            self.b = A[2]
        else:
            self.A = np.eye(2)
            self.b = np.zeros(2)
        self.Ai = np.linalg.inv(self.A)
        v = self.A[:, 1]
        self.rd = np.array([v[0], v[1], 0.0])
        self.rd /= np.linalg.norm(self.rd)
        # 舵机/模型
        import yaml
        from pathlib import Path
        cfg = yaml.safe_load(Path(DRIVER).read_text())['so101_driver']['ros__parameters']
        self.zero = np.array(cfg['zero_raw'])
        self.dir = np.array(cfg['direction'])
        self.model = GripperModel(stride=8)
        T0 = self.model.solve(dict(zip(JOINTS, np.zeros(6))))
        F0, G0 = T0['gripper_frame_link'], T0[FLANGE_LINK]
        w0 = (G0[:3, :3] @ self.model.parts[FLANGE_LINK].T).T + G0[:3, 3]
        pf = (F0[:3, :3].T @ (w0 - F0[:3, 3]).T).T
        inner = pf[np.abs(pf[:, 0]) < 0.004]
        self.p_fix = inner[int(np.argmax(inner[:, 2]))]
        self.bus = None
        try:
            self.bus = FeetechSerialBus(default_arm_port(), timeout_s=0.08)
        except Exception as exc:
            print('串口打开失败:', exc)

    def px2grid(self, px):
        v = self.H @ np.array([px[0], px[1], 1.0])
        return v[:2] / v[2]

    def grid2base(self, g_mm):
        g_m = np.array(g_mm) / 1000.0
        return self.A @ g_m + self.b

    def base2grid(self, b_m):
        return self.Ai @ (np.array(b_m) - self.b) * 1000.0

    def grid2px(self, g_mm):
        v = self.Hi @ np.array([g_mm[0], g_mm[1], 1.0])
        return v[:2] / v[2]

    def base2px(self, b_m):
        return self.grid2px(self.base2grid(b_m))

    def arm_tip(self):
        if self.bus is None:
            return None
        try:
            raw = np.array(self.bus.read_positions())
        except Exception:
            return None
        q = (raw - self.zero) * self.dir * 2 * math.pi / 4096
        T = self.model.solve(dict(zip(JOINTS, q)))
        F = T['gripper_frame_link']
        return F[:3, 3] + F[:3, :3] @ self.p_fix

    def frame(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blobs = []
        for name, ranges in SPEC.items():
            mask = None
            for lo_, hi_ in ranges:
                m = cv2.inRange(hsv, np.array(lo_), np.array(hi_))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                    np.ones((7, 7), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                a = cv2.contourArea(c)
                if a < 350:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                if not 0.6 <= w / float(max(h, 1)) <= 1.7:
                    continue
                cx, cy = x + w / 2, y + h / 2
                g = self.px2grid((cx, cy))
                b = self.grid2base(g)
                blobs.append({'color': name, 'px': [cx, cy], 'wh': [w, h],
                              'area': a, 'grid_mm': g.tolist(),
                              'base_m': b.tolist(),
                              'radius_mm': float(np.linalg.norm(b) * 1000)})
        blobs.sort(key=lambda d: -d['area'])
        info = {'blobs': blobs, 'table_z_mm': TABLE_Z * 1000,
                'arm': None, 'target': None}
        # 选目标：指定颜色里"在棋盘上/最像方块"的那个
        cand = [b for b in blobs if b['color'] == self.color]
        pick = None
        for b in cand:
            gx, gy = np.array(b['grid_mm']) / 33.0
            on_board = -0.6 <= gx <= 7.6 and -0.6 <= gy <= 5.6
            if on_board or pick is None:
                pick = b
                if on_board:
                    break
        for b in blobs:
            x, y, w, h = int(b['px'][0] - b['wh'][0] / 2), \
                int(b['px'][1] - b['wh'][1] / 2), b['wh'][0], b['wh'][1]
            is_pick = (b is pick)
            cv2.rectangle(img, (x, y), (x + w, y + h),
                          BGR[b['color']], 3 if is_pick else 1)
            txt = (f"{b['color']} g({b['grid_mm'][0]/10:+.1f},"
                   f"{b['grid_mm'][1]/10:+.1f})cm "
                   f"b({b['base_m'][0]:.3f},{b['base_m'][1]:.3f})")
            cv2.putText(img, txt, (x, max(12, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        BGR[b['color']], 1, cv2.LINE_AA)
        if pick is not None:
            pb = np.array(pick['base_m'])
            tgt = pb + (self.half + self.gap) * self.rd[:2]
            info['target'] = {'base_m': tgt.tolist(),
                              'grid_mm': self.base2grid(tgt).tolist(),
                              'source_blob': pick}
            tp = self.base2px(tgt)
            cv2.drawMarker(img, (int(tp[0]), int(tp[1])), (0, 255, 0),
                           cv2.MARKER_CROSS, 26, 2)
            cv2.putText(img, 'TARGET', (int(tp[0]) + 10, int(tp[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        tip = self.arm_tip()
        if tip is not None:
            info['arm'] = {'tip_m': tip.tolist()}
            ap = self.base2px(tip[:2])
            cv2.drawMarker(img, (int(ap[0]), int(ap[1])), (0, 0, 255),
                           cv2.MARKER_TILTED_CROSS, 26, 2)
            cv2.putText(img, 'ARM', (int(ap[0]) + 10, int(ap[1]) + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            if info.get('target'):
                d = np.linalg.norm(tip[:2] - np.array(info['target']['base_m']))
                info['target_vs_arm_mm'] = float(d * 1000)
        cv2.rectangle(img, (0, 0), (img.shape[1], 22), (40, 40, 40), -1)
        cv2.putText(img, 'green=TARGET(where arm should go)  '
                         'red=ARM(actual tip, FK)',
                    (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return img, info


PAGE = """<!doctype html><meta charset=utf-8>
<title>目标点核对</title>
<style>
body{background:#181a1f;color:#ddd;font:13px/1.5 monospace;margin:0;padding:12px}
img{max-width:100%;border:1px solid #444}
table{border-collapse:collapse;margin-top:10px}
td,th{border:1px solid #444;padding:3px 7px;text-align:right}
th{background:#262a33}
.warn{color:#ff8080}.ok{color:#80ff80}
h3{margin:12px 0 4px}
</style>
<h3>相机画面（绿=目标点，红=机械臂实际爪尖）</h3>
<img id=im src="/stream">
<h3>检测详情</h3>
<table id=t></table>
<script>
async function tick(){
  const r = await fetch('/data'); const d = await r.json();
  let h = '<tr><th>颜色</th><th>像素</th><th>边长px</th>'
        + '<th>棋盘cm</th><th>base m</th><th>半径mm</th></tr>';
  for(const b of d.blobs){
    h += `<tr><td>${b.color}</td><td>${b.px[0].toFixed(0)},${b.px[1].toFixed(0)}</td>`
       + `<td>${Math.max(b.wh[0],b.wh[1]).toFixed(0)}</td>`
       + `<td>${(b.grid_mm[0]/10).toFixed(2)}, ${(b.grid_mm[1]/10).toFixed(2)}</td>`
       + `<td>${b.base_m[0].toFixed(4)}, ${b.base_m[1].toFixed(4)}</td>`
       + `<td>${b.radius_mm.toFixed(0)}</td></tr>`;
  }
  h += '</table>';
  if(d.target){
    h += `<p>目标点 base = (${d.target.base_m[0].toFixed(4)}, `
       + `${d.target.base_m[1].toFixed(4)}) m<br>`
       + `目标点 棋盘 = (${(d.target.grid_mm[0]/10).toFixed(2)}, `
       + `${(d.target.grid_mm[1]/10).toFixed(2)}) cm</p>`;
  }
  if(d.arm){
    h += `<p>机械臂爪尖 base = (${d.arm.tip_m[0].toFixed(4)}, `
       + `${d.arm.tip_m[1].toFixed(4)}, ${(d.arm.tip_m[2]*1000).toFixed(1)}mm)</p>`;
  }
  if(d.target_vs_arm_mm!==undefined){
    const v=d.target_vs_arm_mm;
    h += `<p class="${v<10?'ok':'warn'}">目标点 与 实际爪尖 水平差 `
       + `<b>${v.toFixed(1)} mm</b> ${v<10?'✅ 吻合':'❌ 偏了'}</p>`;
  }
  document.getElementById('t').innerHTML = h;
  setTimeout(tick, 500);
}
tick();
</script>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8099)
    ap.add_argument('--half-mm', type=float, default=20.0)
    ap.add_argument('--clearance-mm', type=float, default=6.0)
    ap.add_argument('--color', default='yellow')
    a = ap.parse_args()
    vis = Vision(a.half_mm, a.clearance_mm, a.color)

    def loop():
        cap = cv2.VideoCapture(default_camera())
        while True:
            ok, img = cap.read()
            if not ok:
                time.sleep(0.2)
                continue
            img, info = vis.frame(img)
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            with STATE['lock']:
                STATE['jpeg'] = buf.tobytes()
                STATE['info'] = info
            time.sleep(0.05)

    threading.Thread(target=loop, daemon=True).start()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == '/':
                body = PAGE.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == '/data':
                with STATE['lock']:
                    body = json.dumps(STATE['info']).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == '/snap':
                with STATE['lock']:
                    j = STATE['jpeg']
                if j is None:
                    self.send_response(503)
                    self.end_headers()
                else:
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(j)))
                    self.end_headers()
                    self.wfile.write(j)
            elif self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=f')
                self.end_headers()
                while True:
                    with STATE['lock']:
                        j = STATE['jpeg']
                    if j is None:
                        time.sleep(0.1)
                        continue
                    try:
                        self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                         b'Content-Length: '
                                         + str(len(j)).encode() + b'\r\n\r\n')
                        self.wfile.write(j)
                        self.wfile.write(b'\r\n')
                    except Exception:
                        break
                    time.sleep(0.05)
            else:
                self.send_response(404)
                self.end_headers()

    srv = ThreadingHTTPServer(('0.0.0.0', a.port), H)
    print(f'前端已启动: http://<VM_IP>:{a.port}  (标定源 {FRAME_JSON})', flush=True)
    srv.serve_forever()


if __name__ == '__main__':
    main()
