#!/usr/bin/env python3
"""实时物块识别网页前端

为什么要有这个
--------------
命令行版 `color_block_detect.py --ros` 只能发话题，看不到画面。
调阈值（chroma-min / fill-min / hue-tol）时最需要的是**实时看到**：
掩码里有没有料、框有没有套上、颜色判得对不对。

复用而不是重写
--------------
检测逻辑全部复用 `color_block_detect.py` 里已经自检通过的函数
（detect_blocks / annotate / Undistorter / parse_color_table），
这个文件只负责"相机循环 + 网页 + MJPEG 推流"。

用法::

    ~/mj/bin/python block_live_gui.py --port 8097
    浏览器打开 http://<虚拟机IP>:8097
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import color_block_detect as cd  # noqa: E402

STATE: dict = {}
LOCK = threading.Lock()
CONFIG: dict = {}
ROS: dict = {}


def _init_ros():
    """可选：把标注图发成 ROS 图像话题，好在 RViz 的 Image 面板里看。

    为什么要走 ROS 而不是开浏览器：VM 上的 firefox/chromium 都是 snap 包，
    snap 沙箱读不到 Xwayland 的授权文件（/run/user/1000/.mutter-Xwaylandauth.*），
    直接报 "cannot open display :0"。而 RViz 是 apt 装的、能正常显示。
    把识别画面塞进 RViz，用户就能在**同一个窗口**里同时看到机械臂姿态和识别结果。
    """
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
    except Exception as exc:  # noqa: BLE001
        print(f'  未启用 ROS 发布（{exc}）')
        return
    rclpy.init()
    node = Node('block_live_gui')
    ROS['node'] = node
    ROS['Image'] = Image
    ROS['pub_img'] = node.create_publisher(Image, '/blocks_annotated', 2)
    ROS['pub_json'] = node.create_publisher(String, '/blocks', 10)
    # 订阅关节角：把爪口开度叠在画面上。
    # 为什么需要：触标要求"爪口开到 40mm 再对准"，但用户看不到实时角度，
    # 而触标脚本会因为这个直接拒绝记录（实测被拦过一次：5.4° vs 期望 19.5°）。
    # 用户本来就盯着这个相机画面在把爪子对到格点上，读数放这儿最顺手。
    from sensor_msgs.msg import JointState
    ROS['js'] = {}

    def _on_js(m):
        if len(m.name) == len(m.position):
            ROS['js'].update(dict(zip(m.name, m.position)))

    node.create_subscription(JointState, '/joint_states', _on_js, 10)
    # **必须 spin**，否则订阅回调永远不触发 —— 只发布不 spin 时，
    # /joint_states 会一直是空的，叠加读数静默消失（实测踩到）。
    import threading as _th
    ROS['spin_stop'] = False

    def _spin():
        while rclpy.ok() and not ROS.get('spin_stop'):
            rclpy.spin_once(node, timeout_sec=0.1)

    _th.Thread(target=_spin, daemon=True).start()
    print('  ROS 发布已启用：/blocks_annotated (Image) + /blocks (String JSON)')
    print('  已订阅 /joint_states：会在画面上叠加爪口开度')


def _overlay_gripper(img):
    """在画面左上角画爪口开度，并标出触标目标（40mm / 19.5°）。"""
    js = ROS.get('js') or {}
    if 'gripper' not in js:
        return img
    ang = float(js['gripper'])
    import math
    deg = math.degrees(ang)
    open_mm = None
    try:
        gm = ROS.get('gmodel')
        if gm is None:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from gripper_model import GripperModel
            gm = GripperModel(stride=24)
            ROS['gmodel'] = gm
        open_mm = gm.jaw_opening(ang)
    except Exception:  # noqa: BLE001
        pass

    # 触标目标：TCP 已改成**固定爪内侧面中心**，不再要求开到 40mm。
    # 开大到 ~60mm(33°) 时固定爪才是最低点，爪尖几乎正好在固定爪内侧面的
    # 正下方（水平只偏 1mm）。若开到 40mm(19.5°)，最低的是活动爪、偏 44.5mm，
    # 会在棋盘外面碰板。所以这里的目标是 33° 而不是 19.5°。
    tgt_deg = CONFIG.get('target_gripper_deg', 33.0)
    tol_deg = CONFIG.get('target_gripper_tol_deg', 10.0)
    ok = abs(deg - tgt_deg) <= tol_deg
    col = (0, 220, 0) if ok else (0, 165, 255)   # BGR
    lines = [f'gripper {deg:+6.1f} deg']
    if open_mm is not None:
        lines.append(f'opening {open_mm*1000:5.1f} mm')
    lines.append(f'touch target {tgt_deg:.0f} deg / ~60 mm')
    lines.append('OK  (fixed jaw is lowest)' if ok else 'OPEN WIDER')
    y = 22
    for i, t in enumerate(lines):
        scale = 0.62 if i < 2 else 0.5
        th = 2 if i < 2 else 1
        cv2.putText(img, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), th + 2)
        cv2.putText(img, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, th)
        y += 22
    return img


def _publish(img_bgr, blocks):
    if not ROS:
        return
    Image, node = ROS['Image'], ROS['node']
    h, w = img_bgr.shape[:2]
    msg = Image()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'camera'
    msg.height, msg.width = h, w
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = w * 3
    msg.data = img_bgr.tobytes()
    ROS['pub_img'].publish(msg)
    from std_msgs.msg import String
    js = String()
    js.data = json.dumps({'stamp': msg.header.stamp.sec
                          + msg.header.stamp.nanosec * 1e-9,
                          'blocks': [{k: v for k, v in b.items()
                                      if k in ('color', 'cx', 'cy',
                                               'angle_deg', 'size_px',
                                               'confidence')}
                                     for b in blocks]}, ensure_ascii=False)
    ROS['pub_json'].publish(js)


def _build_args():
    """给 detect_blocks 造一个 args。它内部会读这些字段，所以必须齐全。"""
    argv = ['--image', 'dummy']          # 只为绕过 parse_args 的模式校验
    argv += ['--chroma-min', str(CONFIG['chroma_min'])]
    argv += ['--hue-tol', str(CONFIG['hue_tol'])]
    argv += ['--hue-consistency-min', str(CONFIG['hue_consistency_min'])]
    argv += ['--min-confidence', str(CONFIG['min_confidence'])]
    argv += ['--fill-min', str(CONFIG['fill_min'])]
    argv += ['--solidity-min', str(CONFIG['solidity_min'])]
    argv += ['--aspect-max', str(CONFIG['aspect_max'])]
    argv += ['--vertex-max', str(CONFIG['vertex_max'])]
    argv += ['--colors', CONFIG['colors']]
    if CONFIG['intrinsics']:
        argv += ['--intrinsics', CONFIG['intrinsics']]
    a = cd.parse_args(argv)
    return a


def reader_thread():
    args = _build_args()
    # 注意符号名是 DEFAULT_HUE_CENTERS（不是 HUE_CENTERS）。
    # 颜色表里凡是用 "标签:数值" 写死色相的项不依赖它；只有不带数值的标签才需要。
    hue_centers = getattr(cd, 'DEFAULT_HUE_CENTERS', {})
    try:
        color_table = cd.parse_color_table(CONFIG['colors'], hue_centers)
    except Exception as exc:  # noqa: BLE001
        with LOCK:
            STATE['message'] = f'颜色表解析失败: {exc}'
        return
    undist = cd.Undistorter(CONFIG['intrinsics'], log_fn=lambda *a, **k: None)

    cap = cv2.VideoCapture(CONFIG['camera'])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CONFIG['width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CONFIG['height'])
    time.sleep(1.0)
    with LOCK:
        STATE['camera_ok'] = cap.isOpened()
        STATE['undistort'] = bool(getattr(undist, 'enabled', True))
    if not cap.isOpened():
        with LOCK:
            STATE['message'] = f'❌ 打不开相机 {CONFIG["camera"]}（可能被占用）'
        return

    n = 0
    t_last = time.time()
    fps = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        frame = undist.apply(frame)
        blocks, mask, info = cd.detect_blocks(frame, args, color_table,
                                              hue_centers)
        vis = cd.annotate(frame, blocks, mask, info)
        if CONFIG.get('ros_publish'):
            vis = _overlay_gripper(vis)
        n += 1
        now = time.time()
        if now - t_last >= 1.0:
            fps = n / (now - t_last)
            n = 0
            t_last = now
        with LOCK:
            STATE['frame'] = vis
            STATE['mask'] = mask
            STATE['blocks'] = [
                {k: v for k, v in b.items()
                 if k in ('color', 'cx', 'cy', 'angle_deg', 'size_px',
                          'confidence', 'mean_hue', 'fill', 'solidity',
                          'vertex_count', 'area')}
                for b in blocks]
            STATE['info'] = {
                'chroma_threshold': info.get('chroma_threshold'),
                'colorful_pixels': info.get('colorful_pixels'),
                'candidates': info.get('candidates'),
                'area_range': info.get('area_range'),
                'rejected': [
                    {k: (round(v, 3) if isinstance(v, float) else v)
                     for k, v in r.items() if k != 'rect'}
                    for r in info.get('rejected', [])][:5],
            }
            STATE['fps'] = fps
            STATE['frames'] = STATE.get('frames', 0) + 1
        if CONFIG.get('ros_publish'):
            try:
                _publish(vis, blocks)
            except Exception:  # noqa: BLE001
                pass
        time.sleep(0.01)


PAGE = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>实时物块识别</title><style>
body{font-family:system-ui,"Microsoft YaHei",sans-serif;margin:0;background:#1b1e24;color:#e6e8ec}
.wrap{display:flex;gap:16px;padding:16px}
img{width:760px;border:1px solid #333;border-radius:6px;background:#000}
.panel{flex:1;min-width:360px}
h3{margin:4px 0}
table{border-collapse:collapse;font-size:13px;width:100%}
td,th{border:1px solid #333;padding:4px 7px;text-align:left}
.sw{display:inline-block;width:12px;height:12px;border-radius:3px;
 margin-right:6px;vertical-align:middle;border:1px solid #555}
pre{background:#12151a;padding:10px;border-radius:6px;font-size:12px;
 overflow:auto;max-height:300px}
.k{display:inline-block;padding:3px 8px;border-radius:4px;font-weight:600;margin-right:6px}
.ok{background:#1f9d55}.no{background:#b03030}.warn{background:#b58900;color:#111}
</style></head><body><div class="wrap">
<div><img src="/stream.mjpg" alt="stream">
<div id="msg" style="margin-top:8px;color:#8fd">　</div></div>
<div class="panel">
<h3>实时物块识别</h3>
<div id="head"></div>
<table id="tbl"><thead><tr>
<th>颜色</th><th>质心(px)</th><th>朝向</th><th>边长</th><th>置信度</th><th>色相</th>
</tr></thead><tbody></tbody></table>
<h3 style="margin-top:14px">掩码（排错先看这里）</h3>
<img src="/mask.jpg" style="width:100%;border:1px solid #333;border-radius:4px">
<h3 style="margin-top:14px">诊断</h3>
<pre id="diag">…</pre>
</div></div>
<script>
const COL={red:'#d33',yellow:'#dc3',green:'#2a3',purple:'#93d',blue:'#36c',
           cyan:'#3cc',orange:'#e83'};
async function refresh(){
  try{
    const s=await (await fetch('/state')).json();
    let h='';
    h+=(s.camera_ok?'<span class="k ok">相机 OK</span>':'<span class="k no">相机未开</span>');
    h+=(s.undistort?'<span class="k ok">已去畸变</span>':'<span class="k warn">未去畸变</span>');
    h+='<span class="k warn">'+s.blocks.length+' 块</span>';
    h+='<span class="k warn">'+ (s.fps||0).toFixed(1) +' fps</span>';
    document.getElementById('head').innerHTML=h;
    if(s.message)document.getElementById('msg').textContent=s.message;
    const tb=document.querySelector('#tbl tbody');tb.innerHTML='';
    for(const b of s.blocks){
      const c=COL[b.color]||'#888';
      tb.innerHTML+='<tr><td><span class="sw" style="background:'+c+'"></span>'
        +b.color+'</td><td>'+b.cx.toFixed(1)+', '+b.cy.toFixed(1)+'</td><td>'
        +b.angle_deg.toFixed(1)+'°</td><td>'+b.size_px.toFixed(1)+'</td><td>'
        +b.confidence.toFixed(2)+'</td><td>'+(b.mean_hue||0).toFixed(1)+'</td></tr>';
    }
    if(!s.blocks.length)tb.innerHTML='<tr><td colspan="6">未识别到物块</td></tr>';
    document.getElementById('diag').textContent=JSON.stringify(s.info,null,1);
  }catch(e){}
}
setInterval(refresh,400);refresh();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        p = self.path
        if p.startswith('/stream.mjpg') or p.startswith('/mask.jpg'):
            key = 'frame' if p.startswith('/stream') else 'mask'
            is_stream = p.startswith('/stream')
            if is_stream:
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=f')
                self.end_headers()
            while True:
                with LOCK:
                    f = STATE.get(key)
                if f is None:
                    f = np.full((CONFIG['height'], CONFIG['width'], 3), 40,
                                np.uint8)
                if f.ndim == 2:
                    f = cv2.cvtColor(f, cv2.COLOR_GRAY2BGR)
                ok, jpg = cv2.imencode('.jpg', f,
                                       [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    time.sleep(0.05)
                    continue
                if not is_stream:
                    return self._send(jpg.tobytes(), 'image/jpeg')
                try:
                    self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' +
                                     str(len(jpg)).encode() + b'\r\n\r\n')
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    return
                time.sleep(1.0 / max(1.0, CONFIG['rate']))
            return
        if p.startswith('/state'):
            with LOCK:
                s = {'camera_ok': STATE.get('camera_ok'),
                     'undistort': STATE.get('undistort'),
                     'blocks': STATE.get('blocks', []),
                     'info': STATE.get('info', {}),
                     'fps': STATE.get('fps', 0.0),
                     'message': STATE.get('message', '')}
            return self._send(json.dumps(s, ensure_ascii=False).encode(),
                              'application/json; charset=utf-8')
        return self._send(PAGE.encode(), 'text/html; charset=utf-8')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8097)
    ap.add_argument('--camera', type=int, default=0)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--intrinsics', default='/tmp/camera_intrinsics.yaml')
    ap.add_argument('--colors', default=None,
                    help='颜色表。**不给就用 color_block_detect 的默认表** —— '
                         '刻意做成单一来源：之前这里和检测器各写一份，'
                         '重启前端时用回旧值把已经修好的 purple 又覆盖回 150，'
                         '导致紫块静默消失。')
    ap.add_argument('--chroma-min', type=float, default=26)
    ap.add_argument('--hue-tol', type=float, default=8)
    ap.add_argument('--hue-consistency-min', type=float, default=0.55)
    ap.add_argument('--min-confidence', type=float, default=0.35)
    ap.add_argument('--fill-min', type=float, default=0.70)
    ap.add_argument('--solidity-min', type=float, default=0.80)
    ap.add_argument('--aspect-max', type=float, default=1.8)
    ap.add_argument('--vertex-max', type=int, default=6)
    ap.add_argument('--ros-publish', action='store_true', default=True,
                    help='把标注图发到 /blocks_annotated，好在 RViz 里看')
    ap.add_argument('--no-ros-publish', dest='ros_publish',
                    action='store_false')
    args = ap.parse_args()

    # 颜色表单一来源：没显式给就照搬检测器的默认表（它带着"这台相机这套
    # 灯光下实测"的注释）。避免前端和检测器各存一份、其中一份过期。
    if not args.colors:
        args.colors = ','.join(getattr(cd, 'DEFAULT_COLOR_TABLE', ()))
        print(f'  颜色表取自 color_block_detect.DEFAULT_COLOR_TABLE: '
              f'{args.colors}')

    CONFIG.update(vars(args))
    CONFIG['width'] = args.width
    CONFIG['height'] = args.height
    STATE.update({'frame': None, 'mask': None, 'blocks': [], 'info': {},
                  'fps': 0.0, 'camera_ok': None, 'message': ''})

    if args.ros_publish:
        _init_ros()
    threading.Thread(target=reader_thread, daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    print(f'实时物块识别前端: http://0.0.0.0:{args.port}')
    print(f'  相机 {args.camera}  {args.width}x{args.height}')
    print(f'  内参 {args.intrinsics}')
    print(f'  颜色表 {args.colors}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
