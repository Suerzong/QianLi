#!/usr/bin/env python3
"""触标前端：画面标出"现在该打哪个点"，放好后按一下按钮即记录

按要求做成最简交互：
    * 画面上绿圈 = 现在把这个点对准
    * 页面上一个大按钮「放好了，记录这个点」
    * 按钮 = 写 /tmp/grid_mark（触标进程监听的触发文件）
    * 页面自动显示当前第几点 / 已记录几个，不用刷新

用法::

    ~/mj/bin/python touch_frontend.py --port 8101 --points "0,0;3.3,0;..."
"""

from __future__ import annotations

from project_paths import open_video_capture

from project_paths import calibration_path, camera_source, default_camera

import argparse
import os
import re
import threading
import time

import cv2
import numpy as np

COLS, ROWS = 7, 5
INTR = os.path.expanduser(calibration_path('camera_intrinsics.yaml'))
LOG = '/tmp/touch_calib.log'
TRIGGER = '/tmp/grid_mark'


class Prog:
    """从触标进程的日志里解析进度。

    刻意"读日志"而不是共享内存：触标工具是独立进程，不该为了给前端看进度
    去改它的接口。日志本来就是它给人看的输出。
    """

    def __init__(self, path=LOG):
        self.path = path
        self.lock = threading.Lock()
        self.d = dict(cur=0, total=0, done=0, note='')
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                txt = open(self.path, encoding='utf-8', errors='replace').read()
            except OSError:
                time.sleep(0.4)
                continue
            m = re.findall(r'第 (\d+)/(\d+) 点', txt)
            done = len(re.findall(r'✅ 接触点', txt))
            note = ''
            # 夹爪角不合格被拒的那一行，直接透出来，省得用户不知道为什么没记上
            for ln in reversed(txt.splitlines()):
                if '夹爪角' in ln and ('偏离' in ln or '⛔' in ln):
                    note = ln.strip()[:60]
                    break
            with self.lock:
                if m:
                    self.d['cur'], self.d['total'] = int(m[-1][0]), int(m[-1][1])
                self.d['done'] = done
                self.d['note'] = note
            time.sleep(0.4)

    def get(self):
        with self.lock:
            return dict(self.d)


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>SO-101 触标</title>
<style>
 body{margin:0;background:#101418;color:#e6edf3;
      font:16px/1.5 system-ui,"Segoe UI",sans-serif}
 .wrap{max-width:1000px;margin:0 auto;padding:14px}
 .head{display:flex;align-items:center;gap:16px;flex-wrap:wrap}
 .big{font-size:34px;font-weight:700}
 .big b{color:#3fb950}
 .sub{color:#8b949e}
 button{font-size:26px;font-weight:700;padding:18px 34px;border:0;
        border-radius:12px;background:#238636;color:#fff;cursor:pointer}
 button:hover{background:#2ea043}
 button:active{transform:translateY(1px)}
 button[disabled]{background:#30363d;color:#8b949e;cursor:default}
 #msg{margin-left:14px;color:#d29922;font-size:15px}
 img{width:100%;border-radius:10px;margin-top:12px;background:#000}
 .bar{color:#8b949e;font-size:14px;margin-top:6px}
</style></head><body><div class="wrap">
 <div class="head">
   <div>
     <div class="big">现在打第 <b id="cur">-</b> 个点</div>
     <div class="sub" id="sub">读取中…</div>
     <div id="lock" style="color:#d29922;font-size:14px"></div>
   </div>
   <div style="margin-left:auto;display:flex;align-items:center">
     <button id="go">放好了，记录这个点</button>
     <span id="msg"></span>
   </div>
 </div>
 <img id="v" src="/stream.mjpg">
 <div class="bar">画面上<b style="color:#3fb950">绿色大圈 + &gt;&gt;&gt; N &lt;&lt;&lt;</b>
  就是要对准的点。爪口开到约 60mm，用固定爪内侧面中心对准十字，轻碰板面，然后按按钮。
  <a href="#" id="rl" style="color:#58a6ff;margin-left:10px">棋盘没对齐？重新锁定</a></div>
</div>
<script>
const $=id=>document.getElementById(id);
async function tick(){
  try{
    const s=await (await fetch('/state.json',{cache:'no-store'})).json();
    $('cur').textContent = s.cur||'-';
    $('sub').textContent = '共 ' + (s.total||'?') + ' 个点，已记录 ' + s.done + ' 个';
    $('msg').textContent = s.note||'';
    if(!s.locked){
      $('lock').textContent = '⚠️ 棋盘未锁定：请让机械臂完全离开棋盘，看到绿色提示后再开始';
      $('go').disabled = true;
    }else{
      $('lock').textContent = '🔒 棋盘角点已锁定于 ' + s.lock_at;
      $('go').disabled = !s.cur || s.done>=s.total;
    }
    if(s.done>=s.total){$('go').textContent='全部完成';}
  }catch(e){}
}
$('go').onclick = async ()=>{
  $('go').disabled=true;
  try{
    const r=await (await fetch('/mark',{cache:'no-store'})).json();
    $('msg').textContent = r.ok ? '已发送记录指令…' : ('失败: '+r.err);
  }catch(e){ $('msg').textContent='请求失败'; }
  setTimeout(tick,900);
};
$('rl').onclick = async (e)=>{ e.preventDefault();
  await fetch('/relock',{cache:'no-store'}); tick(); };
setInterval(tick,700); tick();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8101)
    ap.add_argument('--camera', type=camera_source, default=default_camera())
    ap.add_argument('--points', required=True,
                    help='必须和触标工具 --points 一字不差')
    ap.add_argument('--intrinsics', default=INTR)
    ap.add_argument('--log', default=LOG)
    ap.add_argument('--cell-cm', type=float, default=3.3)
    args = ap.parse_args()

    pts = [tuple(float(v) for v in p.split(','))
           for p in args.points.split(';') if p.strip()]
    K = D = None
    size = [640, 480]
    mats, cur = [], None
    for ln in open(args.intrinsics, encoding='utf-8-sig'):
        s = ln.strip()
        if s.startswith('image_width:'):
            size[0] = int(float(s.split(':')[1]))
        elif s.startswith('image_height:'):
            size[1] = int(float(s.split(':')[1]))
        elif s.startswith('camera_matrix:'):
            cur = 'K'
        elif s.startswith('distortion_coefficients:'):
            cur = 'D'
        elif s.startswith('- [') and cur == 'K':
            mats.append([float(v)
                         for v in s[s.index('[') + 1:s.rindex(']')].split(',')])
        elif s.startswith('- [') and cur == 'D':
            D = np.array([float(v)
                          for v in s[s.index('[') + 1:s.rindex(']')].split(',')])
    K = np.array(mats)

    prog = Prog(args.log)
    st = {'jpg': None, 'lock': threading.Lock()}
    print(f'目标 {len(pts)} 个点，内参 {size[0]}x{size[1]}，触发文件 {TRIGGER}')

    cap = open_video_capture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    time.sleep(1.0)

    def label(img, text, org, color, scale=0.55, thick=1):
        (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX,
                                       scale, thick)
        x, y = org
        cv2.rectangle(img, (x - 3, y - h - 4), (x + w + 3, y + base + 2),
                      (0, 0, 0), -1)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, thick, cv2.LINE_AA)

    frozen = {'C': None, 'at': None, 'lock': threading.Lock()}

    def worker():
        last_ok = True
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            und = cv2.undistort(frame, K, D, None, K)
            gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
            okc, corners = cv2.findChessboardCorners(
                gray, (COLS, ROWS),
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            with frozen['lock']:
                have = frozen['C'] is not None
                # 只在**还没锁定**时才去找棋盘。
                #
                # 为什么必须"锁死"而不是每帧重检：打点过程中机械臂就压在棋盘
                # 上方，爪子/腕部必然遮住棋盘一角，而 findChessboardCorners 要求
                # **整块棋盘可见**，一被遮挡就整个失败。若退回用"上一次的角点"，
                # 那位置来自机械臂姿态不同的另一时刻，画出来的圈就会漂、与用户
                # 看到的棋盘对不上 —— 这正是现场看到的 bug。
                #
                # 棋盘和相机都不动，所以角点像素位置是**恒定**的：开始前让机械臂
                # 让开、检一次，之后一直用这套就行。
                if not have and okc:
                    corners = cv2.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1),
                        (cv2.TERM_CRITERIA_EPS
                         + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    frozen['C'] = corners.reshape(ROWS, COLS, 2).copy()
                    frozen['at'] = time.strftime('%H:%M:%S')
                    print(f'🔒 棋盘角点已锁定 @ {frozen["at"]}'
                          f'（之后机械臂遮挡也不影响）', flush=True)
                C = frozen['C']
            last_ok = okc
            p = prog.get()
            vis = und.copy()
            if C is not None:
                for i, (gx, gy) in enumerate(pts, 1):
                    cc, rr = gx / args.cell_cm, gy / args.cell_cm
                    c0, r0 = int(np.floor(cc)), int(np.floor(rr))
                    fc, fr = cc - c0, rr - r0
                    if c0 + 1 >= COLS or r0 + 1 >= ROWS:
                        continue
                    xy = ((1 - fc) * (1 - fr) * C[r0, c0]
                          + fc * (1 - fr) * C[r0, c0 + 1]
                          + (1 - fc) * fr * C[r0 + 1, c0]
                          + fc * fr * C[r0 + 1, c0 + 1])
                    x, y = int(xy[0]), int(xy[1])
                    is_cur = (i == p['cur'])
                    is_done = (i <= p['done'])
                    if is_cur:
                        # 当前点：大绿圈 + **引线把标签拉到画面左侧空白处**。
                        # 为什么用引线而不是标签贴圈：棋盘格间距在这里只有
                        # ~33px，而圈半径就得 16px 以上才看得清，相邻圈几乎
                        # 贴在一起 —— 编号放圈的上下左右都会撞到隔壁的圈，
                        # 看起来就像"编号对错了圈"（现场就是这么被误认为 bug）。
                        # 拉到空白处 + 引线，指向哪个圈毫无歧义。
                        cv2.circle(vis, (x, y), 24, (0, 255, 0), 4)
                        cv2.drawMarker(vis, (x, y), (0, 255, 0),
                                       cv2.MARKER_CROSS, 18, 2)
                        cv2.line(vis, (x - 24, y), (118, y), (0, 255, 0), 2)
                        label(vis, f'>>> {i}/{p["total"]} <<<', (8, y + 7),
                              (0, 255, 0), 0.62, 2)
                    else:
                        col = (0, 170, 255) if is_done else (0, 0, 255)
                        cv2.circle(vis, (x, y), 5, col, -1)
                        cv2.circle(vis, (x, y), 9, col, 1)
            cv2.rectangle(vis, (0, 0), (vis.shape[1], 28), (0, 0, 0), -1)
            if frozen['C'] is None:
                head = ('⚠️ 棋盘还没锁定 —— 请让机械臂完全离开棋盘，'
                        '看到"已锁定"再开始')
                hcol = (0, 165, 255)
            else:
                head = (f'第 {p["cur"]}/{p["total"]} 点   已记录 {p["done"]} 个   '
                        f'🔒 角点锁定于 {frozen["at"]}')
                hcol = (0, 255, 0)
            cv2.putText(vis, head, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        hcol, 1, cv2.LINE_AA)
            if not last_ok and frozen['C'] is not None:
                cv2.putText(vis, '(当前帧棋盘被遮挡，圈用的是锁定坐标，正常)',
                            (8, vis.shape[0] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150),
                            1, cv2.LINE_AA)
            okj, jpg = cv2.imencode('.jpg', vis,
                                    [cv2.IMWRITE_JPEG_QUALITY, 80])
            if okj:
                with st['lock']:
                    st['jpg'] = jpg.tobytes()

    threading.Thread(target=worker, daemon=True).start()

    import json
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path in ('/', '/index.html'):
                self._send(PAGE.encode('utf-8'), 'text/html; charset=utf-8')
                return
            if path == '/state.json':
                d = prog.get()
                with frozen['lock']:
                    d['locked'] = frozen['C'] is not None
                    d['lock_at'] = frozen['at'] or ''
                self._send(json.dumps(d).encode(), 'application/json')
                return
            if path == '/relock':
                # 重新锁一次角点（万一锁的时候机械臂还挡着棋盘）。
                # 必须让机械臂先离开棋盘，再点这个。
                with frozen['lock']:
                    frozen['C'] = None
                    frozen['at'] = None
                print('🔓 已解锁，等待重新检测棋盘角点', flush=True)
                self._send(json.dumps({'ok': True}).encode(),
                           'application/json')
                return
            if path == '/mark':
                # 按钮就是写这个触发文件 —— 触标进程在轮询它
                try:
                    open(TRIGGER, 'w').close()
                    self._send(json.dumps({'ok': True}).encode(),
                               'application/json')
                except OSError as exc:
                    self._send(json.dumps({'ok': False, 'err': str(exc)}
                                          ).encode(), 'application/json')
                return
            if path.startswith('/stream.mjpg'):
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        with st['lock']:
                            jpg = st['jpg']
                        if jpg:
                            self.wfile.write(b'--frame\r\n'
                                             b'Content-Type: image/jpeg\r\n')
                            self.wfile.write(
                                f'Content-Length: {len(jpg)}\r\n\r\n'.encode())
                            self.wfile.write(jpg + b'\r\n')
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            self.send_error(404)

    srv = ThreadingHTTPServer(('0.0.0.0', args.port), H)
    print(f'触标前端: http://0.0.0.0:{args.port}   '
          f'（打开后按「放好了，记录这个点」即可）')
    srv.serve_forever()


if __name__ == '__main__':
    main()
