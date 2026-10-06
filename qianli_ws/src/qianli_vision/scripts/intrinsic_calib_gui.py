#!/usr/bin/env python3
"""相机内参标定网页前端（棋盘格，含畸变）

为什么必须先做这个
------------------
现有管线是"每帧用棋盘角点拟合一个 Homography"来做像素→物理映射。
但**畸变不是单应变换，单应吸收不了它** —— 角点本身是弯的，拟合出的 H
在物块所在位置就是错的，离棋盘中心越远误差越大。

标定内参后每帧先 undistort，再做单应/外参，边缘误差才能压下去。

验证指标：直线度
----------------
针孔模型下，世界里的一条直线在图像上仍是直线，畸变会把它弯掉。
所以对棋盘每一行/每一列角点拟合直线、量垂直偏差 RMS。
去畸变后这个数应显著下降 —— 这比只看 calibrateCamera 的 RMS 更能说明问题。

用法::

    ~/mj/bin/python intrinsic_calib_gui.py --port 8098 --cols 7 --rows 5 --cell-mm 33
    浏览器打开 http://<虚拟机IP>:8098
"""

from __future__ import annotations

import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

STATE: dict = {}
LOCK = threading.Lock()
CONFIG: dict = {}

SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
CB_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)


# ---------------------------------------------------------------- 几何指标
def line_rms_dev(pts: np.ndarray) -> float:
    """一组点相对其最佳拟合直线的垂直偏差 RMS（像素）。

    理想针孔成像下，世界直线上的点投影后仍共线，这个值应接近 0；
    径向畸变会把直线弯成曲线，于是这个值变大。
    """
    if len(pts) < 3:
        return 0.0
    p = np.asarray(pts, float)
    c = p.mean(axis=0)
    # 主方向 = 协方差矩阵最大特征向量
    u, s, vt = np.linalg.svd(p - c)
    d = vt[0]
    n = np.array([-d[1], d[0]])
    return float(np.sqrt(np.mean(((p - c) @ n) ** 2)))


def grid_straightness(corners: np.ndarray, cols: int, rows: int):
    """棋盘行/列角点的直线度。返回 (行 RMS 均值, 列 RMS 均值, 全部最大值)。"""
    g = corners.reshape(rows, cols, 2)
    row_r = [line_rms_dev(g[r, :, :]) for r in range(rows)]
    col_r = [line_rms_dev(g[:, c, :]) for c in range(cols)]
    return (float(np.mean(row_r)), float(np.mean(col_r)),
            float(max(max(row_r), max(col_r))))


def objp(cols, rows, cell_mm):
    p = np.zeros((rows * cols, 3), np.float32)
    p[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return p * (cell_mm / 1000.0)


# ---------------------------------------------------------------- 采集线程
def _blank(w=640, h=480):
    return np.full((h, w, 3), 40, np.uint8)


def reader_thread():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CONFIG['width'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CONFIG['height'])
    time.sleep(1.0)
    ok = cap.isOpened()
    with LOCK:
        STATE['camera_ok'] = ok
    if not ok:
        with LOCK:
            STATE['message'] = '❌ 打不开 /dev/video0'
        return

    cols, rows = CONFIG['cols'], CONFIG['rows']
    last_pose = None
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), CB_FLAGS)
        vis = frame.copy()
        info = {'found': bool(found)}
        if found:
            c = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX)
            pts = c.reshape(-1, 2)
            rr, cr, mx = grid_straightness(pts, cols, rows)
            info.update({
                'row_rms': rr, 'col_rms': cr, 'straight_max': mx,
                'centroid': [float(pts[:, 0].mean()), float(pts[:, 1].mean())],
                'span': float(np.linalg.norm(pts[0] - pts[-1])),
                'corners': pts.tolist(),
            })
            cv2.drawChessboardCorners(vis, (cols, rows), c, found)
            # 姿态差异：与上次采集相比，中心位移 + 尺度变化
            pose = np.array([pts[:, 0].mean() / w, pts[:, 1].mean() / h,
                             np.linalg.norm(pts[0] - pts[-1]) / max(w, h)])
            if last_pose is None:
                info['pose_diff'] = 1.0
            else:
                info['pose_diff'] = float(np.linalg.norm(pose - last_pose))
            info['_pose'] = pose.tolist()

        with LOCK:
            STATE['frame'] = vis
            STATE['info'] = info
            # 自动采集：检测到 + 棋盘够大 + 姿态与上次有足够差异
            # "棋盘够大"这一条是必须的：畸变在画面边缘最大，棋盘只占中间一小块时
            # 采到的角点全在无畸变区域，标出来的畸变系数纯属外推噪声。
            # 实测踩过：1900 多个角点全挤在画面中间 36%，解出 k1=-0.45 k2=+0.35。
            if (CONFIG['auto'] and found and STATE.get('auto_on')
                    and len(STATE['views']) < CONFIG['target_views']):
                pts = np.asarray(info['corners'], dtype=float)
                frac = ((pts[:, 0].max() - pts[:, 0].min())
                        / CONFIG['width'])
                info['board_frac'] = float(frac)
                if frac < CONFIG['min_board_frac']:
                    STATE['flash'] = (
                        f'棋盘只占画面宽 {frac*100:.0f}%，太小没采 '
                        f'(要 {CONFIG["min_board_frac"]*100:.0f}%+)', time.time())
                elif (last_pose is None
                        or info['pose_diff'] > CONFIG['min_pose_diff']):
                    STATE['views'].append({
                        'corners': info['corners'],
                        'shape': [w, h],
                        'row_rms': info['row_rms'],
                        'col_rms': info['col_rms'],
                        'at': time.strftime('%H:%M:%S'),
                    })
                    last_pose = np.array(info['_pose'])
                    STATE['flash'] = (f'已采集第 {len(STATE["views"])} 张 '
                                      f'(姿态差异 {info["pose_diff"]:.3f})',
                                      time.time())
                    _refresh_coverage()
            STATE['hints'] = _guidance(STATE['info'], STATE['coverage'])
        time.sleep(0.03)


def _coverage_grid():
    return np.zeros((CONFIG['cov_rows'], CONFIG['cov_cols']), int)


def _refresh_coverage():
    """角点落在图像哪些区域 —— 畸变标定最忌讳角点全挤在中间。"""
    g = _coverage_grid()
    for v in STATE['views']:
        w, h = v['shape']
        for x, y in v['corners']:
            i = min(CONFIG['cov_rows'] - 1, int(y / h * CONFIG['cov_rows']))
            j = min(CONFIG['cov_cols'] - 1, int(x / w * CONFIG['cov_cols']))
            g[i, j] += 1
    STATE['coverage'] = g.tolist()


def _guidance(info, coverage):
    """告诉用户"下一步棋盘该怎么动"。

    为什么需要这个：用户知道要拿棋盘动，但不知道**怎么动才标得准**。
    畸变在画面边缘最大，所以边缘没有角点数据的话，k1/k2/k3 就是欠约束的，
    解出来的系数纯属拟合噪声（实测踩过：1900 多个角点全挤在画面中间 36%，
    解出 k1=-0.45 k2=+0.35 k3=-0.24 这种反复变号的组合）。
    """
    hints = []
    if not info.get('found'):
        # 现场实测：夹爪停在棋盘正中央时 findChessboardCorners 直接返回 false，
        # 整块板一个角点都提不出来。所以"被挡住"要排在排查顺序第一位。
        hints.append('没检测到棋盘 —— 按顺序查：'
                     '① <b>机械臂是不是挡住棋盘了</b>（用手拖开即可，'
                     '驱动是只读的、扭矩没使能）'
                     '② 棋盘超出画面了吗 ③ 距离太近或太远吗')
        return hints
    pts = np.asarray(info['corners'], dtype=float)
    frac = (pts[:, 0].max() - pts[:, 0].min()) / CONFIG['width']
    if frac < 0.55:
        hints.append(f'棋盘只占画面宽度 {frac*100:.0f}%，<b>拿近一点</b>'
                     f'（要到 60% 以上，让角点接近左右边缘）')
    elif frac > 0.97:
        hints.append('棋盘快出画面了，稍微拿远一点')
    cov = np.asarray(coverage)
    if cov.size:
        edges = {'左': int(cov[:, 0].sum()), '右': int(cov[:, -1].sum()),
                 '上': int(cov[0, :].sum()), '下': int(cov[-1, :].sum())}
        miss = [k for k, v in edges.items() if v < 4]
        if miss:
            hints.append('画面 <b>' + '、'.join(miss) +
                         '</b> 边缘还没覆盖 → 把棋盘往那边挪，并加倾斜')
    n = len(STATE['views'])
    if n < CONFIG['min_views']:
        hints.append(f'还差 {CONFIG["min_views"] - n} 张')
    if not hints:
        hints.append('✅ 覆盖度够了，可以点「求解内参」')
    return hints


def solve():
    views = STATE['views']
    if len(views) < CONFIG['min_views']:
        return {'ok': False,
                'msg': f'至少需要 {CONFIG["min_views"]} 张，现在 {len(views)} 张'}
    # 注意：views[*]['shape'] 存的是 [width, height]（与 numpy 的 shape 顺序相反）。
    # 早先写成 `h, w = shape` 会让 calibrateCamera 收到 (480, 640)，而它是用图像
    # 尺寸给**主点设初值**的 —— 初值被放到 (240, 320)，解出来的主点 (248, 338)
    # 就锁死在那个错位置上，畸变系数随之欠约束跑偏。
    w, h = views[0]['shape']
    op = objp(CONFIG['cols'], CONFIG['rows'], CONFIG['cell_mm'])
    obj = [op.copy() for _ in views]
    img = [np.asarray(v['corners'], np.float32).reshape(-1, 1, 2)
           for v in views]
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj, img, (w, h), None, None)

    per = []
    before_r, before_c, before_m = [], [], []
    for v, o, i2, rv, tv in zip(views, obj, img, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, rv, tv, K, D)
        e = float(np.sqrt(np.mean(np.sum((proj - i2) ** 2, axis=2))))
        per.append(round(e, 3))
        br, bc, bm = grid_straightness(np.asarray(v['corners']), CONFIG['cols'],
                                       CONFIG['rows'])
        before_r.append(br)
        before_c.append(bc)
        before_m.append(bm)

    # 去畸变后重新检测，量直线度改善
    after_r, after_c, after_m = [], [], []
    K2 = K.copy()
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K2, (w, h), cv2.CV_16SC2)
    for v in views:
        # 用同一张图重新去畸变不现实（只存了角点），这里用角点做投影变换近似：
        # 直接用 undistortPoints，再量直线度 —— 与去畸变后重检测等价。
        pts = np.asarray(v['corners'], np.float32).reshape(-1, 1, 2)
        und = cv2.undistortPoints(pts, K, D, P=K2).reshape(-1, 2)
        ar, ac, am = grid_straightness(und, CONFIG['cols'], CONFIG['rows'])
        after_r.append(ar)
        after_c.append(ac)
        after_m.append(am)

    res = {
        'ok': True,
        'rms': float(rms),
        'per_view_rms': per,
        'camera_matrix': K.tolist(),
        'dist_coeffs': D.reshape(-1).tolist(),
        'image_size': [w, h],
        'n_views': len(views),
        'straight_before': {
            'row': float(np.mean(before_r)), 'col': float(np.mean(before_c)),
            'max': float(np.max(before_m))},
        'straight_after': {
            'row': float(np.mean(after_r)), 'col': float(np.mean(after_c)),
            'max': float(np.max(after_m))},
    }
    # ---- 质量裁决 ----
    # 单看 RMS 不够：边缘没数据时，标定可以"拟合得很好"却完全是错的
    # （畸变系数在边缘外推，抓取时刚好在边缘，误差会被放大）。
    cov = np.asarray(STATE.get('coverage') or [[0]])
    edge_n = 0
    if cov.size > 1:
        edge_n = int(cov[:, 0].sum() + cov[:, -1].sum()
                     + cov[0, :].sum() + cov[-1, :].sum())
    imp_row = 1 - np.mean(after_r) / max(np.mean(before_r), 1e-9)
    imp_col = 1 - np.mean(after_c) / max(np.mean(before_c), 1e-9)
    imp = min(imp_row, imp_col)

    bad = []
    if edge_n < 30:
        bad.append(f'画面边缘只有 {edge_n} 个角点，畸变系数欠约束'
                   f'（畸变在边缘最大，必须有数据）')
    if rms > 1.0:
        bad.append(f'RMS {rms:.2f}px 偏大（目标 <0.5，可接受 <1.0）')
    if imp < 0.5:
        bad.append(f'直线度改善仅 {imp*100:.0f}%（<50% 说明畸变没被正确建模）')
    n_bad = int(np.sum(np.asarray(per) > 2.0))
    if n_bad:
        bad.append(f'有 {n_bad} 张重投影误差 >2px（那张的姿态可能没解好，建议删掉重拍）')

    res['edge_corners'] = edge_n
    res['straight_improve'] = float(imp)
    res['verdict'] = ('✅ 可用' if not bad
                      else '❌ <b>不可用于抓取</b>：' + '；'.join(bad))
    res['bad_views'] = [i + 1 for i, v in enumerate(per) if v > 2.0]
    return res


def save_yaml(res):
    path = CONFIG['out']
    ok = str(res.get('verdict', '')).startswith('✅')
    with open(path, 'w') as fh:
        fh.write(f'# SO-101 相机内参标定  {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
        fh.write(f'# 棋盘 {CONFIG["cols"]}x{CONFIG["rows"]} 内角点，'
                 f'格边长 {CONFIG["cell_mm"]}mm，{res["n_views"]} 张\n')
        fh.write(f'# calibrateCamera RMS = {res["rms"]:.4f} px\n')
        sb, sa = res['straight_before'], res['straight_after']
        fh.write(f'# 直线度 RMS(px)  去畸变前 行{sb["row"]:.3f} 列{sb["col"]:.3f} '
                 f'最大{sb["max"]:.3f}\n')
        fh.write(f'#                去畸变后 行{sa["row"]:.3f} 列{sa["col"]:.3f} '
                 f'最大{sa["max"]:.3f}\n')
        fh.write(f'# 边缘角点数 {res.get("edge_corners")}   '
                 f'直线度改善 {res.get("straight_improve", 0)*100:.1f}%\n')
        # quality_ok 是给下游管线（block_pipeline.py）**机器判定**用的：
        # 没有这个字段的文件会被直接拒绝，避免拿没裁决过的参数去抓取。
        fh.write(f'quality_ok={1 if ok else 0}\n')
        fh.write(f'# 质量裁决: '
                 f'{res.get("verdict", "").replace("<b>", "").replace("</b>", "")}\n')
        fh.write(f'image_width: {res["image_size"][0]}\n')
        fh.write(f'image_height: {res["image_size"][1]}\n')
        fh.write('camera_matrix:\n')
        for row in res['camera_matrix']:
            fh.write('  - [' + ', '.join(f'{v:.8f}' for v in row) + ']\n')
        fh.write('distortion_coefficients:\n')
        fh.write('  - [' + ', '.join(f'{v:.10f}'
                                     for v in res['dist_coeffs']) + ']\n')
        # 可信区域：标定时角点实际覆盖到的百分比范围。
        # 畸变在边缘最大，覆盖之外的区域去畸变是外推，结果不可信。
        cov = STATE.get('coverage')
        if cov:
            import numpy as _np
            c = _np.asarray(cov)
            cols_nz = _np.nonzero(c.sum(axis=0))[0]
            rows_nz = _np.nonzero(c.sum(axis=1))[0]
            if len(cols_nz) and len(rows_nz):
                cw, ch = CONFIG['cov_cols'], CONFIG['cov_rows']
                fh.write(f'# 可信区域（角点覆盖到的画面范围，归一化 0~1）\n')
                fh.write(f'trusted_x0={cols_nz[0]/cw:.4f}\n')
                fh.write(f'trusted_x1={(cols_nz[-1]+1)/cw:.4f}\n')
                fh.write(f'trusted_y0={rows_nz[0]/ch:.4f}\n')
                fh.write(f'trusted_y1={(rows_nz[-1]+1)/ch:.4f}\n')
    return path


# ---------------------------------------------------------------- HTTP
PAGE = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>相机内参标定</title><style>
body{font-family:system-ui,"Microsoft YaHei",sans-serif;margin:0;background:#1b1e24;color:#e6e8ec}
.wrap{display:flex;gap:16px;padding:16px}
img{width:760px;border:1px solid #333;border-radius:6px;background:#000}
.panel{flex:1;min-width:340px}
button{background:#2d6cdf;color:#fff;border:0;border-radius:5px;padding:9px 14px;
 font-size:14px;cursor:pointer;margin:3px 3px 3px 0}
button.g{background:#1f9d55}button.r{background:#b03030}button.y{background:#b58900;color:#111}
button:disabled{background:#3a3f47;color:#888;cursor:default}
pre{background:#12151a;padding:10px;border-radius:6px;font-size:12px;overflow:auto;max-height:340px}
.k{display:inline-block;padding:3px 8px;border-radius:4px;font-weight:600;margin-right:6px}
.ok{background:#1f9d55}.no{background:#b03030}.warn{background:#b58900;color:#111}
table{border-collapse:collapse;font-size:13px;width:100%}
td,th{border:1px solid #333;padding:4px 7px;text-align:left}
.big{font-size:22px;font-weight:700}
</style></head><body><div class="wrap">
<div><img src="/stream.mjpg" alt="stream"></div>
<div class="panel">
<h3 style="margin:4px 0">相机内参标定</h3>
<div style="background:#3a2a10;border-left:4px solid #d09a20;padding:7px 10px;
 border-radius:4px;font-size:13px;line-height:1.6;margin-bottom:8px">
<b>开始前：先用手把机械臂挪出画面</b>（它现在夹着物块，正挡在棋盘中央，
会直接导致棋盘检测失败）。驱动是只读的，扭矩没使能，可以直接拖。
</div>
<div id="msg" style="min-height:22px;color:#8fd">　</div>
<div id="hints" style="background:#12151a;border-left:4px solid #2d6cdf;
 padding:8px 10px;border-radius:4px;font-size:13px;line-height:1.7;
 margin-bottom:8px">…</div>
<div>
<button class="g" onclick="post('/capture')">立即采集一张</button>
<button id="autoBtn" onclick="post('/auto')">自动采集：关</button>
<button class="r" onclick="post('/reset')">清空</button>
</div>
<div>
<button class="y" onclick="post('/solve')">求解内参</button>
<button onclick="post('/save')">写入文件</button>
</div>
<div id="stat" style="margin-top:8px"></div>
<canvas id="cov" width="240" height="180"
 style="border:1px solid #333;border-radius:4px;margin-top:8px"></canvas>
<pre id="out">等待…</pre>
</div></div>
<script>
async function post(p){const r=await fetch(p,{method:'POST'});const j=await r.json();
  document.getElementById('msg').textContent=j.msg||'';refresh();}
function drawCov(cov){
  const cv=document.getElementById('cov'),g=cv.getContext('2d');
  const R=cov.length,C=cov[0].length,w=cv.width/C,h=cv.height/R;
  g.clearRect(0,0,cv.width,cv.height);
  let mx=1;for(const row of cov)for(const v of row)mx=Math.max(mx,v);
  for(let i=0;i<R;i++)for(let j=0;j<C;j++){
    const v=cov[i][j];g.fillStyle=v?'rgba(45,108,223,'+(0.25+0.75*v/mx)+')':'#23262c';
    g.fillRect(j*w,i*h,w-2,h-2);
  }
  g.strokeStyle='#555';g.strokeRect(0,0,cv.width,cv.height);
  g.fillStyle='#9aa';g.font='11px sans-serif';g.fillText('角点覆盖度',4,12);
}
async function refresh(){
  const r=await fetch('/state');const s=await r.json();
  let h='';
  const inf=s.info||{};
  h+=(inf.found?'<span class="k ok">检测到棋盘</span>':'<span class="k no">未检测到</span>');
  h+='<span class="k warn">已采集 '+s.n_views+' 张</span>';
  if(inf.found){
    h+='<div style="margin-top:6px">直线度 RMS：行 '+inf.row_rms.toFixed(3)+
       ' px　列 '+inf.col_rms.toFixed(3)+' px</div>';
    h+='<div>姿态差异：'+(inf.pose_diff!==undefined?inf.pose_diff.toFixed(3):'-')+'</div>';
  }
  document.getElementById('stat').innerHTML=h;
  if(s.hints&&s.hints.length){
    document.getElementById('hints').innerHTML=
      s.hints.map(t=>'▸ '+t).join('<br>');
  }
  if(s.coverage)drawCov(s.coverage);
  if(s.result)document.getElementById('out').textContent=JSON.stringify(s.result,null,1);
}
setInterval(refresh,400);refresh();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj):
        b = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path.startswith('/stream.mjpg'):
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=f')
            self.end_headers()
            while True:
                with LOCK:
                    f = STATE.get('frame')
                if f is None:
                    f = _blank()
                ok, jpg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 78])
                if not ok:
                    time.sleep(0.05)
                    continue
                try:
                    self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' +
                                     str(len(jpg)).encode() + b'\r\n\r\n')
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    return
                time.sleep(0.03)
            return
        if self.path.startswith('/state'):
            with LOCK:
                s = {
                    'n_views': len(STATE['views']),
                    'info': STATE.get('info', {}),
                    'coverage': STATE.get('coverage'),
                    'result': STATE.get('result'),
                    'camera_ok': STATE.get('camera_ok'),
                    'hints': STATE.get('hints', []),
                }
                if STATE.get('info'):
                    s['info'] = {k: v for k, v in STATE['info'].items()
                                 if not k.startswith('_') and k != 'corners'}
            return self._json(s)
        body = PAGE.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        p = self.path
        if p.startswith('/capture'):
            with LOCK:
                info = STATE.get('info', {})
                if info.get('found'):
                    STATE['views'].append({
                        'corners': info['corners'],
                        'shape': [CONFIG['width'], CONFIG['height']],
                        'row_rms': info['row_rms'], 'col_rms': info['col_rms'],
                        'at': time.strftime('%H:%M:%S')})
                    _refresh_coverage()
                    return self._json({'msg': f'已采集第 {len(STATE["views"])} 张'})
                return self._json({'msg': '没检测到棋盘，采集失败'})
        if p.startswith('/auto'):
            with LOCK:
                STATE['auto_on'] = not STATE.get('auto_on', False)
                return self._json({'msg': '自动采集：' +
                                   ('开' if STATE['auto_on'] else '关')})
        if p.startswith('/reset'):
            with LOCK:
                STATE['views'].clear()
                STATE['result'] = None
                _refresh_coverage()
                return self._json({'msg': '已清空'})
        if p.startswith('/solve'):
            with LOCK:
                res = solve()
                if res.get('ok'):
                    STATE['result'] = res
                    sb, sa = res['straight_before'], res['straight_after']
                    msg = (f'RMS {res["rms"]:.4f} px；直线度 行 '
                           f'{sb["row"]:.3f}→{sa["row"]:.3f} 列 '
                           f'{sb["col"]:.3f}→{sa["col"]:.3f}')
                else:
                    msg = res.get('msg', '求解失败')
                return self._json({'msg': msg})
        if p.startswith('/save'):
            with LOCK:
                res = STATE.get('result')
                if not res:
                    return self._json({'msg': '先求解'})
                path = save_yaml(res)
                return self._json({'msg': f'已写入 {path}'})
        self._json({'msg': 'unknown'})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8098)
    ap.add_argument('--cols', type=int, default=7)
    ap.add_argument('--rows', type=int, default=5)
    ap.add_argument('--cell-mm', type=float, default=33.0)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--min-views', type=int, default=12)
    ap.add_argument('--target-views', type=int, default=20)
    ap.add_argument('--min-pose-diff', type=float, default=0.035)
    ap.add_argument('--min-board-frac', type=float, default=0.45,
                    help='棋盘至少要占画面宽度的比例，否则不自动采集。'
                         '畸变在边缘最大，棋盘太小则角点全在无畸变区域，'
                         '标出来的畸变系数是外推噪声。')
    ap.add_argument('--cov-rows', type=int, default=4)
    ap.add_argument('--cov-cols', type=int, default=5)
    ap.add_argument('--out', default='/tmp/camera_intrinsics.yaml')
    ap.add_argument('--auto', action='store_true', default=True)
    args = ap.parse_args()

    CONFIG.update(vars(args))
    STATE.update({'views': [], 'info': {}, 'coverage': _coverage_grid().tolist(),
                  'result': None, 'frame': None, 'message': '',
                  'camera_ok': None, 'auto_on': args.auto, 'flash': None})

    threading.Thread(target=reader_thread, daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    print(f'相机内参标定前端: http://0.0.0.0:{args.port}')
    print(f'  棋盘 {args.cols}x{args.rows} 内角点，格边长 {args.cell_mm}mm')
    print(f'  至少 {args.min_views} 张，目标 {args.target_views} 张')
    print(f'  输出 {args.out}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
