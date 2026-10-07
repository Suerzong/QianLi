#!/usr/bin/env python3
"""TCP 标定的网页前端：看着实时姿态，手动点"记录这一点"。

为什么要有前端
--------------
自动判定"摆稳 1.5 秒就记录"看起来省事，实际很难用：标定现场人手会抖、
机械臂会回弹，用户根本不知道系统到底记没记、记得对不对。工业上的做法
就是**人看着数据、人按按钮**。

功能
----
* 实时显示当前工具轴方向、法兰位置、是否静止
* 大按钮"记录这一点"，记完立刻显示它跟已有点的夹角
* 每记录一点就实时重算 TCP 偏移与残差 —— 点够了、残差降下来了，
  用户自己就能判断什么时候停
* 可删除最后一点、清空重来
* 点"求解并写入"落盘到 /tmp/tcp_calib.txt

用法::

    ~/mj/bin/python tcp_calibrate_gui.py --port 8099
    # 然后在 Windows 浏览器打开 http://192.168.26.128:8099
"""

from __future__ import annotations

from project_paths import calibration_path

import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tcp_calibrate import (FLANGE, BASE, quat_to_R,  # noqa: E402
                           solve_tcp_point, solve_tcp)

STATE_LOCK = threading.Lock()
STATE = {
    'axis': None,
    'position': None,
    'stable_s': 0.0,
    'marks': [],
    'solved': None,
    'tf_ok': False,
    'message': '等待 TF …',
    'mode': 'point',
}
CONFIG = {'out': calibration_path('tcp_calib.txt'), 'json': calibration_path('tcp_marks_gui.json')}

_last_pose = {'R': None, 'p': None, 'since': time.monotonic()}
_rclpy_ok = {'value': True}


def _solve_locked():
    marks = STATE['marks']
    if len(marks) < 3:
        return None
    poses = [(np.array(m['R'], dtype=float), np.array(m['p'], dtype=float))
             for m in marks]
    try:
        if STATE['mode'] == 'point':
            t, P, residuals = solve_tcp_point(poses)
            extra = {'fixed_point_m': np.round(P, 5).tolist()}
        else:
            t, (n, d), residuals, info = solve_tcp(poses)
            extra = {'table_z': float(d), 'info': info}
    except Exception as exc:  # noqa: BLE001
        return {'error': str(exc)}
    axes = [np.array(m['R'], dtype=float) @ np.array([0.0, 0.0, 1.0])
            for m in marks]
    spread = 0.0
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            spread = max(spread, math.degrees(math.acos(
                float(np.clip(axes[i] @ axes[j], -1, 1)))))
    rms = float(np.sqrt(np.mean(np.asarray(residuals) ** 2)))
    return {
        'offset_mm': np.round(t * 1000, 3).tolist(),
        'rms_mm': round(rms, 3),
        'max_mm': round(float(np.max(np.abs(residuals))), 3),
        'residuals_mm': np.round(np.asarray(residuals), 2).tolist(),
        'axis_spread_deg': round(spread, 1),
        'n': len(marks),
        **extra,
    }


def _min_sep(axis, marks):
    if not marks:
        return None
    best = 180.0
    for m in marks:
        other = np.array(m['R'], dtype=float) @ np.array([0.0, 0.0, 1.0])
        best = min(best, math.degrees(math.acos(
            float(np.clip(axis @ other, -1, 1)))))
    return best


def tf_thread():
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node('tcp_calibrate_gui')
    buf = Buffer()
    TransformListener(buf, node)
    while _rclpy_ok['value'] and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tr = buf.lookup_transform(BASE, FLANGE, rclpy.time.Time())
        except Exception:
            STATE['tf_ok'] = False
            STATE['message'] = '读不到 TF —— robot_state_publisher 在跑吗？'
            continue
        t = tr.transform.translation
        q = tr.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        p = np.array([t.x, t.y, t.z])
        axis = R @ np.array([0.0, 0.0, 1.0])

        now = time.monotonic()
        stationary = True
        if _last_pose['R'] is not None:
            if np.linalg.norm(p - _last_pose['p']) * 1000 > 0.4:
                stationary = False
            ang = math.degrees(math.acos(float(np.clip(
                (np.trace(_last_pose['R'].T @ R) - 1) / 2, -1, 1))))
            if ang > 0.4:
                stationary = False
        if not stationary:
            _last_pose['since'] = now
        _last_pose['R'], _last_pose['p'] = R, p

        with STATE_LOCK:
            STATE['axis'] = np.round(axis, 4).tolist()
            STATE['position'] = np.round(p, 4).tolist()
            STATE['stable_s'] = round(now - _last_pose['since'], 1)
            STATE['tf_ok'] = True
            STATE['message'] = ''
            STATE['min_sep_deg'] = (None if _min_sep(axis, STATE['marks'])
                                    is None
                                    else round(_min_sep(axis, STATE['marks']), 1))
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.try_shutdown()


PAGE = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>TCP 标定</title><style>
*{box-sizing:border-box}
body{margin:0;font-family:-apple-system,"Segoe UI","Microsoft YaHei",sans-serif;
background:#11151c;color:#e6edf3;padding:18px}
h1{font-size:17px;margin:0 0 4px}
.sub{color:#8b949e;font-size:12px;margin-bottom:14px}
.grid{display:grid;grid-template-columns:340px 1fr;gap:14px}
.card{background:#1a2029;border:1px solid #2a323d;border-radius:10px;padding:14px}
.card h2{font-size:13px;margin:0 0 10px;color:#8b949e;font-weight:600;
letter-spacing:.04em;text-transform:uppercase}
.row{display:flex;justify-content:space-between;font-size:13px;padding:5px 0;
border-bottom:1px solid #232a33}
.row:last-child{border-bottom:0}
.k{color:#8b949e}.v{font-family:ui-monospace,Consolas,monospace}
.big{font-size:30px;font-weight:700;font-family:ui-monospace,Consolas,monospace;
letter-spacing:-.02em}
button{width:100%;padding:15px;font-size:17px;font-weight:700;border:0;
border-radius:9px;cursor:pointer;margin-top:10px;font-family:inherit}
#rec{background:#2ea043;color:#fff}
#rec:disabled{background:#243b2a;color:#5c7a63;cursor:not-allowed}
.mini{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:8px}
.mini button{padding:9px;font-size:13px;background:#252d38;color:#c9d1d9}
.mini button:hover{background:#2f3947}
.ok{color:#3fb950}.warn{color:#d29922}.bad{color:#f85149}
table{width:100%;border-collapse:collapse;font-size:12.5px;
font-family:ui-monospace,Consolas,monospace}
th,td{text-align:left;padding:5px 6px;border-bottom:1px solid #232a33}
th{color:#8b949e;font-weight:600}
#msg{margin-top:10px;font-size:13px;min-height:18px}
.hint{font-size:12px;color:#8b949e;margin-top:8px;line-height:1.6}
</style></head><body>
<h1>SO-101 TCP 标定</h1>
<div class="sub">爪口抓取点 —— 让同一点始终顶住同一个固定尖点，只改变姿态</div>
<div class="grid">
 <div>
  <div class="card">
   <h2>当前姿态</h2>
   <div class="row"><span class="k">记录点数</span>
     <span class="v big" id="n">0</span></div>
   <div class="row"><span class="k">工具轴 (base)</span>
     <span class="v" id="axis">—</span></div>
   <div class="row"><span class="k">法兰位置</span>
     <span class="v" id="pos">—</span></div>
   <div class="row"><span class="k">与已有点最小夹角</span>
     <span class="v" id="sep">—</span></div>
   <div class="row"><span class="k">静止</span>
     <span class="v" id="stable">—</span></div>
   <button id="rec">记录这一点</button>
   <div class="mini">
     <button onclick="act('undo')">删最后一点</button>
     <button onclick="act('reset')">全部清空</button>
     <button onclick="act('mode')">切换模式</button>
   </div>
   <div id="msg"></div>
  </div>
 </div>
 <div>
  <div class="card" style="margin-bottom:14px">
   <h2>实时解算（点够 3 个就开始算）</h2>
   <div class="row"><span class="k">TCP 偏移 (gripper_link)</span>
     <span class="v big" id="off">—</span></div>
   <div class="row"><span class="k">残差 RMS / 最大</span>
     <span class="v" id="res">—</span></div>
   <div class="row"><span class="k">姿态最大跨度</span>
     <span class="v" id="spread">—</span></div>
   <div class="row"><span class="k">单点残差 (mm)</span>
     <span class="v" id="reslist">—</span></div>
   <button id="solve" style="background:#1f6feb;color:#fff">
     求解并写入 /tmp/tcp_calib.txt</button>
   <div class="hint">
    残差 RMS &lt; 1mm 就算可信。<b>姿态跨度越大越准</b>：跨度 60° 以上、
    6 个点左右通常能到 0.5mm。<br>
    够不到某些朝向不用勉强 —— 把能摆到的方向尽量岔开即可。
   </div>
  </div>
  <div class="card">
   <h2>已记录的点</h2>
   <table><thead><tr><th>#</th><th>工具轴</th><th>法兰位置</th>
   <th>时间</th></tr></thead><tbody id="marks"></tbody></table>
  </div>
 </div>
</div>
<script>
async function act(a){await fetch('/'+a,{method:'POST'});tick();}
['rec','solve'].forEach(id=>document.getElementById(id).onclick=()=>act(id));
function fmt(v,n){return v===null||v===undefined?'—':Number(v).toFixed(n);}
async function tick(){
 try{
  const s=await (await fetch('/state')).json();
  document.getElementById('n').textContent=s.marks.length;
  document.getElementById('axis').textContent=s.axis?'['+s.axis.map(v=>v.toFixed(3))+']':'—';
  document.getElementById('pos').textContent=s.position?'['+s.position.map(v=>v.toFixed(3))+']':'—';
  const sep=document.getElementById('sep');
  if(s.min_sep_deg===null||s.min_sep_deg===undefined){sep.textContent='— (第 1 点)';sep.className='v';}
  else{sep.textContent=s.min_sep_deg.toFixed(1)+'°';sep.className='v '+(s.min_sep_deg>=25?'ok':'warn');}
  const st=document.getElementById('stable');
  st.textContent=s.stable_s.toFixed(1)+'s';
  st.className='v '+(s.stable_s>1.5?'ok':'warn');
  document.getElementById('rec').disabled=!s.tf_ok;
  const so=s.solved;
  if(so&&!so.error){
   document.getElementById('off').textContent=
     '['+so.offset_mm.map(v=>v.toFixed(2))+'] mm';
   const r=document.getElementById('res');
   r.textContent=so.rms_mm.toFixed(3)+' / '+so.max_mm.toFixed(3)+' mm';
   r.className='v '+(so.rms_mm<1?'ok':(so.rms_mm<2.5?'warn':'bad'));
   document.getElementById('spread').textContent=so.axis_spread_deg+'°';
   document.getElementById('reslist').textContent='['+so.residuals_mm+']';
  }else{
   document.getElementById('off').textContent='—';
   document.getElementById('res').textContent='—';
   document.getElementById('spread').textContent='—';
   document.getElementById('reslist').textContent='—';
  }
  document.getElementById('marks').innerHTML=s.marks.map((m,i)=>
    '<tr><td>'+(i+1)+'</td><td>['+m.axis.map(v=>v.toFixed(3))+']</td>'+
    '<td>['+m.p.map(v=>v.toFixed(3))+']</td><td>'+m.at+'</td></tr>').join('');
  document.getElementById('msg').innerHTML=
    s.message?'<span class="bad">'+s.message+'</span>':
    (s.flash?'<span class="ok">'+s.flash+'</span>':'');
 }catch(e){document.getElementById('msg').textContent='连接断开：'+e;}
}
setInterval(tick,300);tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):

    def log_message(self, *_args):
        pass

    def _send(self, code, body, ctype='application/json'):
        data = body if isinstance(body, bytes) else str(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._send(200, PAGE.encode(), 'text/html; charset=utf-8')
        elif self.path == '/state':
            with STATE_LOCK:
                payload = dict(STATE)
                payload['solved'] = STATE['solved']
            self._send(200, json.dumps(payload))
        else:
            self._send(404, '{}')

    def do_POST(self):
        route = self.path.lstrip('/')
        if route == 'rec':
            self._record()
        elif route == 'undo':
            with STATE_LOCK:
                if STATE['marks']:
                    STATE['marks'].pop()
                STATE['solved'] = _solve_locked() if len(STATE['marks']) >= 3 else None
                STATE['flash'] = '已删除最后一点'
        elif route == 'reset':
            with STATE_LOCK:
                STATE['marks'] = []
                STATE['solved'] = None
                STATE['flash'] = '已清空'
        elif route == 'mode':
            with STATE_LOCK:
                STATE['mode'] = 'plane' if STATE['mode'] == 'point' else 'point'
                STATE['solved'] = (_solve_locked()
                                   if len(STATE['marks']) >= 3 else None)
                STATE['flash'] = f"模式切换为 {STATE['mode']}"
        elif route == 'solve':
            self._write()
        else:
            self._send(404, '{}')
            return
        self._send(200, '{"ok":true}')

    def _record(self):
        if _last_pose['R'] is None:
            with STATE_LOCK:
                STATE['message'] = '读不到 TF，无法记录'
            return
        R, p = _last_pose['R'], _last_pose['p']
        axis = R @ np.array([0.0, 0.0, 1.0])
        with STATE_LOCK:
            STATE['marks'].append({
                'R': R.tolist(), 'p': p.tolist(),
                'axis': np.round(axis, 4).tolist(),
                'at': time.strftime('%H:%M:%S'),
            })
            STATE['solved'] = (_solve_locked()
                               if len(STATE['marks']) >= 3 else None)
            STATE['flash'] = f"已记录第 {len(STATE['marks'])} 点"
            with open(CONFIG['json'], 'w') as fh:
                json.dump(STATE['marks'], fh, indent=2)

    def _write(self):
        with STATE_LOCK:
            solved = STATE['solved'] or _solve_locked()
            if not solved or solved.get('error'):
                STATE['message'] = '点还不够或解算失败'
                return
            off = solved['offset_mm']
            lines = [
                '# TCP 标定：爪口抓取点相对 gripper_link 的偏移（网页前端实测）',
                f"# 打点数 {solved['n']}  姿态最大跨度 {solved['axis_spread_deg']}°",
                f"# 残差 RMS {solved['rms_mm']} mm  最大 {solved['max_mm']} mm",
                f"flange={FLANGE}",
                f"offset_x={off[0]/1000:.6f}",
                f"offset_y={off[1]/1000:.6f}",
                f"offset_z={off[2]/1000:.6f}",
            ]
            if 'table_z' in solved:
                lines.append(f"table_z={solved['table_z']:.6f}")
            with open(CONFIG['out'], 'w') as fh:
                fh.write('\n'.join(lines) + '\n')
            STATE['flash'] = (f"已写入 {CONFIG['out']}："
                              f"偏移 [{off[0]:.2f}, {off[1]:.2f}, {off[2]:.2f}] mm，"
                              f"残差 RMS {solved['rms_mm']} mm")
            STATE['message'] = ''
        print(f"[write] {CONFIG['out']} <- {STATE['flash']}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8099)
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--mode', default='point', choices=['point', 'plane'])
    ap.add_argument('--out', default=calibration_path('tcp_calib.txt'))
    args = ap.parse_args()

    STATE['mode'] = args.mode
    CONFIG['out'] = args.out

    thread = threading.Thread(target=tf_thread, daemon=True)
    thread.start()
    time.sleep(2.5)

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f'TCP 标定前端已启动: http://<虚拟机IP>:{args.port}', flush=True)
    print(f'  模式 = {args.mode}   写入目标 = {args.out}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        _rclpy_ok['value'] = False
        server.server_close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
