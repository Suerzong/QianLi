#!/usr/bin/env python3
"""桌面标定（网页前端）：拖动机械臂让夹爪碰到桌面，记录**夹爪最低点**

为什么不记录 TCP 的 Z
---------------------
"让爪子碰桌子，读 TCP 的 Z" 读数会随姿态剧烈变化：

* 碰到桌子的是**夹爪几何最低点**，不是 TCP；工具竖直时 TCP 比爪尖高 4.6mm；
* TCP 沿工具轴横向偏 7mm，倾斜 θ 时额外产生 7·sin(θ) 的高度差；
* 倾斜 70° 时最低点变成夹爪机身，实测比 TCP 低 40.3mm。

所以本工具记录的是**由模型 FK 算出的夹爪最低点**。无论什么姿态，
只要碰到桌面，这个值就恒等于桌面高度 —— 姿态无关，多点才能互相印证。

用法::

    ~/mj/bin/python table_limit_gui.py --port 8100
    # Windows 浏览器 http://192.168.26.128:8100
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
from gripper_model import GripperModel, JOINTS  # noqa: E402

STATE_LOCK = threading.Lock()
STATE = {
    'joints': None,
    'tcp': None,
    'lowest': None,
    'lowest_link': None,
    'tilt_deg': None,
    'marks': [],
    'plane': None,
    'ok': False,
    'message': '等待 /joint_states …',
    'stable_s': 0.0,
}
CONFIG = {'out': calibration_path('table_limit.txt'), 'margin_mm': 8.0}
_last = {'low': None, 'since': time.monotonic()}


def fit_plane(points):
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid)
    n = vt[2]
    if n[2] < 0:
        n = -n
    return n, float(n @ centroid)


def _plane_locked():
    marks = STATE['marks']
    if len(marks) < 3:
        return None
    pts = np.array([m['p'] for m in marks], dtype=float)
    n, d = fit_plane(pts)
    res = (pts @ n - d) * 1000.0
    tilt = math.degrees(math.acos(min(1.0, abs(float(n[2])))))
    zs = pts @ n
    safe = float(zs.max()) + CONFIG['margin_mm'] / 1000.0
    return {
        'n': np.round(n, 5).tolist(), 'd': round(d, 5),
        'tilt_deg': round(tilt, 3),
        'rms_mm': round(float(np.sqrt(np.mean(res ** 2))), 3),
        'max_mm': round(float(np.max(np.abs(res))), 3),
        'residuals_mm': np.round(res, 2).tolist(),
        'table_z_mm': round(float(zs.mean()) * 1000, 2),
        'safe_z_min_mm': round(safe * 1000, 2),
        'n_marks': len(marks),
        # 注意：NumPy 2.0 移除了 ndarray.ptp()，必须用 np.ptp(arr)
        'xy_span_mm': [round(float(np.ptp(pts[:, 0])) * 1000, 1),
                       round(float(np.ptp(pts[:, 1])) * 1000, 1)],
    }


def state_thread(model):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node('table_limit_gui')
    got = {}

    def on_joints(msg):
        if len(msg.name) == len(msg.position):
            got.clear()
            got.update(dict(zip(msg.name, msg.position)))

    node.create_subscription(JointState, '/joint_states', on_joints, 10)
    while rclpy.ok():
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
            if len(got) < 6:
                with STATE_LOCK:
                    STATE['ok'] = False
                    STATE['message'] = (f'只收到 {len(got)}/6 个关节的 '
                                        f'/joint_states')
                continue
            joints = {k: float(got[k]) for k in JOINTS}
            low, link = model.lowest_point(joints)
            tcp = model.tcp(joints)
            T = model.solve(joints)
            now = time.monotonic()
            if (_last['low'] is None
                    or np.linalg.norm(low - _last['low']) * 1000 > 0.3):
                _last['since'] = now
            _last['low'] = low
            tool = None if tcp is None else T['tcp_link'][:3, 2]
            tilt = (None if tool is None else
                    math.degrees(math.acos(min(1.0, abs(float(tool[2]))))))
            with STATE_LOCK:
                STATE['joints'] = {k: round(v, 4) for k, v in joints.items()}
                STATE['tcp'] = (np.round(tcp, 5).tolist()
                                if tcp is not None else None)
                STATE['lowest'] = np.round(low, 5).tolist()
                STATE['lowest_link'] = link
                STATE['tilt_deg'] = None if tilt is None else round(tilt, 1)
                STATE['stable_s'] = round(now - _last['since'], 1)
                STATE['ok'] = True
                STATE['message'] = ''
        except Exception as exc:  # noqa: BLE001
            # 线程死了会静默卡住，前端看起来"就是没数据"，极难排查 ——
            # 所以把异常写进状态里让界面显示出来。
            import traceback
            with STATE_LOCK:
                STATE['ok'] = False
                STATE['message'] = f'计算失败: {exc}'
            print(traceback.format_exc(), flush=True)
            time.sleep(1.0)
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass


PAGE = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>桌面标定</title><style>
*{box-sizing:border-box}
body{margin:0;font-family:-apple-system,"Segoe UI","Microsoft YaHei",sans-serif;
background:#11151c;color:#e6edf3;padding:18px}
h1{font-size:17px;margin:0 0 4px}
.sub{color:#8b949e;font-size:12px;margin-bottom:14px;line-height:1.6}
.grid{display:grid;grid-template-columns:380px 1fr;gap:14px}
.card{background:#1a2029;border:1px solid #2a323d;border-radius:10px;padding:14px}
.card h2{font-size:13px;margin:0 0 10px;color:#8b949e;font-weight:600;
letter-spacing:.04em;text-transform:uppercase}
.row{display:flex;justify-content:space-between;font-size:13px;padding:5px 0;
border-bottom:1px solid #232a33}
.row:last-child{border-bottom:0}
.k{color:#8b949e}.v{font-family:ui-monospace,Consolas,monospace}
.big{font-size:24px;font-weight:700;font-family:ui-monospace,Consolas,monospace}
button{width:100%;padding:15px;font-size:16px;font-weight:700;border:0;
border-radius:9px;cursor:pointer;margin-top:10px;font-family:inherit}
#rec{background:#2ea043;color:#fff}
#rec:disabled{background:#243b2a;color:#5c7a63}
.mini{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:8px}
.mini button{padding:9px;font-size:13px;background:#252d38;color:#c9d1d9}
.ok{color:#3fb950}.warn{color:#d29922}.bad{color:#f85149}
table{width:100%;border-collapse:collapse;font-size:12.5px;
font-family:ui-monospace,Consolas,monospace}
th,td{text-align:left;padding:5px 6px;border-bottom:1px solid #232a33}
th{color:#8b949e}
#msg{margin-top:10px;font-size:13px;min-height:18px}
.hint{font-size:12px;color:#8b949e;margin-top:8px;line-height:1.7}
</style></head><body>
<h1>桌面标定 —— 用「夹爪最低点」测，不受姿态影响</h1>
<div class="sub">
拖动机械臂让夹爪<b>任意部位</b>轻触桌面即可，<b>不需要</b>刻意摆正。
下面显示的是模型算出的夹爪最低点，碰到桌面时它就等于桌面高度。
</div>
<div class="grid">
 <div>
  <div class="card">
   <h2>当前状态</h2>
   <div class="row"><span class="k">已记录点数</span>
     <span class="v big" id="n">0</span></div>
   <div class="row"><span class="k"><b>夹爪最低点 Z</b></span>
     <span class="v big" id="low">—</span></div>
   <div class="row"><span class="k">最低点位置 XY</span>
     <span class="v" id="lowxy">—</span></div>
   <div class="row"><span class="k">最低点所在部件</span>
     <span class="v" id="lowlink">—</span></div>
   <div class="row"><span class="k">TCP Z（对照）</span>
     <span class="v" id="tcp">—</span></div>
   <div class="row"><span class="k">TCP 比最低点高</span>
     <span class="v" id="delta">—</span></div>
   <div class="row"><span class="k">工具轴偏离竖直</span>
     <span class="v" id="tilt">—</span></div>
   <div class="row"><span class="k">静止</span><span class="v" id="st">—</span></div>
   <button id="rec">记录接触点</button>
   <div class="mini">
     <button onclick="act('undo')">删最后一点</button>
     <button onclick="act('reset')">全部清空</button>
     <button onclick="act('write')">写入限制</button>
   </div>
   <div id="msg"></div>
  </div>
 </div>
 <div>
  <div class="card" style="margin-bottom:14px">
   <h2>拟合结果（≥3 点开始）</h2>
   <div class="row"><span class="k">桌面高度</span>
     <span class="v big" id="tz">—</span></div>
   <div class="row"><span class="k">安全下限 safe_z_min</span>
     <span class="v big ok" id="safe">—</span></div>
   <div class="row"><span class="k">平面倾角</span>
     <span class="v" id="ptilt">—</span></div>
   <div class="row"><span class="k">拟合残差 RMS / 最大</span>
     <span class="v" id="res">—</span></div>
   <div class="row"><span class="k">采样 XY 跨度</span>
     <span class="v" id="span">—</span></div>
   <div class="row"><span class="k">单点残差 (mm)</span>
     <span class="v" id="reslist">—</span></div>
   <div class="hint">
    <b>现在残差应该很小了</b>：因为记录的是最低点，姿态不影响读数。
    RMS &lt; 1mm 属于正常；如果出现大残差，通常是某一下没真碰到桌面。<br>
    建议在你要抓取的区域内分散点 4~6 处，<b>顺便换几个不同姿态</b> ——
    这同时也验证了模型的最低点算得对。
   </div>
  </div>
  <div class="card">
   <h2>已记录的接触点（最低点）</h2>
   <table><thead><tr><th>#</th><th>X</th><th>Y</th><th>Z</th>
   <th>当时倾角</th><th>时间</th></tr></thead><tbody id="marks"></tbody></table>
  </div>
 </div>
</div>
<script>
async function act(a){await fetch('/'+a,{method:'POST'});tick();}
document.getElementById('rec').onclick=()=>act('rec');
function f(v,n){return v===null||v===undefined?'—':Number(v).toFixed(n);}
async function tick(){
 try{
  const s=await (await fetch('/state')).json();
  document.getElementById('n').textContent=s.marks.length;
  const lo=s.lowest;
  document.getElementById('low').textContent=lo?(lo[2]*1000).toFixed(1)+' mm':'—';
  document.getElementById('lowxy').textContent=lo?
    '['+(lo[0]*1000).toFixed(0)+', '+(lo[1]*1000).toFixed(0)+'] mm':'—';
  document.getElementById('lowlink').textContent=s.lowest_link||'—';
  document.getElementById('tcp').textContent=s.tcp?(s.tcp[2]*1000).toFixed(1)+' mm':'—';
  document.getElementById('delta').textContent=
    (lo&&s.tcp)?((s.tcp[2]-lo[2])*1000).toFixed(1)+' mm':'—';
  const t=document.getElementById('tilt');
  t.textContent=s.tilt_deg===null?'—':s.tilt_deg.toFixed(1)+'°';
  t.className='v '+((s.tilt_deg!==null&&s.tilt_deg<15)?'ok':'warn');
  const st=document.getElementById('st');
  st.textContent=(s.stable_s||0).toFixed(1)+'s';
  st.className='v '+((s.stable_s||0)>0.8?'ok':'warn');
  document.getElementById('rec').disabled=!s.ok;
  const pl=s.plane;
  if(pl){
   document.getElementById('tz').textContent=pl.table_z_mm.toFixed(1)+' mm';
   document.getElementById('safe').textContent=pl.safe_z_min_mm.toFixed(1)+' mm';
   document.getElementById('ptilt').textContent=pl.tilt_deg.toFixed(2)+'°';
   const r=document.getElementById('res');
   r.textContent=pl.rms_mm.toFixed(2)+' / '+pl.max_mm.toFixed(2)+' mm';
   r.className='v '+(pl.rms_mm<1?'ok':(pl.rms_mm<2.5?'warn':'bad'));
   document.getElementById('span').textContent=
     '['+pl.xy_span_mm[0]+', '+pl.xy_span_mm[1]+'] mm';
   document.getElementById('reslist').textContent='['+pl.residuals_mm+']';
  }else{
   ['tz','safe','ptilt','res','span','reslist'].forEach(i=>
     document.getElementById(i).textContent='—');
  }
  document.getElementById('marks').innerHTML=s.marks.map((m,i)=>
    '<tr><td>'+(i+1)+'</td>'+m.p.map(v=>'<td>'+(v*1000).toFixed(1)+'</td>').join('')+
    '<td>'+(m.tilt===null?'—':m.tilt.toFixed(1)+'°')+'</td><td>'+m.at+'</td></tr>').join('');
  document.getElementById('msg').innerHTML=
    s.message?'<span class="bad">'+s.message+'</span>':
    (s.flash?'<span class="ok">'+s.flash+'</span>':'');
 }catch(e){document.getElementById('msg').textContent='连接断开：'+e;}
}
setInterval(tick,400);tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):

    def log_message(self, *_a):
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
                self._send(200, json.dumps(STATE))
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
                STATE['plane'] = _plane_locked()
                STATE['flash'] = '已删除最后一点'
        elif route == 'reset':
            with STATE_LOCK:
                STATE['marks'] = []
                STATE['plane'] = None
                STATE['flash'] = '已清空'
        elif route == 'write':
            self._write()
        else:
            self._send(404, '{}')
            return
        self._send(200, '{"ok":true}')

    def _record(self):
        low = _last['low']
        if low is None:
            with STATE_LOCK:
                STATE['message'] = '还没有最低点数据'
            return
        with STATE_LOCK:
            STATE['marks'].append({
                'p': [float(v) for v in low],
                'tilt': STATE['tilt_deg'],
                'link': STATE['lowest_link'],
                'at': time.strftime('%H:%M:%S'),
            })
            STATE['plane'] = _plane_locked()
            STATE['flash'] = (f"已记录第 {len(STATE['marks'])} 点  "
                              f"最低点 Z = {low[2]*1000:.1f} mm  "
                              f"(倾角 {STATE['tilt_deg']}°)")
            with open(calibration_path('table_marks.json'), 'w') as fh:
                json.dump(STATE['marks'], fh, indent=2)

    def _write(self):
        with STATE_LOCK:
            pl = STATE['plane']
            if not pl:
                STATE['message'] = '至少需要 3 个点'
                return
            lines = [
                '# 桌面标定（记录夹爪最低点，姿态无关）',
                f"# 采样 {pl['n_marks']} 点  XY跨度 {pl['xy_span_mm']} mm",
                f"# 平面倾角 {pl['tilt_deg']}°  拟合残差 RMS {pl['rms_mm']} mm",
                '# 依据：碰到桌面的是夹爪几何最低点，不是 TCP',
                f"table_z={pl['d']:.6f}",
                f"table_z_mean={pl['table_z_mm']/1000:.6f}",
                f"table_normal_x={pl['n'][0]:.6f}",
                f"table_normal_y={pl['n'][1]:.6f}",
                f"table_normal_z={pl['n'][2]:.6f}",
                f"safe_z_min={pl['safe_z_min_mm']/1000:.6f}",
                f"margin_mm={CONFIG['margin_mm']}",
            ]
            with open(CONFIG['out'], 'w') as fh:
                fh.write('\n'.join(lines) + '\n')
            STATE['flash'] = (f"已写入 {CONFIG['out']}：桌面 Z "
                              f"{pl['table_z_mm']:.1f} mm，安全下限 "
                              f"{pl['safe_z_min_mm']:.1f} mm")
            STATE['message'] = ''
        print(f'[write] {STATE["flash"]}', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8100)
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--stride', type=int, default=4)
    ap.add_argument('--margin-mm', type=float, default=8.0)
    ap.add_argument('--out', default=calibration_path('table_limit.txt'))
    args = ap.parse_args()

    CONFIG['margin_mm'] = args.margin_mm
    CONFIG['out'] = args.out

    print('加载夹爪模型 …', flush=True)
    model = GripperModel(stride=args.stride)
    print(f'  {len(model.parts)} 个部件，'
          f'{sum(len(v) for v in model.parts.values())} 个采样点', flush=True)

    threading.Thread(target=state_thread, args=(model,), daemon=True).start()
    time.sleep(2.0)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f'桌面标定前端: http://<虚拟机IP>:{args.port}  '
          f'(记录夹爪最低点，姿态无关)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
