#!/usr/bin/env python3
"""rk_control.py v5 - USB直连C3 + 遥控/跟随模式切换 + 云台遥控"""
import os, termios, time, threading, json, socketserver
from http.server import HTTPServer, BaseHTTPRequestHandler

UART = "/dev/ttyACM0"
BAUD = 115200
VIDEO_PORT = 8080
CONTROL_PORT = 80

# 云台行程 (与 C3 固件一致, Pan 实测为 180° 舵机)
PAN_RANGE = 180.0
TILT_RANGE = 180.0
PAN_CENTER = 90.0
TILT_CENTER = 90.0

uart_fd = None
uart_lock = threading.Lock()
mode = "manual"
mode_lock = threading.Lock()
fc = None

def uart_open(port):
    global uart_fd
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        attr = termios.tcgetattr(fd)
        attr[2] = attr[2] & ~(termios.CSTOPB | termios.PARENB | termios.CSIZE) | termios.CS8 | termios.CREAD | termios.CLOCAL
        attr[2] &= ~termios.CRTSCTS; attr[3] = 0
        attr[4] = termios.B115200; attr[5] = termios.B115200
        attr[6][termios.VMIN] = 0; attr[6][termios.VTIME] = 1
        termios.tcsetattr(fd, termios.TCSANOW, attr)
        import fcntl
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl & ~os.O_NONBLOCK)
        uart_fd = fd
        return True
    except Exception as e:
        print(f"[UART] {e}")
        return False

def uart_send(cmd):
    with uart_lock:
        try: os.write(uart_fd, f"{cmd}\r\n".encode()); return True
        except: return False

def gimbal_send(body):
    """body: {"pan":deg} 和/或 {"tilt":deg}, 绝对角度"""
    cmds = []
    if 'pan' in body:
        p = max(0.0, min(PAN_RANGE, float(body['pan'])))
        cmds.append(f"PAN {p:.1f}")
    if 'tilt' in body:
        t = max(0.0, min(TILT_RANGE, float(body['tilt'])))
        cmds.append(f"TLT {t:.1f}")
    if not cmds:
        return False
    ok = True
    for c in cmds:
        ok = uart_send(c) and ok
    return ok

def uart_reader():
    global uart_fd
    while uart_fd is not None:
        try:
            data = os.read(uart_fd, 256)
            if not data:
                time.sleep(0.05)
        except Exception:
            time.sleep(0.1)

# ====== 跟人控制 (延迟import, 失败不影响主服务) ======

def _follow_loop():
    global fc
    try:
        from follow_controller import FollowController
        fc = FollowController(uart_send)
        fc.start()
        while fc.running:
            fc.tick()
            time.sleep(0.4)
    except Exception as e:
        print(f"[follow] import/run error: {e}")
        import traceback; traceback.print_exc()
    finally:
        if fc: fc.stop()

def _start_follow():
    global mode
    try:
        if fc is not None and fc.running:
            fc.resume()
            return
        with mode_lock: mode = "follow"
        threading.Thread(target=_follow_loop, daemon=True).start()
    except Exception as e:
        print(f"[follow] start error: {e}")

def _stop_follow():
    global mode
    with mode_lock: mode = "manual"
    try:
        if fc is not None:
            fc.stop()
    except Exception:
        pass
    uart_send("STOP")

# ====== HTTP Handler ======

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path == '/': self._page()
        elif self.path == '/ping': self._json({"uart": uart_fd is not None, "mode": _get_mode()})
        elif self.path == '/mode': self._json({"mode": _get_mode()})
        else: self.send_response(302); self.send_header('Location','/'); self.end_headers()
    def do_POST(self):
        if self.path == '/move':
            try:
                cl = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(cl)) if cl else {}
                d = body.get('direction', 'F')
                s = max(0, min(255, int(body.get('speed', 150))))
                if d in 'FBLR' and len(d) == 1:
                    self._text("OK" if uart_send(f"MOVE {d} {s}") else "FAIL")
                    return
            except Exception: pass
            self.send_response(400); self._text('BAD')
            return
        if self.path == '/gimbal':
            try:
                cl = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(cl)) if cl else {}
                if gimbal_send(body):
                    self._text("OK"); return
            except Exception: pass
            self.send_response(400); self._text('BAD')
            return
        if self.path == '/gimbal/center':
            ok = gimbal_send({"pan": PAN_CENTER, "tilt": TILT_CENTER})
            self._text("OK" if ok else "FAIL")
            return
        r = {'/forward':'MOVE F 180','/back':'MOVE B 180','/left':'MOVE L 150','/right':'MOVE R 150','/stop':'STOP'}
        if self.path in r:
            self._text("OK" if uart_send(r[self.path]) else "FAIL")
        elif self.path == '/mode/manual':
            _stop_follow()
            self._json({"mode": "manual"})
        elif self.path == '/mode/follow':
            _start_follow()
            self._json({"mode": "follow"})
        else: self.send_response(404); self._text('404')
    def _text(self, m):
        self.send_response(200); self.send_header('Content-Type','text/plain'); self.end_headers(); self.wfile.write(m.encode())
    def _json(self, d):
        self.send_response(200); self.send_header('Content-Type','application/json'); self.end_headers(); self.wfile.write(json.dumps(d).encode())
    def _page(self):
        vp = VIDEO_PORT
        html = f'''<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ShadowCarrier-RK</title>
<style>
*{{margin:0;padding:0;box-sizing:border-box}}
body{{background:#0d1117;color:#c9d1d9;font-family:-apple-system,sans-serif;height:100dvh;display:flex;flex-direction:column}}
.video{{flex:1;display:flex;align-items:center;justify-content:center;background:#000;min-height:0}}
.video img{{max-width:100%;max-height:100%;object-fit:contain}}
.modebar{{display:flex;padding:12px 12px 0;gap:0}}
.modebar button{{flex:1;height:44px;border:0;font-size:17px;font-weight:700;color:#fff;cursor:pointer}}
.modebar button:first-child{{border-radius:12px 0 0 12px}}
.modebar button:last-child{{border-radius:0 12px 12px 0}}
.modebar button.active{{background:#238636}}
.modebar button.inactive{{background:#21262d;color:#8b949e}}
.ctrl{{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;padding:12px;background:#161b22}}
.gimbal{{display:flex;gap:10px;padding:0 12px 12px;background:#161b22;align-items:center}}
.pad{{flex:1;height:96px;background:#21262d;border:1px solid #30363d;border-radius:12px;position:relative;touch-action:none;overflow:hidden}}
.knob{{position:absolute;left:50%;top:50%;width:26px;height:26px;margin:-13px 0 0 -13px;background:#238636;border-radius:50%;pointer-events:none;opacity:.9;transition:transform .08s}}
.padhint{{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:#6e7681;font-size:13px;pointer-events:none}}
.gbtn{{height:96px;width:76px;font-size:15px;background:#30363d;line-height:1.3}}
button{{border:0;border-radius:10px;font-size:20px;font-weight:700;height:64px;touch-action:none;user-select:none;color:#fff}}
button:active{{transform:scale(.96);filter:brightness(.85)}}
.fwd{{background:#238636;grid-column:2}}.bck{{background:#1f6feb;grid-column:2}}
.left{{background:#d29922;grid-column:1;grid-row:2}}
.stop{{background:#da3633;grid-column:2;grid-row:2;font-size:28px}}
.right{{background:#d29922;grid-column:3;grid-row:2}}
.gbtn{{height:48px;font-size:18px;background:#30363d}}
button:disabled{{opacity:.3}}
.status{{padding:8px 12px;background:#0d1117;color:#8b949e;font-size:13px;text-align:center}}
</style></head><body>
<div class="modebar">
<button id="btnManual" class="active" onclick="setMode('manual')">遥控</button>
<button id="btnFollow" class="inactive" onclick="setMode('follow')">跟随</button>
</div>
<div class="video"><img id="vfeed"></div>
<div class="ctrl" id="ctrlPad">
<button class="fwd" data-cmd="forward">▲</button>
<button class="left" data-cmd="left">◀</button>
<button class="stop" data-cmd="stop">■</button>
<button class="right" data-cmd="right">▶</button>
<button class="bck" data-cmd="back">▼</button>
</div>
<div class="gimbal">
<div class="pad" id="gpad"><div class="knob" id="gknob"></div><span class="padhint">拖动云台</span></div>
<button class="gbtn" id="gcenter">◎<br>回中</button>
</div><div class="status" id="s">USB直连 C3 | 遥控模式</div>
<script>
let curMode='manual',t=null,a=null;
function s(c,l){{fetch('/'+c,{{method:'POST'}}).then(r=>r.text()).then(x=>{{document.getElementById('s').textContent=l+': '+x}}).catch(e=>document.getElementById('s').textContent='ERR')}}
function h(c,l){{if(curMode!=='manual')return;if(t)clearInterval(t);a=c;s(c,l);t=setInterval(()=>s(c,l),150)}}
function r(){{if(t){{clearInterval(t);t=null}}if(a){{s('stop','STOP');a=null}}}}
let gt=null,gpos={{pan:{PAN_CENTER},tilt:{TILT_CENTER}}};
function g(d){{
if(curMode!=='manual')return;
if(d==='center'){{gpos={{pan:{PAN_CENTER},tilt:{TILT_CENTER}}};}}
else{{
 gpos.pan=Math.max(0,Math.min({PAN_RANGE},gpos.pan+(d.pan||0)));
 gpos.tilt=Math.max(0,Math.min({TILT_RANGE},gpos.tilt+(d.tilt||0)));
}}
fetch('/gimbal',{{method:'POST',headers:{{'Content-Type':'application/json'}},body:JSON.stringify(gpos)}})
.then(r=>r.text()).then(x=>{{document.getElementById('s').textContent='PAN '+gpos.pan.toFixed(0)+' | TILT '+gpos.tilt.toFixed(0)+' | '+x}})
.catch(e=>document.getElementById('s').textContent='ERR');
}}
const pad=document.getElementById('gpad'),knob=document.getElementById('gknob');
let dragging=false,ox=0,oy=0,kx=0,ky=0;
setInterval(()=>{{if(dragging&&curMode==='manual')g({{pan:kx*0.12,tilt:ky*0.12}})}},100);
document.getElementById('gcenter').addEventListener('pointerdown',e=>{{e.preventDefault();g('center')}});
pad.addEventListener('pointerdown',e=>{{e.preventDefault();if(curMode!=='manual')return;dragging=true;ox=e.clientX-kx;oy=e.clientY-ky;pad.setPointerCapture(e.pointerId)}});
pad.addEventListener('pointermove',e=>{{if(!dragging)return;kx=Math.max(-40,Math.min(40,e.clientX-ox));ky=Math.max(-40,Math.min(40,e.clientY-oy));knob.style.transform='translate('+kx+'px,'+ky+'px)'}});
function endDrag(){{dragging=false;kx=0;ky=0;knob.style.transform='translate(0,0)'}}
pad.addEventListener('pointerup',endDrag);pad.addEventListener('pointercancel',endDrag);pad.addEventListener('lostpointercapture',endDrag);
window.addEventListener('blur',endDrag);
function setMode(m){{
if(m===curMode)return;
fetch('/mode/'+m,{{method:'POST'}}).then(r=>r.json()).then(d=>{{
 curMode=d.mode;
 document.getElementById('btnManual').className=curMode==='manual'?'active':'inactive';
 document.getElementById('btnFollow').className=curMode==='follow'?'active':'inactive';
 document.getElementById('s').textContent='USB直连 C3 | '+(curMode==='manual'?'遥控模式':'跟随模式');
 let btns=document.querySelectorAll('.ctrl button,.gbtn');
 btns.forEach(b=>{{if(b.dataset.cmd!=='stop')b.disabled=curMode!=='manual';}});
 if(curMode!=='manual'){{
   r();
   gpos={{pan:{PAN_CENTER},tilt:{TILT_CENTER}}};
   fetch('/gimbal/center',{{method:'POST'}});
 }}
}}).catch(e=>console.log(e));}}
document.querySelectorAll('.ctrl button').forEach(b=>{{
 b.addEventListener('contextmenu',e=>e.preventDefault());
 if(b.dataset.cmd==='stop'){{
  b.addEventListener('pointerdown',e=>{{
   e.preventDefault();
   if(curMode==='follow'){{setMode('manual');return;}}
   r();
  }});return;
 }}
 b.addEventListener('pointerdown',e=>{{e.preventDefault();b.setPointerCapture(e.pointerId);h(b.dataset.cmd,b.textContent.trim())}});
 b.addEventListener('pointerup',r);b.addEventListener('pointercancel',r);b.addEventListener('lostpointercapture',r);
}});
window.addEventListener('pointerup',r);window.addEventListener('blur',r);
window.onload=function(){{setTimeout(function(){{document.getElementById('vfeed').src='http://192.168.4.1:{vp}/stream';}},500);}};
</script></body></html>'''
        self.send_response(200); self.send_header('Content-Type','text/html; charset=utf-8'); self.end_headers(); self.wfile.write(html.encode())

class TServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True; daemon_threads = True

def _get_mode():
    with mode_lock: return mode

def main():
    global UART
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, default=CONTROL_PORT)
    p.add_argument('--uart', type=str, default=UART)
    args = p.parse_args()
    UART = args.uart
    print(f"ShadowCarrier-RK v5 | HTTP :{args.port} | USB->C3 {UART}")
    ok = uart_open(UART)
    if ok: time.sleep(0.5)
    print("OK" if ok else "UART FAIL (will retry on udev)")
    threading.Thread(target=uart_reader, daemon=True).start()
    httpd = TServer(('0.0.0.0', args.port), Handler)
    try: httpd.serve_forever()
    except KeyboardInterrupt: pass
    finally: httpd.shutdown(); os.close(uart_fd) if uart_fd else None

if __name__ == '__main__': main()
