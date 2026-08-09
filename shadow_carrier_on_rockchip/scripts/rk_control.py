#!/usr/bin/env python3
"""rk_control.py v4 - USB直连C3 + 遥控/跟随模式切换"""
import os, termios, time, threading, json, socketserver
from http.server import HTTPServer, BaseHTTPRequestHandler

UART = "/dev/ttyACM0"
BAUD = 115200
VIDEO_PORT = 8080
CONTROL_PORT = 80

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
button{{border:0;border-radius:10px;font-size:20px;font-weight:700;height:64px;touch-action:none;user-select:none;color:#fff}}
button:active{{transform:scale(.96);filter:brightness(.85)}}
.fwd{{background:#238636;grid-column:2}}.bck{{background:#1f6feb;grid-column:2}}
.left{{background:#d29922;grid-column:1;grid-row:2}}
.stop{{background:#da3633;grid-column:2;grid-row:2;font-size:28px}}
.right{{background:#d29922;grid-column:3;grid-row:2}}
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
</div><div class="status" id="s">USB直连 C3 | 遥控模式</div>
<script>
let curMode='manual',t=null,a=null;
function s(c,l){{fetch('/'+c,{{method:'POST'}}).then(r=>r.text()).then(x=>{{document.getElementById('s').textContent=l+': '+x}}).catch(e=>document.getElementById('s').textContent='ERR')}}
function h(c,l){{if(curMode!=='manual')return;if(t)clearInterval(t);a=c;s(c,l);t=setInterval(()=>s(c,l),150)}}
function r(){{if(t){{clearInterval(t);t=null}}if(a){{s('stop','STOP');a=null}}}}
function setMode(m){{
if(m===curMode)return;
fetch('/mode/'+m,{{method:'POST'}}).then(r=>r.json()).then(d=>{{
 curMode=d.mode;
 document.getElementById('btnManual').className=curMode==='manual'?'active':'inactive';
 document.getElementById('btnFollow').className=curMode==='follow'?'active':'inactive';
 document.getElementById('s').textContent='USB直连 C3 | '+(curMode==='manual'?'遥控模式':'跟随模式');
 let btns=document.querySelectorAll('.ctrl button');
 btns.forEach(b=>{{if(b.dataset.cmd!=='stop')b.disabled=curMode!=='manual';}});
 if(curMode!=='manual')r();
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
    print(f"ShadowCarrier-RK v4 | HTTP :{args.port} | USB->C3 {UART}")
    ok = uart_open(UART)
    if ok: time.sleep(0.5)
    print("OK" if ok else "UART FAIL (will retry on udev)")
    threading.Thread(target=uart_reader, daemon=True).start()
    httpd = TServer(('0.0.0.0', args.port), Handler)
    try: httpd.serve_forever()
    except KeyboardInterrupt: pass
    finally: httpd.shutdown(); os.close(uart_fd) if uart_fd else None

if __name__ == '__main__': main()
