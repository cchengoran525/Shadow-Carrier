#!/usr/bin/env python3
"""rk_control.py - KickPi控制面板 v2 (TCP替代UART)
C3 连 KickPi 热点后, 连 C3:8888 发 ASCII 命令。
"""
import socket, time, threading, json, socketserver
from http.server import HTTPServer, BaseHTTPRequestHandler

C3_HOST = "192.168.4.2"
C3_PORT = 8888
VIDEO_PORT = 8080
CONTROL_PORT = 80

sock = None
sock_lock = threading.Lock()

def tcp_send(cmd):
    global sock
    with sock_lock:
        for _ in range(2):  # 最多一次重试
            try:
                if sock is None:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(1.0)
                    sock.connect((C3_HOST, C3_PORT))
                sock.sendall(f"{cmd}\r\n".encode())
                return True
            except Exception:
                try: sock.close()
                except: pass
                sock = None
        return False

class Handler(BaseHTTPRequestHandler):
    label = f"C3@{C3_HOST}:{C3_PORT}"

    def log_message(self, *a): pass

    def do_GET(self):
        if self.path == '/':
            self._serve_page()
        elif self.path == '/ping':
            self._json({"status": "ok", "c3": C3_HOST})
        else:
            self.send_response(302)
            self.send_header('Location', '/')
            self.end_headers()

    def do_POST(self):
        routes = {
            '/forward': 'MOVE F 180', '/back': 'MOVE B 180',
            '/left': 'MOVE L 150', '/right': 'MOVE R 150',
            '/stop': 'STOP',
        }
        if self.path in routes:
            ok = tcp_send(routes[self.path])
            self._text("OK" if ok else "FAIL")
        else:
            self.send_response(404)
            self._text('Not found')

    def _text(self, msg):
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.end_headers()
        self.wfile.write(msg.encode())

    def _json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _serve_page(self):
        vp, ul = VIDEO_PORT, self.label
        html = f'''<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ShadowCarrier-RK</title>
<style>
*{{margin:0;padding:0;box-sizing:border-box}}
body{{background:#0d1117;color:#c9d1d9;font-family:-apple-system,sans-serif;height:100dvh;display:flex;flex-direction:column}}
.video{{flex:1;display:flex;align-items:center;justify-content:center;background:#000;min-height:0}}
.video img{{max-width:100%;max-height:100%;object-fit:contain}}
.ctrl{{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;padding:12px;background:#161b22}}
button{{border:0;border-radius:10px;font-size:20px;font-weight:700;height:64px;touch-action:none;user-select:none;color:#fff}}
button:active{{transform:scale(.96);filter:brightness(.85)}}
.fwd{{background:#238636;grid-column:2}}.bck{{background:#1f6feb;grid-column:2}}
.left{{background:#d29922;grid-column:1;grid-row:2}}
.stop{{background:#da3633;grid-column:2;grid-row:2;font-size:28px}}
.right{{background:#d29922;grid-column:3;grid-row:2}}
.status{{padding:8px 12px;background:#0d1117;color:#8b949e;font-size:13px;text-align:center}}
</style></head><body>
<div class="video"><img src="http://192.168.4.1:{vp}/stream"></div>
<div class="ctrl">
<button class="fwd" data-cmd="forward">▲</button>
<button class="left" data-cmd="left">◀</button>
<button class="stop" data-cmd="stop">■</button>
<button class="right" data-cmd="right">▶</button>
<button class="bck" data-cmd="back">▼</button>
</div><div class="status" id="s">ShadowCarrier-RK | {ul}</div>
<script>
let t=null,a=null;
function s(c,l){{fetch('/'+c,{{method:'POST'}}).then(r=>r.text()).then(x=>{{document.getElementById('s').textContent=l+': '+x}}).catch(e=>document.getElementById('s').textContent='ERR')}}
function h(c,l){{if(t)clearInterval(t);a=c;s(c,l);t=setInterval(()=>s(c,l),150)}}
function r(){{if(t){{clearInterval(t);t=null}}if(a){{s('stop','STOP');a=null}}}}
document.querySelectorAll('button').forEach(b=>{{
 b.addEventListener('contextmenu',e=>e.preventDefault());
 if(b.dataset.cmd==='stop'){{b.addEventListener('pointerdown',e=>{{e.preventDefault();r()}});return}}
 b.addEventListener('pointerdown',e=>{{e.preventDefault();b.setPointerCapture(e.pointerId);h(b.dataset.cmd,b.textContent.trim())}});
 b.addEventListener('pointerup',r);b.addEventListener('pointercancel',r);b.addEventListener('lostpointercapture',r);
}});
window.addEventListener('pointerup',r);window.addEventListener('blur',r);
</script></body></html>'''
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(html.encode())

class TServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True; daemon_threads = True

def main():
    global C3_HOST, C3_PORT
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, default=CONTROL_PORT)
    p.add_argument('--c3', type=str, default=C3_HOST)
    p.add_argument('--c3port', type=int, default=C3_PORT)
    args = p.parse_args()
    C3_HOST = args.c3; C3_PORT = args.c3port
    Handler.label = f"C3@{C3_HOST}:{C3_PORT}"
    print(f"ShadowCarrier-RK v2 | HTTP :{args.port} | C3 {C3_HOST}:{C3_PORT}")
    httpd = TServer(('0.0.0.0', args.port), Handler)
    try: httpd.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        httpd.shutdown()
        if sock: sock.close()

if __name__ == '__main__': main()
