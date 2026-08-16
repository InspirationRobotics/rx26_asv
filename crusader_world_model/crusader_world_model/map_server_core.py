"""map_server_core — the world model's window, served to a laptop browser.

No ROS imports: this module is a snapshot callable, an HTTP server and a page.
Point it at a function that returns a dict and it will serve that dict as JSON
and a page that draws it. That is what makes the display testable off-boat —
`python3 -c` with a fake snapshot function renders the whole map.

THE PATTERN, and why it is not MJPEG like tools/oak_view.py. Those viewers ship
IMAGES because their subject is an image: a camera frame cannot be sent as
anything else. A map is DATA — a boat position and a few dozen targets, well
under a kilobyte — and sending it as data instead of a rendered JPEG means the
Jetson spends no CPU drawing or encoding, the link carries ~1 KB per update
instead of ~100 KB, the laptop can zoom and pan without a round trip, and the
text readouts are selectable rather than pixels. The shared rule survives
either way: the laptop needs a browser and nothing else. No ROS on Windows, no
rviz2, no X forwarding.

WHY THE PAGE IS ONE BIG STRING. The boat has no internet at the dock, let alone
on the water, so every byte the page needs must come from the Jetson. No CDN,
no framework, no separate .css or .js file to install into share/ and then
resolve at runtime. One self-contained string is the version of this that
cannot half-load.

Placeholders are `__NAME__` tokens replaced by str.replace, NOT str.format:
the page is mostly CSS and JavaScript, and both are made of braces. Formatting
it would mean doubling every one of them, and a single missed brace is a
runtime error in a string that nothing type-checks.

WHAT THE PAGE MUST NEVER DO is imply the boat is somewhere it is not. If pose
goes stale the readout does not hold the last number and keep drawing a boat at
it — it says so, loudly, in red, and greys the marker out. A moving map is the
most convincing thing on a screen, and a convincing map of a lie is worse than
a blank one.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<title>Crusader — world map</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
 :root{--bg:#111;--panel:#1b1b1b;--line:#333;--dim:#777;--fg:#ccc;
   --ok:#5fbf6a;--warn:#d8a13a;--bad:#e05252}
 html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);
   font:13px/1.5 ui-monospace,Menlo,Consolas,monospace;overflow:hidden}
 #app{display:flex;height:100%}
 #side{width:280px;flex:none;background:var(--panel);
   border-right:1px solid var(--line);overflow-y:auto;padding:10px 12px}
 #map{flex:1;position:relative}
 canvas{display:block;width:100%;height:100%}
 h2{font-size:12px;text-transform:uppercase;letter-spacing:.08em;
   color:var(--dim);margin:16px 0 6px;font-weight:600}
 h2:first-child{margin-top:0}
 .row{display:flex;justify-content:space-between;gap:8px;padding:2px 0}
 .row span:last-child{color:#fff}
 .big{font-size:20px;color:#fff;letter-spacing:.02em}
 .stale{color:var(--bad)!important}
 #banner{position:absolute;top:8px;left:50%;transform:translateX(-50%);
   background:var(--bad);color:#fff;padding:5px 14px;border-radius:4px;
   font-weight:600;display:none;z-index:5}
 #ctl{position:absolute;top:8px;right:8px;display:flex;gap:4px;z-index:5}
 button{background:#262626;color:var(--fg);border:1px solid #3a3a3a;
   border-radius:4px;padding:4px 10px;font:inherit;cursor:pointer}
 button:hover{background:#303030}
 button.on{background:#2d5a7a;color:#fff;border-color:#3d7aa5}
 table{width:100%;border-collapse:collapse;font-size:12px}
 th{text-align:left;color:var(--dim);font-weight:500;padding:3px 4px 3px 0}
 td{padding:2px 4px 2px 0;white-space:nowrap}
 tr.tent td{color:var(--dim);font-style:italic}
 .dot{display:inline-block;width:8px;height:8px;border-radius:50%;
   margin-right:5px;vertical-align:middle}
</style></head><body>
<div id="app">
 <div id="side">
  <h2>Vessel</h2>
  <div class="row"><span>lat</span><span id="lat" class="big">—</span></div>
  <div class="row"><span>lon</span><span id="lon" class="big">—</span></div>
  <div class="row"><span>heading</span><span id="hdg" class="big">—</span></div>
  <div class="row"><span>speed</span><span id="spd" class="big">—</span></div>
  <div class="row"><span>roll / pitch</span><span id="rp">—</span></div>
  <div class="row"><span>mode</span><span id="mode">—</span></div>
  <div class="row"><span>armed</span><span id="armed">—</span></div>
  <h2>Streams</h2>
  <div class="row"><span>pose</span><span id="s_pose">—</span></div>
  <div class="row"><span>attitude</span><span id="s_att">—</span></div>
  <div class="row"><span>targets</span><span id="s_tgt">—</span></div>
  <h2>Targets (<span id="n">0</span>)</h2>
  <table><thead><tr><th>id</th><th>label</th><th>rng</th><th>brg</th>
    <th>hits</th><th>seen</th></tr></thead><tbody id="tbody"></tbody></table>
 </div>
 <div id="map">
  <canvas id="c"></canvas>
  <div id="banner"></div>
  <div id="ctl">
   <button onclick="zoom(1.35)">+</button>
   <button onclick="zoom(1/1.35)">&minus;</button>
   <button id="b_follow" onclick="toggleFollow()">follow</button>
   <button onclick="clearTrail()">clear trail</button>
  </div>
 </div>
</div>
<script>
"use strict";
var POLL = __POLL_MS__;
var cv = document.getElementById('c'), cx = cv.getContext('2d');
var S = null;                 /* newest snapshot */
var scale = 6;                /* pixels per metre */
var follow = true;
var view = {x:0, y:0};        /* map centre, world metres */
var drag = null;

/* Buoy classes are named by colour, so the map can be honest about what the
   detector said instead of inventing a palette. An unrecognised or empty label
   draws grey — an anonymous LiDAR-only object, which is a real thing to show,
   not a gap to hide. */
function colorOf(label){
  var L = (label||'').toLowerCase();
  if(L.indexOf('red')===0) return '#e05252';
  if(L.indexOf('green')===0) return '#5fbf6a';
  if(L.indexOf('yellow')===0) return '#d8c33a';
  if(L.indexOf('blue')===0) return '#4c8fd8';
  if(L.indexOf('black')===0) return '#8a8a8a';
  return '#9a9a9a';
}
function fmt(v, n){ return (v===null||v===undefined) ? '—' : v.toFixed(n); }
/* Compact elapsed time. A track remembered across a whole run reaches minutes,
   and "247.0" is harder to read at a glance than "4m". */
function ago(s){
  if(s < 60) return s.toFixed(0)+'s';
  if(s < 3600) return Math.floor(s/60)+'m';
  return Math.floor(s/3600)+'h';
}

/* ---- geometry: world metres (x east, y north) -> canvas pixels, north up ----
   All drawing works in CSS pixels (W, H). The backing store is scaled by the
   device pixel ratio and the transform absorbs it, so text and lines are sharp
   on a HiDPI laptop without every coordinate below carrying the ratio. */
var W = 0, H = 0;
function sx(x){ return W/2 + (x - view.x)*scale; }
function sy(y){ return H/2 - (y - view.y)*scale; }

function resize(){
  var r = cv.getBoundingClientRect(), d = window.devicePixelRatio||1;
  W = r.width; H = r.height;
  cv.width = Math.round(W*d); cv.height = Math.round(H*d);
  cx.setTransform(d,0,0,d,0,0);   /* setting .width resets it; must come after */
  draw();
}
window.addEventListener('resize', resize);

function zoom(k){ scale = Math.max(0.3, Math.min(80, scale*k)); draw(); }
function toggleFollow(){
  follow = !follow;
  document.getElementById('b_follow').className = follow ? 'on' : '';
  draw();
}
function clearTrail(){ fetch('/trail/clear', {method:'POST'}); }

/* Dragging turns follow OFF: a user who has panned somewhere is looking at
   that place on purpose, and yanking the view back on the next poll is the
   single most annoying thing a moving map can do. */
cv.addEventListener('mousedown', function(e){ drag = {x:e.clientX, y:e.clientY}; });
window.addEventListener('mouseup', function(){ drag = null; });
window.addEventListener('mousemove', function(e){
  if(!drag) return;
  if(follow) toggleFollow();
  view.x -= (e.clientX-drag.x)/scale;
  view.y += (e.clientY-drag.y)/scale;
  drag = {x:e.clientX, y:e.clientY};
  draw();
});
cv.addEventListener('wheel', function(e){ e.preventDefault(); zoom(e.deltaY<0?1.1:1/1.1); },
                    {passive:false});

/* ---- drawing ---- */
function draw(){
  if(!W) return;
  cx.fillStyle = '#111'; cx.fillRect(0,0,W,H);
  if(!S){ return; }
  var b = S.boat;
  if(follow && b.ok){ view.x = b.x; view.y = b.y; }
  drawRings(b);
  drawTrail();
  drawTargets(b);
  drawBoat(b);
  drawScaleBar();
  drawNorth();
}

/* Range rings are centred on the BOAT, not on the view: they answer "how far
   is that from me", which is the only distance anyone reads off this map. The
   spacing steps through 1/2/5 x 10^n so a ring is always a round number. */
function drawRings(b){
  if(!b.ok) return;
  var step = ringStep();
  cx.strokeStyle = '#232323'; cx.fillStyle = '#4a4a4a';
  cx.font = '11px ui-monospace,monospace'; cx.lineWidth = 1;
  var maxr = Math.hypot(W, H)/scale;
  for(var r=step; r<=maxr; r+=step){
    cx.beginPath(); cx.arc(sx(b.x), sy(b.y), r*scale, 0, 6.2832); cx.stroke();
    cx.fillText(r+' m', sx(b.x)+4, sy(b.y)-r*scale-3);
  }
}
function ringStep(){
  var target = 90/scale, p = Math.pow(10, Math.floor(Math.log(target)/Math.LN10));
  var m = target/p;
  return (m<2?1:m<5?2:5)*p;
}

function drawTrail(){
  var tr = S.trail;
  if(!tr || tr.length<2) return;
  cx.strokeStyle = '#3a5f7a'; cx.lineWidth = 2; cx.beginPath();
  cx.moveTo(sx(tr[0][0]), sy(tr[0][1]));
  for(var i=1;i<tr.length;i++) cx.lineTo(sx(tr[i][0]), sy(tr[i][1]));
  cx.stroke();
}

function drawTargets(b){
  var items = S.targets.items || [];
  for(var i=0;i<items.length;i++){
    var t = items[i], X = sx(t.x), Y = sy(t.y);
    /* A track the boat has turned away from is REMEMBERED, not stale, and the
       map has to say which. Fading alone could not: the old curve hit its floor
       after six seconds, so a perfectly good remembered buoy looked like it had
       nearly gone, and "it disappears when the camera looks away" was the
       natural reading.
       So the fade is now gentle and stops well short of invisible, and anything
       out of sight for more than a moment gets an explicit dashed halo and its
       age in seconds. Remembered objects stay legible; you can still tell at a
       glance which ones the boat is looking at right now. */
    var remembered = t.unseen > 2.0;
    var fade = Math.max(0.55, 1 - t.unseen/60);
    cx.globalAlpha = fade;
    cx.fillStyle = colorOf(t.label);
    cx.strokeStyle = t.confirmed ? '#fff' : '#666';
    cx.lineWidth = t.confirmed ? 1.5 : 1;
    cx.beginPath(); cx.arc(X, Y, 6, 0, 6.2832); cx.fill();
    if(!t.confirmed){ cx.setLineDash([2,2]); }
    cx.stroke(); cx.setLineDash([]);
    /* One-sigma spread of the sightings, drawn only when it is big enough to
       mean something — a ring smaller than the marker is decoration. */
    if(t.stddev*scale > 8){
      cx.strokeStyle = colorOf(t.label); cx.globalAlpha = fade*0.35;
      cx.beginPath(); cx.arc(X, Y, t.stddev*scale, 0, 6.2832); cx.stroke();
      cx.globalAlpha = fade;
    }
    /* Dashed halo = "held from memory, not in view". Drawn outside the marker
       so it never obscures the position itself. */
    if(remembered){
      cx.strokeStyle = colorOf(t.label); cx.lineWidth = 1;
      cx.setLineDash([3,3]); cx.globalAlpha = fade*0.7;
      cx.beginPath(); cx.arc(X, Y, 11, 0, 6.2832); cx.stroke();
      cx.setLineDash([]); cx.globalAlpha = fade;
    }
    cx.fillStyle = '#ddd'; cx.font = '11px ui-monospace,monospace';
    cx.fillText('#'+t.id+' '+(t.label||'unknown'), X+14, Y-4);
    if(b.ok) cx.fillText(fmt(t.range,1)+' m', X+14, Y+8);
    if(remembered){
      cx.fillStyle = '#8a8a8a';
      cx.fillText('seen '+ago(t.unseen)+' ago', X+14, Y+20);
    }
    cx.globalAlpha = 1;
  }
}

/* The boat is a heading arrow, not a dot. Heading is the thing a helmsman
   checks first and a dot cannot show it. */
function drawBoat(b){
  if(!b.ok){ return; }
  var X = sx(b.x), Y = sy(b.y), h = b.heading*Math.PI/180;
  cx.save(); cx.translate(X, Y); cx.rotate(h);   /* compass: cw+ from north */
  cx.fillStyle = b.stale ? '#666' : '#f0f0f0';
  cx.beginPath(); cx.moveTo(0,-12); cx.lineTo(7,9); cx.lineTo(0,4);
  cx.lineTo(-7,9); cx.closePath(); cx.fill();
  cx.restore();
}

function drawScaleBar(){
  var step = ringStep(), px = step*scale, x0 = 14, y0 = H-20;
  cx.strokeStyle = '#888'; cx.lineWidth = 2; cx.beginPath();
  cx.moveTo(x0,y0); cx.lineTo(x0+px,y0);
  cx.moveTo(x0,y0-4); cx.lineTo(x0,y0+4);
  cx.moveTo(x0+px,y0-4); cx.lineTo(x0+px,y0+4); cx.stroke();
  cx.fillStyle = '#aaa'; cx.font = '11px ui-monospace,monospace';
  cx.fillText(step+' m', x0+px+8, y0+4);
}
function drawNorth(){
  var x = W-28, y = 30;
  cx.strokeStyle = '#888'; cx.fillStyle = '#aaa'; cx.lineWidth = 2;
  cx.beginPath(); cx.moveTo(x,y+12); cx.lineTo(x,y-12); cx.stroke();
  cx.beginPath(); cx.moveTo(x,y-16); cx.lineTo(x-4,y-8); cx.lineTo(x+4,y-8);
  cx.closePath(); cx.fill();
  cx.font = '11px ui-monospace,monospace'; cx.fillText('N', x-3, y+24);
}

/* ---- readouts ---- */
function set(id, text, bad){
  var e = document.getElementById(id);
  e.textContent = text;
  e.className = e.className.replace(/ ?stale/,'') + (bad?' stale':'');
}
function ageText(s){
  if(!s.ok) return s.age===null ? 'never' : fmt(s.age,1)+' s STALE';
  return fmt(s.age,2)+' s';
}
function render(){
  var b = S.boat, f = S.fcu, t = S.targets;
  set('lat', b.lat===null?'—':b.lat.toFixed(7), !b.ok);
  set('lon', b.lon===null?'—':b.lon.toFixed(7), !b.ok);
  set('hdg', b.heading===null?'—':fmt(b.heading,1)+'°', !b.ok);
  set('spd', b.speed===null?'—':fmt(b.speed,2)+' m/s', !b.ok);
  set('rp', b.att_ok ? fmt(b.roll,1)+'° / '+fmt(b.pitch,1)+'°' : '—',
      !b.att_ok);
  set('mode', f.ok ? f.mode : '—', !f.ok);
  set('armed', f.ok ? (f.armed?'ARMED':'disarmed') : '—', f.ok && f.armed);
  set('s_pose', ageText(b), !b.ok);
  set('s_att', ageText({ok:b.att_ok, age:b.att_age}), !b.att_ok);
  set('s_tgt', ageText(t), !t.ok);
  document.getElementById('n').textContent = (t.items||[]).length;

  var rows = '';
  (t.items||[]).forEach(function(x){
    /* "seen" is blank while the boat is actually looking at it, so the column
       reads as a list of what is currently OUT of view rather than a wall of
       near-zero numbers. */
    rows += '<tr class="'+(x.confirmed?'':'tent')+'">'
      + '<td><span class="dot" style="background:'+colorOf(x.label)+'"></span>'
      + x.id + '</td><td>' + (x.label||'unknown') + '</td><td>'
      + fmt(x.range,1) + '</td><td>' + fmt(x.bearing,0) + '°</td><td>'
      + x.hits + '</td><td>' + (x.unseen > 2 ? ago(x.unseen) : '') + '</td></tr>';
  });
  document.getElementById('tbody').innerHTML = rows
    || '<tr><td colspan="6" style="color:#666">nothing tracked</td></tr>';

  var msg = '';
  if(!b.ok) msg = 'POSE STALE — vessel position is NOT current';
  else if(!b.att_ok) msg = 'ATTITUDE STALE — target positions are uncompensated';
  else if(!t.ok) msg = 'WORLD MODEL SILENT — is target_tracker running?';
  var el = document.getElementById('banner');
  el.textContent = msg; el.style.display = msg ? 'block' : 'none';
}

function poll(){
  fetch('/state').then(function(r){ return r.json(); }).then(function(j){
    S = j; render(); draw();
  }).catch(function(){
    /* The server going away must not look like a healthy boat sitting still. */
    var el = document.getElementById('banner');
    el.textContent = 'NO CONNECTION TO THE BOAT';
    el.style.display = 'block';
  });
}
document.getElementById('b_follow').className = 'on';
resize();
poll();
setInterval(poll, POLL);
</script></body></html>"""


class MapServer:
    """HTTP front end for one snapshot callable.

    Args:
      snapshot_fn: zero-arg callable returning a JSON-serialisable dict. Called
        once per browser poll, from an HTTP thread — it must be cheap and it
        must be safe to call from a thread that is not the ROS executor's.
      on_clear: zero-arg callable invoked by the page's "clear trail" button,
        or None to make that button a no-op.

    Deliberately holds no state of its own. Everything the page shows comes
    from snapshot_fn, so there is exactly one place where "what is true right
    now" is decided, and it is the node — not a copy inside the web server that
    can drift from it.
    """

    def __init__(self, snapshot_fn, on_clear=None, poll_ms=200):
        self.snapshot_fn = snapshot_fn
        self.on_clear = on_clear
        self.page = _PAGE.replace("__POLL_MS__", str(int(poll_ms))).encode()
        self._server = None

    def handler(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.0"

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    # charset MUST be declared. The page is UTF-8 (degree
                    # signs on every bearing, an em-dash in the title) and a
                    # browser with no charset falls back to latin-1 and renders
                    # "175°" as "175Â°" — a readout that looks broken enough to
                    # be distrusted, which for a display is the whole failure.
                    return self._send(server.page, "text/html; charset=utf-8")
                if self.path == "/state":
                    body = json.dumps(server.snapshot_fn()).encode()
                    # A poll is a request for what is true NOW; a cached
                    # snapshot is a map of the past, drawn convincingly.
                    return self._send(body, "application/json", nocache=True)
                self.send_error(404)

            def do_POST(self):
                if self.path == "/trail/clear":
                    if server.on_clear:
                        server.on_clear()
                    return self._send(b"{}", "application/json", nocache=True)
                self.send_error(404)

            def _send(self, body, ctype, nocache=False):
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", ctype)
                    self.send_header("Content-Length", str(len(body)))
                    if nocache:
                        self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                except ConnectionError:
                    pass      # tab closed mid-write; normal, and not our problem

            def log_message(self, *args):
                pass          # keep the console for ROS logs

        return Handler

    def start(self, port: int, host: str = "0.0.0.0"):
        """Serve from a daemon thread. Returns self so the caller can stop().

        daemon_threads because a browser tab left open must never be able to
        hold up node shutdown — the same reason tools/mjpeg_server.py sets it.
        """
        self._server = ThreadingHTTPServer((host, port), self.handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        return self

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
