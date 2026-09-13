#!/usr/bin/env python3
"""field_gui — place the invented buoy field by hand, in a browser.

Imported by bench_world_model.py under --gui. Not a ROS node and not runnable
on its own: it owns an HTTP server and a page, and nothing else. The bench owns
the field, the anchor and the sensor synthesis, and hands this module two
callables. Keeping the split there means the GUI cannot invent a buoy the
sensor model would not have produced, which is the whole failure mode a second
copy of the field would introduce.

WHY A GUI AT ALL. The static FIELDS tables in bench_world_model are the right
tool for a repeatable check: the same layout every run, diffable, in git. They
are the wrong tool for a first wet test in a small pool, where the field has to
fit whatever water you actually have and you find out the size by looking at it.

COORDINATES. The page works in the WORLD frame the bench stores after anchoring
-- east and north metres from the anchor point -- not the (right, ahead) form
the FIELDS tables are written in. North is up. That makes the page a map rather
than a bow-relative view, so a buoy stays put on screen while the boat swings,
which is the only way to tell a tracker bug from a heading bug.

LAT/LON IS COMPUTED SERVER-SIDE, deliberately. geo.py owns M_PER_DEG and warns
against re-deriving it elsewhere; a copy of that constant in JavaScript is
exactly the drift it warns about, so the page is sent finished numbers.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# 8080 buoy_detector/oak_view, 8081 lidar_view, 8085 bt_view, 8090 ground
# station. 8086 sits next to bt_view because the two are read side by side: the
# field on one screen, the tree reacting to it on the other.
DEFAULT_PORT = 8086

# label -> (screen colour, short name, what the boat must do about it). The
# labels are NOT free text: bt_runner_node's beaconFromLabel() matches
# substrings in the order red, green, blue, off/black, so "flashing_blue" and
# "steady_blue" are the only spellings that reach the tree as ENTRY and EXIT,
# and no label may contain two colour words.
PALETTE = [
    ("flashing_blue_buoy", "#4EAFD2", "ENTRY", "circle it clockwise"),
    ("red_buoy",           "#E0736A", "RED",   "pass it to STARBOARD"),
    ("green_buoy",         "#45BCA0", "GREEN", "pass it to PORT"),
    ("black_buoy",         "#4A5B6C", "OFF",   "obstacle, either side"),
    ("steady_blue_buoy",   "#1D6E8C", "EXIT",  "circle it anticlockwise"),
]

PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Crusader field</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>
:root{--bg:#0C141C;--panel:#131F2A;--line:#25384a;--ink:#DFE8EF;--dim:#7C8FA0;
      --ok:#45BCA0;--warn:#E0A245;--fail:#E0736A;--accent:#4EAFD2}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
  font:14px/1.5 "IBM Plex Sans",system-ui,sans-serif;padding:18px}
.wrap{max-width:1080px;margin:0 auto;display:flex;flex-direction:column;gap:14px}
h1{font-size:19px;margin:0;font-weight:600;letter-spacing:-.01em}
.sub{color:var(--dim);font-size:13px;margin:0}
.bar{display:flex;gap:14px;align-items:center;flex-wrap:wrap;background:var(--panel);
  border:1px solid var(--line);border-radius:8px;padding:10px 14px;
  font-family:"IBM Plex Mono",monospace;font-size:12.5px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--fail);flex:none}
.dot.live{background:var(--ok)}
.cols{display:flex;gap:14px;align-items:flex-start;flex-wrap:wrap}
canvas{background:var(--panel);border:1px solid var(--line);border-radius:8px;
  cursor:crosshair;touch-action:none}
.side{flex:1;min-width:250px;display:flex;flex-direction:column;gap:10px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px 14px}
.card h2{font-size:12px;margin:0 0 9px;color:var(--dim);font-weight:600;
  letter-spacing:.07em;text-transform:uppercase}
.sw{display:flex;align-items:center;gap:9px;padding:6px 8px;border-radius:5px;
  cursor:pointer;font-size:13px;border:1px solid transparent}
.sw:hover{background:#ffffff08}
.sw.on{border-color:var(--accent);background:#4eafd216}
.chip{width:13px;height:13px;border-radius:50%;flex:none}
.sw small{color:var(--dim);margin-left:auto;font-size:11px}
button{background:#ffffff0d;color:var(--ink);border:1px solid var(--line);
  border-radius:5px;padding:6px 11px;font:inherit;font-size:12.5px;cursor:pointer}
button:hover{background:#ffffff18}
button.danger:hover{background:#e0736a26;border-color:var(--fail)}
table{width:100%;border-collapse:collapse;font-family:"IBM Plex Mono",monospace;font-size:11.5px}
td{padding:3px 4px;border-bottom:1px solid #ffffff08}
td.n{text-align:right;color:var(--dim)}
.hint{color:var(--dim);font-size:12px}
label{font-size:12.5px;color:var(--dim)}
input[type=range]{width:110px;vertical-align:middle}
</style></head><body>
<div class="wrap">
  <div>
    <h1>Crusader invented field</h1>
    <p class="sub">Click the water to place a buoy, drag to move it, click a placed
      buoy then Delete to remove it. The bench turns whatever is here into camera
      detections and LiDAR clusters for the real target_tracker.</p>
  </div>

  <div class="bar">
    <span class="dot" id="dot"></span><span id="conn">connecting&hellip;</span>
    <span id="pose"></span>
    <span style="margin-left:auto">
      <label>span <input type="range" id="ext" min="8" max="120" step="2" value="24">
      <span id="extv">24 m</span></label>
    </span>
  </div>

  <div class="cols">
    <canvas id="c" width="620" height="620"></canvas>
    <div class="side">
      <div class="card">
        <h2>Beacon to place</h2>
        <div id="pal"></div>
      </div>
      <div class="card">
        <h2>Field</h2>
        <table id="tbl"></table>
        <p class="hint" id="empty">Nothing placed yet.</p>
        <div style="display:flex;gap:7px;margin-top:10px;flex-wrap:wrap">
          <button id="clr" class="danger">Clear all</button>
          <button id="cpy">Copy east/north table</button>
        </div>
      </div>
      <div class="card">
        <h2>Anchor</h2>
        <div class="hint" id="anch">waiting for the first GPS fix&hellip;</div>
      </div>
    </div>
  </div>
</div>
<script>
var PAL = __PALETTE__;
var sel = 0, buoys = [], boat = null, anchored = false;
var extent = 24, drag = null, picked = -1;
var c = document.getElementById('c'), g = c.getContext('2d');

function mpp(){ return extent / c.width; }              // metres per pixel
function toPx(e, n){ return [c.width/2 + e/mpp(), c.height/2 - n/mpp()]; }
function toM(px, py){ return [(px - c.width/2)*mpp(), (c.height/2 - py)*mpp()]; }

function paint(){
  g.clearRect(0,0,c.width,c.height);
  // Grid at a step that stays legible as the span changes: 1, 2, 5, 10, 20 m.
  var step = 1, steps = [1,2,5,10,20,50];
  for (var i=0;i<steps.length;i++){ step = steps[i]; if (extent/step <= 12) break; }
  g.lineWidth = 1;
  for (var m = -Math.ceil(extent); m <= extent; m += step){
    var p = toPx(m, 0), q = toPx(0, m);
    g.strokeStyle = (m === 0) ? '#2f4a60' : '#1c2c3a';
    g.beginPath(); g.moveTo(p[0],0); g.lineTo(p[0],c.height); g.stroke();
    g.beginPath(); g.moveTo(0,q[1]); g.lineTo(c.width,q[1]); g.stroke();
  }
  g.fillStyle = '#5a7086';
  g.font = '11px "IBM Plex Mono",monospace';
  g.fillText('N', c.width/2 + 5, 14);
  g.fillText(step + ' m grid', 8, c.height - 8);

  // The boat, only while its pose is fresh. A frozen marker at the last known
  // position is the exact lie this repo keeps designing out, so a stale pose
  // draws nothing at all and the status bar says how old it is.
  if (boat){
    var b = toPx(boat.east, boat.north);
    var h = (boat.heading_deg === null) ? null : boat.heading_deg * Math.PI/180;
    g.fillStyle = '#E8B84B';
    g.beginPath(); g.arc(b[0], b[1], 6, 0, 7); g.fill();
    if (h !== null){
      // heading is degrees clockwise from north; screen y grows downward.
      g.strokeStyle = '#E8B84B'; g.lineWidth = 2;
      g.beginPath(); g.moveTo(b[0], b[1]);
      g.lineTo(b[0] + Math.sin(h)*20, b[1] - Math.cos(h)*20); g.stroke();
    } else {
      g.fillStyle = '#E0736A'; g.fillText('yaw?', b[0]+9, b[1]-8);
    }
  }

  for (var j=0;j<buoys.length;j++){
    var q2 = toPx(buoys[j].east, buoys[j].north);
    g.fillStyle = colOf(buoys[j].label);
    g.beginPath(); g.arc(q2[0], q2[1], 8, 0, 7); g.fill();
    if (j === picked){ g.strokeStyle = '#fff'; g.lineWidth = 2;
      g.beginPath(); g.arc(q2[0], q2[1], 12, 0, 7); g.stroke(); }
    g.fillStyle = '#0C141C'; g.font = 'bold 10px "IBM Plex Mono",monospace';
    g.fillText(String(j), q2[0]-3, q2[1]+3.5);
  }
}
// PAL rows are [label, colour, short name, rule]. One scan, not one per
// column: two copies of the same loop drift the moment a column is inserted.
function palOf(l){ for (var i=0;i<PAL.length;i++) if (PAL[i][0]===l) return PAL[i]; return null; }
function colOf(l){ var p = palOf(l); return p ? p[1] : '#888'; }
function shortOf(l){ var p = palOf(l); return p ? p[2] : l; }

function hit(px, py){
  for (var i=buoys.length-1;i>=0;i--){
    var q = toPx(buoys[i].east, buoys[i].north);
    if ((q[0]-px)*(q[0]-px) + (q[1]-py)*(q[1]-py) < 169) return i;
  }
  return -1;
}
function evPx(ev){ var r = c.getBoundingClientRect();
  return [(ev.clientX-r.left)*c.width/r.width, (ev.clientY-r.top)*c.height/r.height]; }

c.addEventListener('pointerdown', function(ev){
  if (!anchored) return;
  var p = evPx(ev), i = hit(p[0], p[1]);
  if (i >= 0){ picked = i; drag = i; }
  else { var m = toM(p[0], p[1]);
    buoys.push({label: PAL[sel][0], east: +m[0].toFixed(2), north: +m[1].toFixed(2)});
    picked = buoys.length-1; drag = picked; push(); }
  c.setPointerCapture(ev.pointerId); paint(); render();
});
c.addEventListener('pointermove', function(ev){
  if (drag === null) return;
  var p = evPx(ev), m = toM(p[0], p[1]);
  buoys[drag].east = +m[0].toFixed(2); buoys[drag].north = +m[1].toFixed(2);
  paint(); render();
});
c.addEventListener('pointerup', function(){ if (drag !== null){ drag = null; push(); } });
window.addEventListener('keydown', function(ev){
  if ((ev.key === 'Delete' || ev.key === 'Backspace') && picked >= 0){
    buoys.splice(picked,1); picked = -1; push(); paint(); render(); ev.preventDefault(); }
});
document.getElementById('ext').addEventListener('input', function(ev){
  extent = +ev.target.value; document.getElementById('extv').textContent = extent + ' m'; paint(); });
document.getElementById('clr').onclick = function(){ buoys = []; picked = -1; push(); paint(); render(); };
document.getElementById('cpy').onclick = function(){
  var s = '# east/north metres from the anchor, NOT the (right, ahead) form FIELDS uses\n';
  for (var i=0;i<buoys.length;i++)
    s += '("' + buoys[i].label + '", ' + buoys[i].east.toFixed(1) +
         ', ' + buoys[i].north.toFixed(1) + '),\n';
  navigator.clipboard.writeText(s);
  var b = this; b.textContent = 'copied';
  setTimeout(function(){ b.textContent = 'Copy east/north table'; }, 2200);
};

function render(){
  var t = document.getElementById('tbl'), h = '';
  for (var i=0;i<buoys.length;i++){
    var b = buoys[i];
    h += '<tr><td style="color:' + colOf(b.label) + '">' + i + '&nbsp;' + shortOf(b.label) +
         '</td><td class="n">E' + b.east.toFixed(1) + '</td><td class="n">N' + b.north.toFixed(1) +
         '</td><td class="n">' + (b.lat ? b.lat.toFixed(6) + ', ' + b.lon.toFixed(6) : '') +
         '</td></tr>';
  }
  t.innerHTML = h;
  document.getElementById('empty').style.display = buoys.length ? 'none' : '';
}
function palette(){
  var d = document.getElementById('pal'), h = '';
  for (var i=0;i<PAL.length;i++)
    h += '<div class="sw' + (i===sel?' on':'') + '" data-i="' + i + '">' +
         '<span class="chip" style="background:' + PAL[i][1] + '"></span>' +
         PAL[i][2] + '<small>' + PAL[i][3] + '</small></div>';
  d.innerHTML = h;
  d.querySelectorAll('.sw').forEach(function(e){
    e.onclick = function(){ sel = +e.dataset.i; palette(); }; });
}
function push(){
  fetch('/field', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({buoys: buoys.map(function(b){
      return {label:b.label, east:b.east, north:b.north}; })})});
}
function poll(){
  fetch('/state').then(function(r){ return r.json(); }).then(function(s){
    anchored = s.anchored; boat = s.boat;
    document.getElementById('dot').className = 'dot' + (s.anchored ? ' live' : '');
    document.getElementById('conn').textContent = s.anchored
      ? 'anchored' : 'waiting for a usable GPS fix - nothing can be placed yet';
    document.getElementById('pose').textContent = s.boat
      ? ('pose ' + s.boat.age_s.toFixed(1) + ' s old' +
         (s.boat.heading_deg === null ? '  -  YAW UNRESOLVED' : ''))
      : '-  no fresh pose';
    document.getElementById('anch').innerHTML = s.anchored
      ? ('<code>' + s.origin[0].toFixed(7) + ', ' + s.origin[1].toFixed(7) +
         '</code><br>heading at anchor ' + s.anchor_heading_deg.toFixed(1) + '&deg;')
      : 'waiting for the first GPS fix&hellip;';
    // Only take the server's field while nothing is being dragged, or the buoy
    // under the pointer would jump back on every poll.
    if (drag === null && s.buoys) buoys = s.buoys;
    paint(); render();
  }).catch(function(){
    document.getElementById('dot').className = 'dot';
    document.getElementById('conn').textContent = 'bench not reachable';
  });
}
palette(); paint(); render(); poll(); setInterval(poll, 500);
</script></body></html>
"""


class _Handler(BaseHTTPRequestHandler):
    gui = None            # set on the class by FieldGui.start()

    def _send(self, code, body, ctype):
        raw = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        if self.path.startswith("/state"):
            self._send(200, json.dumps(self.gui.get_state()), "application/json")
        elif self.path in ("/", "/index.html"):
            self._send(200, PAGE.replace("__PALETTE__", json.dumps(PALETTE)),
                       "text/html; charset=utf-8")
        else:
            self._send(404, "no such path", "text/plain")

    def do_POST(self):
        if not self.path.startswith("/field"):
            self._send(404, "no such path", "text/plain")
            return
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
            # Validated rather than trusted: a label the sensor model does not
            # know would become a KeyError inside the synthesis timer, where it
            # reads as the bench having died rather than the page having sent
            # nonsense. Unknown labels are dropped here, loudly, at the edge.
            known = {p[0] for p in PALETTE}
            clean = [(str(b["label"]), float(b["east"]), float(b["north"]))
                     for b in body.get("buoys", []) if b.get("label") in known]
        except (ValueError, TypeError, KeyError) as exc:
            self._send(400, "bad field: %s" % exc, "text/plain")
            return
        self.gui.set_field(clean)
        self._send(200, "ok", "text/plain")

    def log_message(self, *a):
        pass          # one line per 500 ms poll would bury the bench's own log


class FieldGui:
    """Serves the page and relays edits back to the bench.

    `get_state` returns the dict the page polls; `set_field` takes
    [(label, east, north)] and is called from the HTTP thread, so whatever it
    writes must be protected by the same lock the synthesis timer reads under.
    """

    def __init__(self, get_state, set_field, port=DEFAULT_PORT):
        self.get_state, self.set_field, self.port = get_state, set_field, port
        self._srv = None

    def start(self):
        _Handler.gui = self
        # Bound on all interfaces because the browser is on the laptop and the
        # bench runs on the Jetson — the same reason lidar_view and bt_view do.
        self._srv = ThreadingHTTPServer(("0.0.0.0", self.port), _Handler)
        threading.Thread(target=self._srv.serve_forever, daemon=True).start()
        return self.port

    def stop(self):
        if self._srv:
            self._srv.shutdown()
