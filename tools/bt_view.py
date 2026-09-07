#!/usr/bin/env python3
"""bt_view — the live behaviour tree in a browser.

    ros2 run ... bt_runner_node ...      # publishes /crsd/bt_status
    python3 tools/bt_view.py             # then open http://localhost:8085

Subscribes to /crsd/bt_status (JSON, published every tick by bt_runner_node) and
serves one page that draws the tree with each node's live status: what has
already succeeded, what is running now, what has not been reached.

WHY A PAGE AND NOT THE TERMINAL. The terminal render in tree_view.hpp answers
the same question, and it is what you get over ssh with nothing installed. But a
tree is a shape, and a shape reprinted forty times over a run is a shape nobody
can follow. Here the nodes keep their positions and only their colour changes,
which is the whole point — the eye tracks the one that moved.

PORT 8085 is chosen to stay clear of the ones the boat already uses: 8090 the
ground station, 8080 buoy_detector's stream, 8081 lidar_view. Two servers on one
port means the second to start dies with an address-in-use that reads like a
crash.
"""
import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# The newest frame, and nothing else. A queue would let the page fall behind and
# then show a burst of history; the only frame worth drawing is the current one.
_state = {"json": None, "seen": 0}
_lock = threading.Lock()

PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Crusader BT</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>
:root{
  --bg:#0C141C; --panel:#131F2A; --line:#25384a; --ink:#DFE8EF; --dim:#7C8FA0;
  --ok:#45BCA0; --ok-bg:#45bca01f; --run:#E0A245; --run-bg:#e0a2452e;
  --fail:#E0736A; --fail-bg:#e0736a1f; --idle:#4A5B6C; --accent:#4EAFD2;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
  font:14px/1.5 "IBM Plex Sans",system-ui,sans-serif;padding:22px}
.wrap{max-width:900px;margin:0 auto;display:flex;flex-direction:column;gap:16px}
h1{font-size:19px;margin:0;font-weight:600;letter-spacing:-.01em}
.sub{color:var(--dim);font-size:13px;margin:0}
.bar{display:flex;gap:14px;align-items:center;background:var(--panel);
  border:1px solid var(--line);border-radius:8px;padding:10px 14px;
  font-family:"IBM Plex Mono",monospace;font-size:12.5px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--fail)}
.dot.live{background:var(--ok)}
.tree{background:var(--panel);border:1px solid var(--line);border-radius:8px;
  padding:12px 8px;overflow-x:auto}
.row{display:flex;align-items:center;gap:10px;padding:4px 10px;border-radius:5px;
  font-family:"IBM Plex Mono",monospace;font-size:13px;white-space:nowrap;
  transition:background .15s}
.st{width:64px;flex:none;text-align:center;padding:2px 0;border-radius:3px;
  font-size:10.5px;font-weight:600;letter-spacing:.05em;
  background:#ffffff08;color:var(--idle)}
.SUCCESS .st{background:var(--ok-bg);color:var(--ok)}
.RUNNING .st{background:var(--run-bg);color:var(--run)}
.FAILURE .st{background:var(--fail-bg);color:var(--fail)}
.IDLE    {opacity:.45}
.RUNNING {background:var(--run-bg)}
.RUNNING .nm{color:var(--run);font-weight:600}
.ty{color:var(--dim);font-size:11.5px}
.kind-control .nm{color:var(--accent)}
.kind-condition .nm{color:#9db4c8;font-style:italic}
.kind-condition .nm::before{content:'? ';opacity:.6}
.legend{display:flex;gap:16px;color:var(--dim);font-size:12px;flex-wrap:wrap}
.legend b{font-family:"IBM Plex Mono",monospace;font-size:10.5px;padding:2px 7px;
  border-radius:3px;font-weight:600}
</style></head><body>
<div class="wrap">
  <div>
    <h1>Crusader behaviour tree</h1>
    <p class="sub">Live from <code>/crsd/bt_status</code>, published every tick by bt_runner_node.</p>
  </div>
  <div class="bar">
    <span class="dot" id="dot"></span><span id="conn">waiting for the runner…</span>
    <span style="margin-left:auto" id="elapsed"></span>
  </div>
  <div class="tree" id="tree"></div>
  <div class="legend">
    <span><b style="background:var(--run-bg);color:var(--run)">RUNNING</b> ticking now</span>
    <span><b style="background:var(--ok-bg);color:var(--ok)">SUCCESS</b> finished, not re-run</span>
    <span><b style="background:var(--fail-bg);color:var(--fail)">FAILURE</b> its parent decides what that means</span>
    <span><b style="background:#ffffff08;color:var(--idle)">IDLE</b> not reached yet</span>
    <span style="color:#9db4c8;font-style:italic">? italic = a condition (asks, changes nothing)</span>
  </div>
</div>
<script>
var lastSeen = -1;
function draw(s){
  var t = document.getElementById('tree');
  t.innerHTML = s.nodes.map(function(n){
    return '<div class="row ' + n.status + ' kind-' + n.kind + '">' +
      '<span class="st">' + n.status + '</span>' +
      '<span style="width:' + (n.d*20) + 'px;flex:none"></span>' +
      '<span class="nm">' + n.name + '</span>' +
      (n.type !== n.name ? '<span class="ty">' + n.type + '</span>' : '') +
      '</div>';
  }).join('');
  document.getElementById('elapsed') .textContent = 't + ' + s.elapsed_s.toFixed(1) + ' s';
}
function poll(){
  fetch('/state').then(function(r){return r.json();}).then(function(d){
    var dot = document.getElementById('dot'), conn = document.getElementById('conn');
    if(d.tree){
      draw(d.tree);
      var moving = d.seen !== lastSeen; lastSeen = d.seen;
      dot.className = 'dot' + (moving ? ' live' : '');
      conn.textContent = moving ? 'running' : 'no goal active (last frame held)';
    } else {
      conn.textContent = 'no frame yet — is a goal running?';
    }
  }).catch(function(){
    document.getElementById('conn').textContent = 'bt_view not reachable';
  });
}
setInterval(poll, 200); poll();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/state"):
            with _lock:
                body = json.dumps({
                    "tree": json.loads(_state["json"]) if _state["json"] else None,
                    "seen": _state["seen"]}).encode()
            ctype = "application/json"
        else:
            body = PAGE.encode()
            ctype = "text/html; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass          # one line per poll at 5 Hz would drown the console


class BtView(Node):
    def __init__(self, port):
        super().__init__("bt_view")
        self.create_subscription(String, "/crsd/bt_status", self._on, 10)
        srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.get_logger().info(
            "bt_view on http://localhost:%d — waiting for /crsd/bt_status" % port)

    def _on(self, msg):
        with _lock:
            _state["json"] = msg.data
            _state["seen"] += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8085,
                    help="clear of 8090 ground station, 8080/8081 the viewers")
    a = ap.parse_args()
    rclpy.init()
    node = BtView(a.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
