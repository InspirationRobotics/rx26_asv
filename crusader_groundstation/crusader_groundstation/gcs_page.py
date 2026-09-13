"""gcs_page — the ground station page: markup, style and browser logic.

One self-contained string, for the reason tools/mjpeg_server.py gives and this
page needs more of: the boat has no internet at the dock, let alone on the
water, so every byte must come from the Jetson. No CDN, no framework, no
separate .css to install into share/ and then resolve at runtime. A page that
cannot half-load is worth more here than a tidy file layout.

Placeholders are `__NAME__` tokens replaced by str.replace, NOT str.format:
this is mostly CSS and JavaScript, both made of braces, and formatting it would
mean doubling every one of them with a missed brace as a runtime error in a
string nothing type-checks.

THE PAGE NEVER DECIDES WHAT IS ALLOWED. Every rule — which nodes are
protected, which contend for a device, whether power is unlocked — is computed
on the server and arrives in /state as a fact plus a reason. The page renders
disabled controls and shows the reason; it does not re-derive the rule. Anyone
can edit JavaScript in a browser, so a rule enforced here is decoration. The
server rejects the request again on arrival regardless of what the page allowed
the operator to click.

STALENESS IS THE FEATURE, throughout. A dead stream never renders as a
plausible last value: the number goes to an em-dash, the label goes red, and
something says what is missing. A convincing dashboard of a lie is worse than
a blank one, and this page has six tabs' worth of opportunity to tell one.
"""

PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<title>Crusader — ground station</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
 /* TWO THEMES, AND EVERY COLOUR IS A VARIABLE. The map is a <canvas>, which
    cannot inherit CSS, so it reads these same variables back out of the
    computed style at draw time — that is the only way one palette can drive
    both halves of the page. A hardcoded hex anywhere below is therefore a bug
    that shows up as one element staying dark in daylight, so there are none:
    even the button faces and field backgrounds are named.

    NIGHT is the default and is the original palette, tuned for a dim room.
    DAY is not "the light theme" — it is a SUNLIGHT theme, for a laptop on a
    dock in Singapore at midday. That is a different design problem: contrast
    is everything and subtlety is the enemy, so text goes to near-black on
    near-white, borders are dark enough to survive glare, and the status
    colours are DARKENED rather than lightened. The night palette's #5fbf6a
    green is a pleasant green on #111 and is invisible on white in sun, which
    would make the one thing worth seeing at a glance — is it OK or not — the
    first thing to disappear. */
 :root{--bg:#111;--panel:#1b1b1b;--card:#202020;--line:#333;--dim:#777;
   --fg:#ccc;--ok:#5fbf6a;--warn:#d8a13a;--bad:#e05252;--accent:#2d5a7a;
   --accentline:#3d7aa5;--onaccent:#fff;--muted:#666;--btn:#262626;
   --btnhi:#303030;--btnline:#3a3a3a;--field:#0c0c0c;--fieldfg:#999;
   --dot:#555;--rowline:#262626;--strong:#eee;--vstrong:#fff;
   --danger:#7a3030;--dangerfg:#e79a9a;--gofg:#cfe6f5;--lockbg:#2a1a1a;
   --trail:#3a5f7a;--lbl-red:#e05252;--lbl-green:#5fbf6a;--lbl-yellow:#d8c33a;
   --lbl-blue:#4c8fd8;--lbl-black:#8a8a8a;--lbl-none:#9a9a9a;
   --lidar:#7fa8c9;--prox:#d8a13a}
 html[data-theme="day"]{--bg:#fff;--panel:#e7ebef;--card:#eef1f4;--line:#8d99a6;
   --dim:#3f4a55;--fg:#0b0e12;--ok:#0a6b2e;--warn:#8a4b00;--bad:#b0151c;
   --accent:#14496e;--accentline:#0d3552;--onaccent:#fff;--muted:#5a6672;
   --btn:#dde3e9;--btnhi:#ccd5dd;--btnline:#8d99a6;--field:#fff;
   --fieldfg:#2b343d;--dot:#98a4b0;--rowline:#c3ccd4;--strong:#000;
   --vstrong:#000;--danger:#b0151c;--dangerfg:#8c1015;--gofg:#0d3552;
   --lockbg:#f7dcdc;--trail:#14496e;--lbl-red:#c01c22;--lbl-green:#0a6b2e;
   --lbl-yellow:#7a5c00;--lbl-blue:#14496e;--lbl-black:#2b343d;
   --lbl-none:#5a6672;--lidar:#0f5c8a;--prox:#8a4b00}
 html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);
   font:13px/1.5 ui-monospace,Menlo,Consolas,monospace;overflow:hidden}
 #app{display:flex;flex-direction:column;height:100%}
 #bar{display:flex;align-items:center;gap:4px;padding:6px 10px;
   background:var(--panel);border-bottom:1px solid var(--line);flex-wrap:wrap}
 #bar button.tab{background:transparent;color:var(--dim);border:1px solid transparent;
   border-radius:4px;padding:5px 12px;font:inherit;cursor:pointer}
 #bar button.tab:hover{background:var(--btn);color:var(--fg)}
 #bar button.tab.on{background:var(--accent);color:var(--onaccent);
   border-color:var(--accentline)}
 #link{margin-left:auto;display:flex;align-items:center;gap:8px;font-size:12px}
 .pill{padding:2px 8px;border-radius:10px;background:var(--btn)}
 main{flex:1;overflow:auto;position:relative}
 .pane{display:none;padding:12px 14px}
 .pane.on{display:block}
 #p-map.on,#p-cam.on,#p-lidar.on{padding:0;height:100%;display:flex}
 button{background:var(--btn);color:var(--fg);border:1px solid var(--btnline);
   border-radius:4px;padding:4px 12px;font:inherit;cursor:pointer}
 button:hover:not(:disabled){background:var(--btnhi)}
 button:disabled{opacity:.4;cursor:not-allowed}
 button.danger{border-color:var(--danger);color:var(--dangerfg)}
 button.go{border-color:var(--accentline);color:var(--gofg)}
 button.on{background:var(--accent);color:var(--onaccent);
   border-color:var(--accentline)}
 h3{font-size:12px;text-transform:uppercase;letter-spacing:.08em;color:var(--dim);
   margin:16px 0 6px;font-weight:600}
 h3:first-child{margin-top:0}
 .hint{color:var(--dim);font-size:12px;margin:4px 0 0}
 .row{display:flex;align-items:center;gap:10px;padding:6px 0;
   border-bottom:1px solid var(--rowline)}
 .dot{width:9px;height:9px;border-radius:50%;flex:none;background:var(--dot)}
 .dot.up{background:var(--ok)}.dot.dead{background:var(--bad)}
 .nm{flex:1;color:var(--strong)}
 .meta{color:var(--dim);font-size:11px}
 .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
 .card{background:var(--card);border-radius:6px;padding:9px 12px}
 .card .k{font-size:11px;color:var(--dim)}
 .card .v{font-size:19px;color:var(--vstrong);margin-top:2px}
 .stale{color:var(--bad)!important}
 #banner{position:absolute;top:8px;left:50%;transform:translateX(-50%);
   background:var(--bad);color:#fff;padding:5px 14px;border-radius:4px;
   font-weight:600;display:none;z-index:5}
 canvas{display:block;width:100%;height:100%}
 #mapwrap{flex:1;position:relative}
 #mapctl{position:absolute;top:8px;right:8px;display:flex;gap:4px;z-index:5;
   flex-wrap:wrap;justify-content:flex-end;max-width:70%}
 #maplegend{position:absolute;top:8px;left:8px;z-index:5;font-size:11px;
   color:var(--dim);line-height:1.7;pointer-events:none;white-space:pre}
 .viewer{flex:1;display:flex;align-items:center;justify-content:center;
   flex-direction:column;gap:10px;padding:20px}
 .viewer img{max-width:100%;max-height:100%;object-fit:contain}
 .empty{border:1px dashed var(--btnline);border-radius:6px;padding:34px 24px;
   text-align:center;color:var(--dim);max-width:520px}
 pre{background:var(--field);border:1px solid var(--line);border-radius:4px;
   padding:8px;margin:6px 0 0;font-size:11px;max-height:150px;overflow:auto;
   white-space:pre-wrap;color:var(--fieldfg)}
 input{background:var(--field);border:1px solid var(--btnline);color:var(--fg);
   border-radius:4px;padding:5px 8px;font:inherit}
 select{background:var(--field);color:var(--fg);border:1px solid var(--btnline);
   border-radius:4px;padding:4px;font:inherit}
 .lock{background:var(--lockbg);border:1px solid var(--danger);
   border-radius:6px;padding:12px}
 #toast{position:absolute;bottom:12px;left:50%;transform:translateX(-50%);
   background:var(--panel);border:1px solid var(--line);color:var(--fg);
   border-radius:4px;padding:7px 14px;display:none;z-index:6;max-width:80%}
</style>
<script>
/* Applied BEFORE the body renders. Doing it from the main script at the end of
   the document means one frame of the dark theme on every load, which on a
   laptop in direct sun is a frame of a screen you cannot read — the exact
   condition day mode exists for. Wrapped because a browser set to block site
   data throws on the accessor rather than returning null. */
try{ if(localStorage.getItem('crsd-theme') === 'day')
       document.documentElement.setAttribute('data-theme','day'); }catch(e){}
</script>
</head><body>
<div id="app">
 <div id="bar">
  <button class="tab" data-t="nodes">Nodes</button>
  <button class="tab" data-t="tel">Telemetry</button>
  <button class="tab" data-t="cam">Camera</button>
  <button class="tab" data-t="lidar">LiDAR</button>
  <button class="tab" data-t="map">Map</button>
  <button class="tab" data-t="tune">Tuning</button>
  <button class="tab" data-t="rec">Record</button>
  <button class="tab" data-t="logs">Logs</button>
  <button class="tab" data-t="sys">System</button>
  <span id="link"><span class="pill" id="rtt">— ms</span>
   <span class="pill" id="armed">—</span>
   <button id="theme" title="high-contrast palette for reading the screen in
 direct sun">&#9728; day</button></span>
 </div>
 <main>
  <div class="pane" id="p-nodes"></div>
  <div class="pane" id="p-tel"></div>
  <div class="pane" id="p-cam"></div>
  <div class="pane" id="p-lidar"></div>
  <div class="pane" id="p-map">
   <div id="mapwrap"><canvas id="c"></canvas>
    <div id="mapctl">
     <button onclick="zoom(1.35)">+</button>
     <button onclick="zoom(1/1.35)">&minus;</button>
     <button id="b_follow" onclick="toggleFollow()">follow</button>
     <button id="b_bow" onclick="toggleBow()" title="rotate the map so the bow
 points up — the frame QGC's PRX1 view uses">bow-up</button>
     <button id="b_clusters" onclick="toggleLayer('clusters')" title="raw
 lidar_cluster_node output, before tracking">clusters</button>
     <button id="b_prox" onclick="toggleLayer('prox')" title="the 72
 OBSTACLE_DISTANCE sectors as the autopilot receives them">PRX1</button>
     <button onclick="post('/trail/clear',{})">clear trail</button>
    </div>
    <div id="maplegend"></div></div>
  </div>
  <div class="pane" id="p-tune"></div>
  <div class="pane" id="p-rec"></div>
  <div class="pane" id="p-logs"></div>
  <div class="pane" id="p-sys"></div>
  <div id="banner"></div>
  <div id="toast"></div>
 </main>
</div>
<script>
"use strict";
var POLL = __POLL_MS__;
var S = null, tab = 'nodes', rtt = null, inflight = false;
var logSeq = 0, logRows = [], logLevel = 20, logNode = '', logBusy = false;

function el(id){ return document.getElementById(id); }
function esc(s){ return String(s==null?'':s).replace(/[&<>"]/g,
  function(c){ return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]; }); }
function fmt(v,n){ return (v===null||v===undefined)?'—':Number(v).toFixed(n); }
function ago(s){
  if(s===null||s===undefined) return '—';
  if(s < 60) return s.toFixed(0)+'s';
  if(s < 3600) return Math.floor(s/60)+'m';
  return Math.floor(s/3600)+'h';
}
function toast(msg, bad){
  var t = el('toast');
  t.textContent = msg;
  t.style.borderColor = bad ? 'var(--danger)' : 'var(--line)';
  t.style.display = 'block';
  clearTimeout(t._h); t._h = setTimeout(function(){ t.style.display='none'; }, 4500);
}

/* Every mutating action goes through here so the reply is always surfaced. A
   button that silently fails is how someone concludes the boat is wedged. */
function post(path, body){
  return fetch(path, {method:'POST', headers:{'Content-Type':'application/json'},
                      body: JSON.stringify(body||{})})
    .then(function(r){ return r.json(); })
    .then(function(j){ if(j.message) toast(j.message, !j.ok); poll(); return j; })
    .catch(function(e){ toast('request failed: '+e, true); });
}

/* THE CANVAS CANNOT INHERIT CSS, so the map's palette is read back out of the
   computed style and cached here. Re-reading it per draw would mean a
   getComputedStyle call for every ring, target and label at 5 Hz; re-reading
   it per THEME CHANGE is once. This is what keeps one palette driving both the
   DOM and the map — the alternative is a second copy of every colour in
   JavaScript, which is a copy that drifts. */
var PAL = {};
function readPalette(){
  var s = getComputedStyle(document.documentElement);
  function v(n){ return s.getPropertyValue(n).trim(); }
  PAL = {bg:v('--bg'), ring:v('--rowline'), ringfg:v('--muted'),
         trail:v('--trail'), conf:v('--vstrong'), tent:v('--muted'),
         fg:v('--strong'), dim:v('--dim'), muted:v('--muted'),
         boat:v('--vstrong'), red:v('--lbl-red'), green:v('--lbl-green'),
         yellow:v('--lbl-yellow'), blue:v('--lbl-blue'),
         black:v('--lbl-black'), none:v('--lbl-none'),
         lidar:v('--lidar'), prox:v('--prox')};
}
function setTheme(mode){
  document.documentElement.setAttribute('data-theme', mode);
  /* Persisted per browser: the operator squinting at a laptop in the sun is
     the same operator after the next reload, and re-picking it every time is
     the friction that means nobody uses it. */
  try{ localStorage.setItem('crsd-theme', mode); }catch(e){}
  var b = el('theme');
  if(b){
    b.innerHTML = mode === 'day' ? '&#9790; night' : '&#9728; day';
    b.title = mode === 'day' ? 'back to the low-light palette'
                             : 'high-contrast palette for direct sun';
  }
  readPalette();
  if(tab === 'map') draw();
}

function show(t){
  tab = t;
  ['nodes','tel','cam','lidar','map','tune','rec','logs','sys'].forEach(function(p){
    el('p-'+p).className = 'pane' + (p===t ? ' on' : ''); });
  document.querySelectorAll('#bar button.tab').forEach(function(b){
    b.className = 'tab' + (b.dataset.t===t ? ' on' : ''); });
  /* Viewer tabs mount their <img> only while visible. mjpeg_server counts
     clients and skips rendering when nobody is watching, so an unopened camera
     tab genuinely costs the Jetson nothing — but only if the img is gone, not
     merely hidden. A display:none <img> keeps its connection open. */
  if(t!=='logs') el('p-logs').innerHTML = '';   /* rebuild with fresh nodes */
  render();
  if(t==='map') resize();
  /* Values are re-read on every entry to the tab. A parameter panel that shows
     what it showed ten minutes ago is worse than one that shows nothing: the
     numbers look current, and the operator is about to decide whether to
     change one based on where it already is. */
  if(t==='tune') tuneLoad();
}

/* ---------------- tab 1: nodes ---------------- */
function renderNodes(){
  var n = S.nodes, out = '';
  out += '<div style="display:flex;gap:6px;align-items:center;flex-wrap:wrap">';
  out += '<span class="meta">'+n.up+' of '+n.items.length+' running</span>';
  out += '<span style="margin-left:auto"></span>';
  S.profiles.forEach(function(p){
    out += '<button class="go" onclick="post(\'/profile/start\',{name:\''+p.id+'\'})">'
        +  esc(p.label)+'</button>'; });
  out += '</div>';

  n.groups.forEach(function(g){
    var items = n.items.filter(function(i){ return i.group===g.id; });
    if(!items.length) return;
    out += '<h3>'+esc(g.label)+'</h3><div class="hint">'+esc(g.hint)+'</div>';
    items.forEach(function(i){
      var cls = i.running ? 'dot up' : (i.state==='exited' ? 'dot dead' : 'dot');
      var right;
      if(i.running && i.stoppable)
        right = '<button onclick="post(\'/node/stop\',{name:\''+i.name+'\'})">Stop</button>';
      else if(i.running)
        right = '<span class="meta" title="'+esc(i.stop_reason)+'">'+
                (i.protected ? 'protected' : 'not ours')+'</span>';
      else
        right = '<button onclick="startNode(\''+i.name+'\')">Start</button>';
      out += '<div class="row"><span class="'+cls+'"></span>'
          +  '<span class="nm">'+esc(i.label)+'</span>'
          +  '<span class="meta">'+esc(i.detail||i.note||'')+'</span>'
          +  '<span class="meta">'+esc(i.package)+'</span>'+right+'</div>';
      if(i.state==='exited' && i.tail && i.tail.length)
        out += '<pre>'+esc(i.tail.join('\n'))+'</pre>';
    });
  });
  el('p-nodes').innerHTML = out;
}

/* The conflict prompt is built from the server's own conflict list, not from a
   copy of the exclusion rule kept here. */
function startNode(name){
  var item = S.nodes.items.filter(function(i){ return i.name===name; })[0];
  if(item && item.conflicts && item.conflicts.length){
    if(!confirm(item.conflicts.join(', ')+' holds the same device.\n\n'
                +'Stop it and start '+name+'?')) return;
    return post('/node/start', {name:name, stop_conflicts:true});
  }
  return post('/node/start', {name:name});
}

/* ---------------- tab 2: telemetry ---------------- */
function renderTel(){
  var b = S.boat, f = S.fcu, out = '<div class="grid">';
  function card(k,v,bad){ return '<div class="card"><div class="k">'+k+'</div>'
    + '<div class="v'+(bad?' stale':'')+'">'+v+'</div></div>'; }
  out += card('Latitude', b.lat===null?'—':b.lat.toFixed(7), !b.ok);
  out += card('Longitude', b.lon===null?'—':b.lon.toFixed(7), !b.ok);
  out += card('Speed', b.speed===null?'—':fmt(b.speed,2)+' m/s', !b.ok);
  out += card('Heading', b.heading===null?'—':fmt(b.heading,1)+'°', !b.ok);
  out += card('Roll', b.roll===null?'—':fmt(b.roll,1)+'°', !b.att_ok);
  out += card('Pitch', b.pitch===null?'—':fmt(b.pitch,1)+'°', !b.att_ok);
  out += card('Yaw', b.yaw===null?'—':fmt(b.yaw,1)+'°', !b.att_ok);
  out += card('Mode', f.ok?esc(f.mode):'—', !f.ok);
  out += card('Armed', f.ok?(f.armed?'ARMED':'disarmed'):'—', f.ok&&f.armed);
  out += '</div>';

  out += '<h3>Latency</h3><div class="hint">Three links that fail '
      +  'independently. One merged number would hide which one is bad.</div>';
  var rows = [
    ['Browser to boat', 'HTTP round trip',
     rtt===null?'—':rtt.toFixed(0)+' ms', rtt!==null && rtt < 400],
    ['Pixhawk to Jetson', '/crsd/pose age',
     b.age===null?'never':fmt(b.age,2)+' s', b.ok],
    ['Attitude', '/crsd/attitude age',
     b.att_age===null?'never':fmt(b.att_age,2)+' s', b.att_ok],
    ['Autopilot status', '/crsd/fcu_status age',
     f.age===null?'never':fmt(f.age,2)+' s', f.ok],
    ['World model', 'crsd/world_targets age',
     S.targets.age===null?'never':fmt(S.targets.age,2)+' s', S.targets.ok]
  ];
  rows.forEach(function(r){
    out += '<div class="row"><span class="nm">'+r[0]+'</span>'
        +  '<span class="meta">'+r[1]+'</span>'
        +  '<span style="min-width:80px;text-align:right;color:'
        +  (r[3]?'var(--ok)':'var(--bad)')+'">'+r[2]+'</span></div>';
  });
  el('p-tel').innerHTML = out;
}

/* ---------------- tabs 3 and 4: the viewers ----------------
   THE STREAM IS OPT-IN, per tab, and starts OFF.

   Tearing the iframe down on tab switch was already here, and it is not
   enough. The expensive case is the operator who WANTS the Camera tab open —
   to see whether the detector is up, to reach the Start buttons — and gets a
   live MJPEG stream with it. That stream is 640x400 JPEG at quality 60, 30
   fps: order 1 MB/s, against roughly 0.5 Mbps for everything else this page
   sends per client. Opening the tab was costing more than the whole rest of
   the dashboard by more than an order of magnitude, on a WiFi link whose
   rated speed is a close-range PHY number and whose real capacity at the far
   end of a course is the thing that matters.

   So the tab shows the viewer's STATUS for free and streams only when asked.
   mjpeg_server counts clients and skips encoding when nobody is attached, so
   an un-started stream costs the Jetson nothing either — the saving is on
   both ends of the link, not just ours.

   The page also stops streaming when the BROWSER tab goes to the background.
   A dashboard left open on a second monitor, or behind a chart window, is the
   normal state of an operator's laptop, and it was streaming the whole time. */
var streamOn = {cam:false, lidar:false}, pageHidden = false;

function toggleStream(which){
  streamOn[which] = !streamOn[which];
  render();
}

/* The three not-streaming states are the same panel with different words in
   it, so they are one function. Three copies of this markup is how one of them
   ends up a different size from the others after somebody adjusts one.
   Arguments are HTML, not text: two of the three callers interpolate markup,
   so escaping is the caller's job and doing it here would double-escape. */
function viewerPanel(title, body, buttons){
  return '<div class="viewer"><div class="empty">'
       + '<div style="font-size:15px;color:var(--fg);margin-bottom:6px">'
       + title + '</div>'
       + '<div style="margin-bottom:14px">' + body + '</div>'
       + (buttons || '') + '</div></div>';
}

function renderViewer(pane, cfg, tabName){
  var e = el(pane);
  /* Tear the iframe down so the MJPEG connection actually closes — a hidden
     one keeps streaming and the Jetson keeps encoding for nobody.
     dataset.src MUST be cleared with it. Leaving it set made the guard below
     match on the way back in, so the pane was never rebuilt: the tab was
     blank from the SECOND visit onwards while the viewer's own URL worked
     perfectly, which points the finger at the viewer instead of at this. */
  if(tab !== tabName){
    if(e.dataset.src !== ''){ e.innerHTML = ''; e.dataset.src = ''; }
    return;
  }
  var live = streamOn[tabName] && !pageHidden;
  var url = (cfg && cfg.source && live)
    ? location.protocol+'//'+location.hostname+':'+cfg.port+'/' : '';
  /* The guard keys on the whole rendered STATE, not just the url. There are
     four states now and three of them have no url — keying on the url alone
     left the tab showing "not running" with Start buttons after the node had
     started, because both states compared equal and the early return skipped
     the rebuild. */
  var key = url || (cfg && cfg.source ? 'paused:' + cfg.source
                 : cfg && cfg.starting ? 'starting:' + cfg.starting_name
                 : 'idle');
  if(e.dataset.src === key) return;                  /* unchanged: leave it */
  e.dataset.src = key;
  if(url){
    /* An iframe of the viewer's OWN page, not a bare <img> of its stream: the
       two servers publish different stream paths (buoy_detector streams from
       any non-root path, mjpeg_server routes /stream/<view>), and lidar_view's
       page carries its own plan/elevation tabs worth keeping. */
    e.innerHTML = '<div class="viewer" style="position:relative;padding:0">'
      + '<div style="position:absolute;top:8px;right:8px;z-index:5">'
      + '<button class="on" onclick="toggleStream(\''+tabName+'\')">'
      + '&#9632; stop stream</button></div>'
      + '<iframe src="'+url+'" style="width:100%;'
      + 'height:100%;border:0;background:var(--bg)"></iframe></div>';
  } else if(cfg && cfg.source){
    /* Running and reachable, deliberately not being watched. This panel is the
       whole point of the feature, so it says what it is costing you to press
       the button rather than just offering it. */
    e.innerHTML = viewerPanel(
      esc(cfg.source) + ' is running on :' + cfg.port,
      'The stream is <b>off</b>. Showing it pulls live MJPEG &mdash; '
        + '640&times;400 at quality 60, 30 fps, order 1 MB/s. That is more '
        + 'than twenty times everything else this page sends, so it stays off '
        + 'until you ask for it, and stops again when this browser tab goes to '
        + 'the background.',
      '<button class="go" onclick="toggleStream(\''+tabName+'\')">'
        + '&#9654; Show stream</button>');
  } else if(cfg && cfg.starting){
    /* Running, but its port is not answering yet. buoy_detector spends ten to
       thirty seconds loading a TensorRT engine and opening the OAK-D before it
       binds; the server withholds `source` until a connect succeeds, so the
       iframe is created once and works, rather than being created early,
       refused, and then never reloaded. */
    e.innerHTML = viewerPanel(
      esc(cfg.starting_name) + ' is starting…',
      'Its process is up; the stream server has not bound port ' + cfg.port
        + ' yet. The camera and the model take a while to load — this will '
        + 'switch to the live view on its own.');
  } else {
    var btns = (cfg && cfg.candidates || []).map(function(c){
      return '<button class="go" onclick="startNode(\''+c.name+'\')">Start '
           + esc(c.label)+'</button>'; }).join(' ');
    e.innerHTML = viewerPanel(esc(cfg ? cfg.title : 'Viewer not running'),
                              esc(cfg ? cfg.hint : ''), btns);
  }
}

/* ---------------- tab 6: system ---------------- */
function renderSys(){
  var s = S.system, p = S.power, out = '<div class="grid">';
  function card(k,v){ return '<div class="card"><div class="k">'+k+'</div>'
    + '<div class="v">'+v+'</div></div>'; }
  out += card('CPU', s.cpu_percent===null?'—':fmt(s.cpu_percent,0)+'%');
  out += card('Temperature', s.temp_c===null?'—':fmt(s.temp_c,1)+'°C');
  out += card('Memory', s.mem_total_gb===null?'—':
              fmt(s.mem_used_gb,1)+' / '+fmt(s.mem_total_gb,1)+' GB');
  out += card('Disk free', s.disk_total_gb===null?'—':
              fmt(s.disk_free_gb,0)+' / '+fmt(s.disk_total_gb,0)+' GB');
  out += card('Uptime', s.uptime_text||'—');
  out += card('Host time', esc(s.host_time||'—'));
  out += '</div><h3>Power</h3>';

  if(!p.available){
    out += '<div class="lock"><div style="color:var(--dangerfg);margin-bottom:6px">'
        +  'Power helper unreachable</div><div class="hint">'+esc(p.reason)
        +  '</div></div>';
  } else if(p.locked){
    out += '<div class="lock"><div style="color:var(--dangerfg);margin-bottom:8px">'
        +  'Locked — '+esc(p.lock_reason)+'</div>'
        +  '<button class="danger" disabled>Shut down</button> '
        +  '<button class="danger" disabled>Reboot</button></div>';
  } else {
    out += '<div class="hint">Type <b>'+esc(p.hostname)+'</b> to confirm. '
        +  'Two words and a click is not enough friction for a button that '
        +  'ends the session.</div>'
        +  '<div style="display:flex;gap:6px;margin-top:8px;max-width:520px">'
        +  '<input id="confirm" placeholder="'+esc(p.hostname)+'" style="flex:1">'
        +  '<button class="danger" onclick="power(\'shutdown\')">Shut down</button>'
        +  '<button class="danger" onclick="power(\'reboot\')">Reboot</button></div>';
  }
  el('p-sys').innerHTML = out;
}

function power(verb){
  var input = el('confirm');
  var typed = input ? input.value.trim() : '';
  if(!typed){ toast('Type the hostname first', true); return; }
  if(!confirm(verb.toUpperCase()+' the Jetson now?')) return;
  post('/power', {verb:verb, confirm:typed});
}

/* ---------------- tab 6: record ----------------
   A session is three things written into one directory: the telemetry
   JSONL this node samples, the MJPEG frames pulled from whichever viewer is
   up, and — new — a real rosbag of whichever TOPICS were ticked.

   The three are not redundant. The bag is the replayable one, and it is the
   only one that can carry the point cloud or the raw detections. The frames
   are the only record of what the operator was actually LOOKING AT, and the
   bag cannot replace them: buoy_detector publishes no image topic unless
   publish_frames is on, and that is off because it costs 38 MB/s on the DDS
   bus. The JSONL is the one that still parses after a power cut mid-line.

   SELECTION STATE LIVES IN JAVASCRIPT, NOT IN THE DOM. This pane repaints on
   every poll, five times a second; checkboxes whose truth was the `checked`
   attribute would be wiped on the next tick, which reads as the page ignoring
   your clicks. The topic list is therefore painted only when it or the
   selection actually changes, and the status block — which is what needs to
   be live — is the only thing on the 5 Hz path. */
var recTopics = null, recSel = {}, recBusy = false, recErr = '';

function recLoadTopics(){
  if(recBusy) return;
  recBusy = true;
  fetch('/record/topics', {method:'POST',
                           headers:{'Content-Type':'application/json'},
                           body:'{}'})
    .then(function(r){ return r.json(); })
    .then(function(j){
      recBusy = false;
      recErr = j.ok ? '' : (j.message || 'no answer');
      recTopics = j.ok ? (j.topics || []) : [];
      /* First load picks everything that is neither heavy nor noise: pressing
         Start without thinking about it gets the whole ROS picture at a few
         hundred KB a minute, and adding the point cloud stays a deliberate
         act. Only on the FIRST load — a reload must not undo your ticks. */
      if(!Object.keys(recSel).length)
        recTopics.forEach(function(t){
          if(!t.heavy && !t.noise) recSel[t.name] = true; });
      paintRecTopics();
    })
    .catch(function(err){
      recBusy = false; recErr = 'request failed: ' + err;
      recTopics = []; paintRecTopics();
    });
}
function recPreset(kind){
  (recTopics||[]).forEach(function(t){
    recSel[t.name] = kind === 'all' ? true
                   : kind === 'none' ? false
                   : (!t.heavy && !t.noise);
  });
  paintRecTopics();
}
function recToggle(name){ recSel[name] = !recSel[name]; paintRecTopics(); }
function recSelected(){
  return (recTopics||[]).filter(function(t){ return recSel[t.name]; })
                        .map(function(t){ return t.name; });
}

function paintRecTopics(){
  var box = el('rectopics');
  if(!box) return;
  if(recErr){
    box.innerHTML = '<h3>Topics</h3><div class="hint stale">'
                  + esc(recErr) + '</div>';
    return;
  }
  if(recTopics === null){
    box.innerHTML = '<h3>Topics</h3><div class="hint">reading the graph…</div>';
    return;
  }
  var chosen = recSelected(), heavy = 0;
  (recTopics||[]).forEach(function(t){ if(recSel[t.name] && t.heavy) heavy++; });
  var out = '<h3>Topics to record</h3>'
    + '<div class="hint">Straight from the live ROS graph, not a curated list '
    + '&mdash; a recording is worth making because something unexpected '
    + 'happened, and the curated list is the judgement that turns out to be '
    + 'wrong on the day. Written as a real rosbag, so <code>ros2 bag play</code> '
    + 'replays it.</div>'
    + '<div style="display:flex;gap:6px;align-items:center;margin:8px 0;'
    + 'flex-wrap:wrap">'
    + '<button onclick="recPreset(\'light\')">all but the heavy sensors</button>'
    + '<button onclick="recPreset(\'all\')">everything</button>'
    + '<button onclick="recPreset(\'none\')">none</button>'
    + '<button onclick="recLoadTopics()">Reload</button>'
    /* ONE style attribute. Two on the same element and the browser keeps the
       first, so the heavy-topic warning colour would silently never appear. */
    + '<span class="meta" style="margin-left:auto'
    + (heavy ? ';color:var(--warn)' : '') + '">'
    + chosen.length + ' of ' + recTopics.length + ' selected'
    + (heavy ? ' \u00b7 ' + heavy + ' HEAVY' : '') + '</span></div>';
  if(!recTopics.length)
    out += '<div class="hint">nothing is publishing</div>';
  recTopics.forEach(function(t){
    var flag = t.heavy
      ? '<span class="meta" style="color:var(--warn)">heavy &mdash; MB per '
        + 'second, not KB</span>'
      : (t.noise ? '<span class="meta">noise; off by default</span>' : '');
    out += '<div class="row">'
        + '<input type="checkbox" data-topic="' + esc(t.name) + '"'
        + (recSel[t.name] ? ' checked' : '') + ' style="width:15px;height:15px">'
        + '<span class="nm">' + esc(t.name) + '</span>'
        + '<span class="meta" style="flex:1">' + esc(t.type) + '</span>'
        + flag + '</div>';
  });
  box.innerHTML = out;
  box.querySelectorAll('input[data-topic]').forEach(function(x){
    x.onclick = function(){ recToggle(x.dataset.topic); }; });
}

function renderRec(){
  if(!el('recstatus')){
    el('p-rec').innerHTML = '<div id="recstatus"></div><div id="rectopics"></div>';
    recLoadTopics();
  }
  paintRecStatus();
}

function paintRecStatus(){
  var r = S.record, live = r.live, bag = r.bag || {}, out = '';
  out += '<h3>Session</h3>';
  if(live && live.recording){
    out += '<div class="card" style="border:1px solid var(--warn)">'
        +  '<div class="k">recording ' + esc(live.name) + '</div><div class="v">'
        +  ago(live.elapsed_s) + ' &middot; ' + live.samples + ' samples &middot; '
        +  fmt(live.size_mb,1) + ' MB</div>'
        +  '<div class="hint">frames: ' + esc(JSON.stringify(live.frames))
        +  ' &middot; ' + fmt(live.free_gb,1) + ' GB free</div>'
        +  (live.error ? '<div class="hint stale">' + esc(live.error) + '</div>' : '')
        +  '</div>';
    if(bag.recording){
      /* MEASURED, not estimated. The rate comes from the bag directory
         actually growing, and hours-left is a blank until there are two
         samples far enough apart to divide — a reassuring number computed
         from no data is the thing this repo keeps designing out. */
      out += '<div class="card" style="margin-top:8px;border:1px solid var(--warn)">'
          +  '<div class="k">bag &middot; ' + bag.topics.length + ' topics</div>'
          +  '<div class="v">' + fmt(bag.size_mb,1) + ' MB'
          +  (bag.rate_mb_s === null || bag.rate_mb_s === undefined ? ''
              : ' &middot; ' + fmt(bag.rate_mb_s,2) + ' MB/s') + '</div>'
          +  '<div class="hint">'
          +  (bag.hours_left === null || bag.hours_left === undefined
              ? 'measuring the growth rate…'
              : 'disk full in about <b>' + fmt(bag.hours_left,1)
                + ' h</b> at this rate')
          +  '</div>'
          +  (bag.error ? '<div class="hint stale">' + esc(bag.error) + '</div>' : '')
          +  '</div>';
    }
    out += '<div style="margin-top:8px">'
        +  '<button class="danger" onclick="recStop()">Stop recording</button></div>';
  } else {
    var srcs = r.sources.length ? r.sources.join(' + ')
             : 'telemetry only \u2014 no viewer is running, so there are no frames to record';
    var pw = r.persist && r.persist.persists
      ? '<span style="color:var(--ok)">persists</span> on the host via '
        + esc(r.persist.mount)
      : '<span class="stale">WILL NOT SURVIVE the container being recreated</span>';
    out += '<div class="hint">Writes to ' + esc(r.dir) + ' on the Jetson, not '
        +  'over the link: a recording has to survive the link dropping, which '
        +  'is exactly when you want it. Download once you are back alongside.</div>'
        +  '<div class="hint">Storage: ' + pw + '.</div>'
        +  '<div class="hint" style="margin-top:6px">Will capture: <b>' + esc(srcs)
        +  '</b> at ' + fmt(r.frame_hz,2) + ' fps, telemetry at '
        +  fmt(r.telemetry_hz,1) + ' Hz, plus <b>' + recSelected().length
        +  '</b> topic(s) as a rosbag. Stops itself below '
        +  fmt(r.min_free_gb,1) + ' GB free.</div>'
        +  (bag.error ? '<div class="hint stale">last bag: ' + esc(bag.error)
                      + '</div>' : '')
        +  '<div style="margin-top:8px">'
        +  '<button class="go" onclick="recStart()">Start recording</button></div>';
  }
  out += '<h3>Sessions</h3>';
  if(!r.sessions.length){ out += '<div class="hint">nothing recorded yet</div>'; }
  r.sessions.forEach(function(s){
    var busy = s.recording ? ' disabled' : '';
    out += '<div class="row"><span class="dot' + (s.recording?' up':'') + '"></span>'
        +  '<span class="nm">' + esc(s.name) + '</span>'
        +  '<span class="meta">' + (s.samples==null ? '' : s.samples+' samples') + '</span>'
        +  '<span class="meta">' + fmt(s.size_mb,1) + ' MB</span>'
        +  '<a href="/record/download?name=' + encodeURIComponent(s.name) + '">'
        +  '<button' + busy + '>Download</button></a> '
        +  '<button class="danger"' + busy + ' data-del="' + esc(s.name) + '">Delete</button>'
        +  '</div>';
    if(s.error) out += '<div class="hint stale">' + esc(s.error) + '</div>';
  });
  el('recstatus').innerHTML = out;
  document.querySelectorAll('#recstatus button[data-del]').forEach(function(b){
    b.onclick = function(){ delSession(b.dataset.del); };
  });
}
function recStart(){ post('/record/start', {topics: recSelected()}); }
function recStop(){ post('/record/stop', {}); }
function delSession(name){
  if(!confirm('Delete recording ' + name + '? This cannot be undone.')) return;
  post('/record/delete', {name: name});
}

/* ---------------- tab 7: logs ----------------
   Incremental: the page sends the newest seq it holds and gets back only what
   is new. Resending the whole ring five times a second would cost more than
   every other tab combined, and a log buffer is far larger than the trail. */
function pollLogs(){
  if(tab !== 'logs' || logBusy) return;
  logBusy = true;
  fetch('/logs', {method:'POST', headers:{'Content-Type':'application/json'},
                  body: JSON.stringify({since:logSeq, level:logLevel, node:logNode})})
    .then(function(r){ return r.json(); })
    .then(function(j){
      logBusy = false;
      if(!j.records) return;
      logSeq = j.newest;
      logRows = logRows.concat(j.records).slice(-800);
      paintLogs(j.dropped);
    })
    .catch(function(){ logBusy = false; });
}
function paintLogs(dropped){
  var body = el('logbody');
  if(!body) return;
  var atBottom = body.scrollTop + body.clientHeight >= body.scrollHeight - 30;
  var colors = {DEBUG:'var(--muted)', INFO:'var(--fg)', WARN:'var(--warn)',
                ERROR:'var(--bad)', FATAL:'var(--bad)'};
  body.innerHTML = logRows.map(function(r){
    var d = new Date(r.t*1000).toTimeString().slice(0,8);
    return '<div><span style="color:var(--muted)">' + d + '</span> '
         + '<span style="color:' + (colors[r.level_name]||'var(--fg)') + '">'
         + r.level_name + '</span> '
         + '<span style="color:var(--accentline)">' + esc(r.node) + '</span> '
         + esc(r.msg) + '</div>';
  }).join('') || '<div style="color:var(--muted)">nothing on /rosout yet</div>';
  /* Only autoscroll when the operator was already at the bottom. Yanking the
     view down while they are reading an error is worse than not following. */
  if(atBottom) body.scrollTop = body.scrollHeight;
  var d = el('logdrop');
  if(d) d.textContent = dropped ? (dropped + ' lines dropped (buffer full)') : '';
}
function renderLogs(){
  if(el('logbody')) return;            /* built once, then painted in place */
  var counts = S.logs.counts, chips = '';
  ['ERROR','WARN','INFO','DEBUG'].forEach(function(k){
    if(counts[k]) chips += '<span class="pill" style="margin-right:6px">'
                         + k + ' ' + counts[k] + '</span>';
  });
  var sel = '';                       /* select is styled in the stylesheet */
  el('p-logs').innerHTML =
      '<div class="hint">Everything on <b>/rosout</b> \u2014 every node in the '
    + 'DDS domain, including other containers. Not journalctl: the host journal '
    + 'is on the far side of the container boundary, and reaching it would mean '
    + 'a mount made purely to read a log.</div>'
    + '<div style="display:flex;gap:6px;align-items:center;margin:8px 0;flex-wrap:wrap">'
    + '<label class="meta">level</label>'
    + '<select id="loglevel" style="' + sel + '">'
    + '<option value="10">DEBUG</option><option value="20" selected>INFO</option>'
    + '<option value="30">WARN</option><option value="40">ERROR</option></select>'
    + '<label class="meta">node</label>'
    + '<select id="lognode" style="' + sel + '"><option value="">all</option>'
    + S.logs.nodes.map(function(n){ return '<option>' + esc(n) + '</option>'; }).join('')
    + '</select>'
    + '<button id="logclear">Clear</button>'
    + '<span id="logdrop" class="meta stale" style="margin-left:auto"></span>'
    + chips + '</div>'
    + '<pre id="logbody" style="max-height:none;height:calc(100vh - 190px)"></pre>';
  el('loglevel').onchange = function(){
    logLevel = parseInt(this.value, 10); logRows = []; logSeq = 0; pollLogs(); };
  el('lognode').onchange = function(){
    logNode = this.value; logRows = []; logSeq = 0; pollLogs(); };
  el('logclear').onclick = function(){
    post('/logs/clear', {}); logRows = []; logSeq = 0; paintLogs(0); };
  paintLogs(0);
  pollLogs();
}

/* ---------------- tab: tuning ----------------
   The knobs that matter are found on the water, and the alternative to this
   tab is an SSH session on the same laptop that is already showing the map.

   NOTHING HERE KNOWS WHAT ANY PARAMETER IS. Rows are built from the node's own
   descriptors, which carry the description, the range and the read_only
   posture, so a knob appears in this list because a node declares it and never
   because this page was edited to match. That is also why read-only parameters
   are LISTED rather than hidden: a knob you cannot turn and a knob you cannot
   see are different problems, and the second one sends somebody looking for it
   in the wrong file.

   A set lands in the running node and dies with it. crusader_params.yaml is
   the source of truth, so the drift marker is the real product of a tuning
   session: it is the list of lines to write back into the file. */
var tuneNode = '', tuneRows = [], tuneBusy = false, tuneErr = '', tuneKey = '';

function tuneFmt(v){
  if(v === null || v === undefined) return '—';
  if(typeof v === 'boolean') return v ? 'true' : 'false';
  if(Object.prototype.toString.call(v) === '[object Array]') return '[' + v.join(', ') + ']';
  return String(v);
}
/* Floats survive a YAML load, a ROS wire hop and a JSON encode, so an exact
   comparison would mark 3.0 as drifted from 3.0 often enough to make the
   marker itself untrustworthy. */
function tuneDrift(p){
  if(!p.in_yaml) return false;
  if(typeof p.value === 'number' && typeof p['default'] === 'number')
    return Math.abs(p.value - p['default']) > 1e-9;
  return p.value !== p['default'];
}

function tuneLoad(node){
  if(node) tuneNode = node;
  if(!tuneNode || tuneBusy) return;
  tuneBusy = true;
  var want = tuneNode;
  fetch('/params/list', {method:'POST', headers:{'Content-Type':'application/json'},
                         body: JSON.stringify({node: want})})
    .then(function(r){ return r.json(); })
    .then(function(j){
      tuneBusy = false;
      /* The selector moved while this was in flight: painting the answer would
         label one node's values with another node's name. */
      if(want !== tuneNode) return;
      tuneErr = j.ok ? '' : (j.message || 'no answer');
      tuneRows = j.ok ? (j.params || []) : [];
      paintTune();
    })
    .catch(function(err){
      tuneBusy = false; tuneErr = 'request failed: ' + err;
      tuneRows = []; paintTune();
    });
}

function tuneRowOf(name){
  return tuneRows.filter(function(p){ return p.name === name; })[0];
}
function tuneApply(name){
  var p = tuneRowOf(name), box = el('tv_' + name), v;
  if(!p || !box) return;
  if(p.type === 'bool') v = box.checked;
  else if(p.type === 'string') v = box.value;
  else {
    v = Number(box.value);
    /* Caught here only to stop an empty box being sent as 0, which the node
       would accept. The RANGE is deliberately not checked here: the node owns
       that rule and refuses in its own words, and a second copy of the bounds
       in this page is a copy that goes stale. */
    if(box.value === '' || !isFinite(v)){ toast(name + ': not a number', true); return; }
  }
  tunePost(name, v);
}
function tuneRevert(name){
  var p = tuneRowOf(name);
  if(p) tunePost(name, p['default']);
}
function tunePost(name, v){
  var body = {node: tuneNode, values: {}};
  body.values[name] = v;
  fetch('/params/set', {method:'POST', headers:{'Content-Type':'application/json'},
                        body: JSON.stringify(body)})
    .then(function(r){ return r.json(); })
    .then(function(j){
      toast(j.message || (j.ok ? 'applied' : 'refused'), !j.ok);
      /* Re-read rather than assume. A node may accept a set and clamp it, and
         the value worth showing is the one it is now running on. */
      tuneLoad();
    })
    .catch(function(err){ toast('request failed: ' + err, true); });
}

function tuneControl(p){
  var id = 'tv_' + p.name;
  if(!p.editable)
    return '<span class="meta" style="min-width:120px;text-align:right">'
         + esc(tuneFmt(p.value)) + '</span>';
  var set = ' <button data-apply="' + esc(p.name) + '">Set</button>';
  if(p.type === 'bool')
    return '<input type="checkbox" id="' + esc(id) + '"'
         + (p.value ? ' checked' : '') + ' style="width:15px;height:15px">' + set;
  if(p.type === 'string')
    return '<input id="' + esc(id) + '" value="' + esc(p.value)
         + '" style="width:180px">' + set;
  return '<input type="number" id="' + esc(id) + '" value="' + esc(p.value)
       + '" step="' + (p.type === 'integer' ? '1' : 'any') + '"'
       + (p.lo === null ? '' : ' min="' + p.lo + '"')
       + (p.hi === null ? '' : ' max="' + p.hi + '"')
       + ' style="width:96px">' + set;
}

function tuneRow(p){
  var note = esc(p.description || '');
  if(p.lo !== null && p.hi !== null)
    note += ' <span style="color:var(--muted)">[' + p.lo + ', ' + p.hi + ']</span>';
  /* The standard read-only reason is already the heading of that section; only
     an unusual one (an array this page will not edit) is worth repeating
     against the row itself. */
  if(p.reason && !p.read_only)
    note += ' <span style="color:var(--muted)">&middot; ' + esc(p.reason) + '</span>';
  var mark = '';
  if(tuneDrift(p))
    mark = '<span class="meta" style="color:var(--warn)">yaml '
         + esc(tuneFmt(p['default'])) + '</span>'
         + '<button data-revert="' + esc(p.name) + '">revert</button>';
  else if(!p.in_yaml)
    mark = '<span class="meta">not in the YAML</span>';
  return '<div class="row" style="flex-wrap:wrap">'
       + '<span class="nm" style="flex:0 0 200px">' + esc(p.name) + '</span>'
       + '<span class="meta" style="flex:1;min-width:190px">' + note + '</span>'
       + mark + tuneControl(p) + '</div>';
}

function paintTune(){
  var b = el('tuneBody');
  if(!b) return;
  var drifted = tuneRows.filter(tuneDrift).length, note = el('tunenote');
  /* Written BEFORE the early returns. Leaving the previous node's count up
     while the body says "no parameters" puts two nodes in one panel, which is
     the same convincing-but-wrong readout the rest of this page is built to
     avoid. */
  if(note){
    note.textContent = !tuneRows.length ? ''
      : drifted ? (drifted === 1 ? '1 value differs from the YAML'
                                 : drifted + ' values differ from the YAML')
                : tuneRows.length + ' parameters';
    note.style.color = drifted ? 'var(--warn)' : 'var(--dim)';
  }
  if(tuneErr){ b.innerHTML = '<div class="hint stale">' + esc(tuneErr) + '</div>'; return; }
  if(!tuneRows.length){
    b.innerHTML = '<div class="hint">'
      + (tuneBusy ? 'reading the node…' : 'this node declares no parameters')
      + '</div>';
    return;
  }
  var dyn = tuneRows.filter(function(p){ return p.editable; });
  var fixed = tuneRows.filter(function(p){ return !p.editable; });

  b.innerHTML = '<h3>Tunable while running</h3>'
    + '<div class="hint">Applied to the running node immediately and <b>lost on '
    + 'restart</b>. crusader_params.yaml is the source of truth; a row showing '
    + '<span style="color:var(--warn)">yaml &lt;value&gt;</span> is one to write '
    + 'back into it before the next run.</div>'
    + (dyn.map(tuneRow).join('') || '<div class="hint">none</div>')
    + '<h3>Fixed at startup</h3>'
    + '<div class="hint">Structural: topic names, mounting geometry, freshness '
    + 'budgets. Change them in crusader_params.yaml and restart the node. Shown '
    + 'rather than hidden because a knob you cannot turn and one you cannot find '
    + 'are different problems.</div>'
    + (fixed.map(tuneRow).join('') || '<div class="hint">none</div>');

  b.querySelectorAll('button[data-apply]').forEach(function(x){
    x.onclick = function(){ tuneApply(x.dataset.apply); }; });
  b.querySelectorAll('button[data-revert]').forEach(function(x){
    x.onclick = function(){ tuneRevert(x.dataset.revert); }; });
  /* Enter applies the row you are in. Reaching for the mouse after every number
     is the difference between sweeping a gate and giving up on it. */
  b.querySelectorAll('input').forEach(function(x){
    if(x.type === 'checkbox') return;
    x.onkeydown = function(ev){
      if(ev.key === 'Enter'){ ev.preventDefault(); tuneApply(x.id.slice(3)); } };
  });
}

function tuneFillSelect(nodes){
  var s = el('tunesel');
  if(!s) return;
  s.innerHTML = nodes.map(function(n){
    return '<option' + (n === tuneNode ? ' selected' : '') + '>' + esc(n)
         + '</option>'; }).join('');
}

function renderTune(){
  var nodes = (S.tuning && S.tuning.nodes) || [], key = nodes.join(',');
  /* Built once, then painted in place — a rebuild on every poll would wipe out
     whatever number is half-typed. Only the node list is refreshed, and only
     when it has actually changed, so a node started from the Nodes tab appears
     here without disturbing the panel. */
  if(el('tuneBody')){
    if(key !== tuneKey){ tuneKey = key; tuneFillSelect(nodes); }
    return;
  }
  tuneKey = key;
  if(!tuneNode)
    tuneNode = nodes.indexOf('/target_tracker') >= 0 ? '/target_tracker'
                                                     : (nodes[0] || '');
  var sel = 'max-width:260px';         /* rest is styled in the stylesheet */
  el('p-tune').innerHTML =
      '<div class="hint">Every row below is read from the node itself — its '
    + 'own parameter descriptors carry the description, the range and whether '
    + 'the value may change while running. Nothing appears here because this '
    + 'page was told about it.</div>'
    + '<div style="display:flex;gap:6px;align-items:center;margin:8px 0;flex-wrap:wrap">'
    + '<label class="meta">node</label>'
    + '<select id="tunesel" style="' + sel + '"></select>'
    + '<button id="tunereload">Reload</button>'
    + '<span class="meta" id="tunenote" style="margin-left:auto"></span></div>'
    + '<div id="tuneBody"></div>';
  tuneFillSelect(nodes);
  el('tunesel').onchange = function(){
    tuneRows = []; tuneErr = ''; paintTune(); tuneLoad(this.value); };
  el('tunereload').onclick = function(){ tuneLoad(); };
  paintTune();
  tuneLoad();
}

/* ---------------- tab 5: the map ----------------
   THREE THINGS ARE DRAWN AND THEY ARE DELIBERATELY NOT THE SAME THING.

     targets   crsd/world_targets  — what the tracker BELIEVES, after fusion,
                                     association and decay.
     clusters  crsd/lidar_clusters — what the LiDAR actually RETURNED this
                                     window, before any of that.
     PRX1      crsd/obstacle_distance — what the autopilot was TOLD, the same
                                     72 sectors telemetry_bridge forwards as
                                     MAVLink OBSTACLE_DISTANCE.

   Laying the three over one another is the whole point: it turns "avoidance
   is behaving oddly" into a question with an answer. If PRX1 here matches
   QGC's proximity view and the clusters under it do not, the bug is in
   proximity_bridge's sector maths. If PRX1 here and QGC disagree, the fault
   is between this ROS graph and the flight controller. If the clusters agree
   with both but the targets are somewhere else, it is the tracker.

   Both extra layers are OPT-IN and off by default, because with a shoreline in
   view the cluster layer is a few KB per poll — an order of magnitude more
   than the rest of /state. The buttons put that cost under the operator's
   thumb rather than spending it on everyone all the time. */
var cv = el('c'), cx = cv.getContext('2d');
var scale = 6, follow = true, view = {x:0,y:0}, drag = null, W = 0, H = 0;
var layers = {clusters:false, prox:false}, bowUp = false, viewRot = 0;
var NO_READING = 65535;              /* ObstacleDistance.msg: not seen */

function toggleLayer(name){
  layers[name] = !layers[name];
  el(name === 'clusters' ? 'b_clusters' : 'b_prox').className =
    layers[name] ? 'on' : '';
  poll();                            /* the query string just changed */
}
function toggleBow(){
  bowUp = !bowUp;
  el('b_bow').className = bowUp ? 'on' : '';
  /* Bow-up without follow is a map that rotates about a point the boat is not
     at, which is disorienting in the one mode meant to reduce confusion. */
  if(bowUp && !follow) toggleFollow();
  draw();
}

function colorOf(label){
  var L = (label||'').toLowerCase();
  if(L.indexOf('red')===0) return PAL.red;
  if(L.indexOf('green')===0) return PAL.green;
  if(L.indexOf('yellow')===0) return PAL.yellow;
  if(L.indexOf('blue')===0) return PAL.blue;
  if(L.indexOf('black')===0) return PAL.black;
  return PAL.none;
}
/* World (east, north) metres -> screen pixels.

   Takes BOTH coordinates because bow-up is a rotation of the world about the
   boat, and a rotated axis cannot be computed from one of its components.
   Rotating the COORDINATE MAPPING rather than the canvas is deliberate: a
   canvas rotation would carry the text with it, and a bow-up map whose labels
   are sideways is unreadable at exactly the moment you are using it to read a
   range off an obstacle. */
function px(x, y){
  var dx = x - view.x, dy = y - view.y;
  if(viewRot){
    var c = Math.cos(viewRot), s = Math.sin(viewRot);
    var rx = dx*c - dy*s;
    dy = dx*s + dy*c; dx = rx;
  }
  return [W/2 + dx*scale, H/2 - dy*scale];
}
/* A compass bearing (degrees clockwise from true north) and a range, as a
   world offset. atan2's arguments are east-then-north everywhere in this
   file for the same reason; swapping them mirrors the map. */
function bearingXY(originX, originY, bearingDeg, r){
  var b = bearingDeg*Math.PI/180;
  return [originX + r*Math.sin(b), originY + r*Math.cos(b)];
}
function resize(){
  var r = cv.getBoundingClientRect(), d = window.devicePixelRatio||1;
  if(!r.width) return;
  W = r.width; H = r.height;
  cv.width = Math.round(W*d); cv.height = Math.round(H*d);
  cx.setTransform(d,0,0,d,0,0);
  draw();
}
window.addEventListener('resize', resize);
function zoom(k){ scale = Math.max(0.3, Math.min(80, scale*k)); draw(); }
function toggleFollow(){
  follow = !follow;
  el('b_follow').className = follow ? 'go' : '';
  draw();
}
cv.addEventListener('mousedown', function(e){ drag={x:e.clientX,y:e.clientY}; });
window.addEventListener('mouseup', function(){ drag=null; });
window.addEventListener('mousemove', function(e){
  if(!drag) return;
  if(follow) toggleFollow();
  /* The drag is in SCREEN pixels and view is in WORLD metres, so under bow-up
     the delta has to come back through the inverse rotation or the map slides
     off at an angle to the pointer. */
  var mx = (e.clientX-drag.x)/scale, my = (e.clientY-drag.y)/scale;
  if(viewRot){
    var c = Math.cos(-viewRot), s = Math.sin(-viewRot);
    var rx = mx*c - my*s; my = mx*s + my*c; mx = rx;
  }
  view.x -= mx; view.y += my;
  drag = {x:e.clientX,y:e.clientY}; draw();
});
cv.addEventListener('wheel', function(e){ e.preventDefault();
  zoom(e.deltaY<0?1.1:1/1.1); }, {passive:false});

function ringStep(){
  var t = 90/scale, p = Math.pow(10, Math.floor(Math.log(t)/Math.LN10)), m = t/p;
  return (m<2?1:m<5?2:5)*p;
}
/* One sector of OBSTACLE_DISTANCE, drawn where the autopilot thinks it is.

   A short tangential dash at the reported range rather than a filled wedge,
   which is both how QGC's PRX1 view renders it and the honest shape: the
   sector says "the nearest thing in these five degrees is at this range", not
   "everything from here outward is blocked". */
function drawSector(bx, by, bearingDeg, widthDeg, r){
  var a = bearingDeg - widthDeg/2, n = 4;
  cx.beginPath();
  for(var k=0; k<=n; k++){
    var w = bearingXY(bx, by, a + widthDeg*k/n, r), p = px(w[0], w[1]);
    if(k===0) cx.moveTo(p[0], p[1]); else cx.lineTo(p[0], p[1]);
  }
  cx.stroke();
}

function drawClusters(b){
  var cl = S.clusters;
  if(!cl || !cl.ok || !cl.placed) return 0;
  cx.strokeStyle = PAL.lidar; cx.fillStyle = PAL.lidar; cx.lineWidth = 1;
  (cl.items||[]).forEach(function(c){
    var p = px(c.wx, c.wy);
    /* The FOOTPRINT, not a point. Cluster3D.msg carries extent because
       avoidance needs a size, and a 6 m dock drawn as a dot next to a 0.3 m
       buoy drawn as the same dot is the picture that makes a hull look
       navigable. Axis-aligned in the body frame, so it rotates with the view
       only in the sense that its centre does — good enough to read a size off,
       and not worth pretending the box is oriented data it is not. */
    var w = Math.max(3, (c.ex||0.3)*scale), h = Math.max(3, (c.ey||0.3)*scale);
    cx.globalAlpha = 0.85;
    cx.strokeRect(p[0]-w/2, p[1]-h/2, w, h);
    cx.globalAlpha = 1;
    cx.beginPath(); cx.arc(p[0], p[1], 1.6, 0, 6.2832); cx.fill();
  });
  return (cl.items||[]).length;
}

function drawProx(b){
  var pr = S.prox;
  if(!pr || !pr.ok || !pr.d || !b.ok) return 0;
  cx.strokeStyle = PAL.prox; cx.lineWidth = 3;
  var inc = pr.increment || 5.0, off = pr.offset || 0.0, n = 0;
  for(var i=0;i<pr.d.length;i++){
    var cm = pr.d[i];
    /* NO READING is skipped, never drawn at max range. ObstacleDistance.msg
       is emphatic that "not seen" and "seen and clear" are different facts,
       and a ring of max-range arcs around the boat would render the whole
       unseen aft sector as a wall. */
    if(cm === NO_READING) continue;
    n++;
    /* Sector 0 is the NOSE and the index runs CLOCKWISE, so the world bearing
       is the boat's heading plus the sector's own. That is the same
       convention proximity_core encodes on the way out; reading it back the
       other way is what makes this layer a check rather than a decoration. */
    drawSector(b.x, b.y, b.heading + off + i*inc, inc, cm/100.0);
  }
  return n;
}

/* Nearest cluster as RANGE AND RELATIVE BEARING off the bow — the two numbers
   QGC's PRX1 panel shows, in the same frame, so they can be read side by side
   without arithmetic. Computed from the body-frame coordinates the server
   sent, not from the world ones, so it stays true even when it is the world
   projection that is wrong. */
function nearestCluster(){
  var cl = S.clusters, best = null;
  if(!cl || !cl.ok) return null;
  (cl.items||[]).forEach(function(c){
    if(best === null || c.r < best.r) best = c;
  });
  if(!best) return null;
  return {r: best.r, brg: ((Math.atan2(-best.y, best.x)*180/Math.PI)+360)%360};
}

function paintLegend(nClusters, nSectors){
  var g = el('maplegend');
  if(!g) return;
  var lines = [];
  if(layers.clusters){
    var cl = S.clusters, near = nearestCluster();
    lines.push(!cl || !cl.ok ? 'clusters  STALE — lidar_cluster_node is not publishing'
      : !cl.placed ? 'clusters  ' + (cl.items||[]).length +
                     ' held, NOT PLACED — pose or attitude stale'
      : 'clusters  ' + nClusters + (near ? '   nearest ' + near.r.toFixed(1)
          + ' m at ' + near.brg.toFixed(0) + '\u00b0 rel' : ''));
  }
  if(layers.prox){
    var pr = S.prox;
    lines.push(!pr || !pr.ok
      ? 'PRX1      STALE — proximity_bridge is not publishing'
      : 'PRX1      ' + nSectors + '/' + (pr.d ? pr.d.length : 0) +
        ' sectors read   ' + (pr.increment||0) + '\u00b0 each');
  }
  if(bowUp) lines.push('view      BOW-UP (north is off-screen up-arrow)');
  g.textContent = lines.join('\n');
}

function draw(){
  if(!W || !S) return;
  cx.fillStyle = PAL.bg; cx.fillRect(0,0,W,H);
  var b = S.boat;
  if(follow && b.ok){ view.x = b.x; view.y = b.y; }
  /* Set BEFORE anything is projected: px() reads it, so a mid-frame change
     would draw half the scene in each frame. */
  viewRot = (bowUp && b.ok) ? b.heading*Math.PI/180 : 0;
  if(b.ok){
    var step = ringStep(), maxr = Math.hypot(W,H)/scale;
    var bp = px(b.x, b.y);
    cx.strokeStyle = PAL.ring; cx.fillStyle = PAL.ringfg; cx.lineWidth = 1;
    cx.font = '11px ui-monospace,monospace';
    for(var r=step; r<=maxr; r+=step){
      cx.beginPath(); cx.arc(bp[0], bp[1], r*scale, 0, 6.2832); cx.stroke();
      cx.fillText(r+' m', bp[0]+4, bp[1]-r*scale-3);
    }
  }
  var tr = S.trail;
  if(tr && tr.length>1){
    cx.strokeStyle = PAL.trail; cx.lineWidth = 2; cx.beginPath();
    var t0 = px(tr[0][0], tr[0][1]);
    cx.moveTo(t0[0], t0[1]);
    for(var i=1;i<tr.length;i++){
      var tp = px(tr[i][0], tr[i][1]); cx.lineTo(tp[0], tp[1]);
    }
    cx.stroke();
  }
  /* Under the targets on purpose: these are the raw input, and the tracker's
     answer is what a mission acts on, so the answer stays on top. */
  var nClusters = layers.clusters ? drawClusters(b) : 0;
  var nSectors = layers.prox ? drawProx(b) : 0;
  (S.targets.items||[]).forEach(function(t){
    var tpos = px(t.x, t.y), X = tpos[0], Y = tpos[1];
    var remembered = t.unseen > 2.0;
    var fade = Math.max(0.55, 1 - t.unseen/60);
    cx.globalAlpha = fade;
    cx.fillStyle = colorOf(t.label);
    cx.strokeStyle = t.confirmed ? PAL.conf : PAL.tent;
    cx.lineWidth = t.confirmed ? 1.5 : 1;
    cx.beginPath(); cx.arc(X,Y,6,0,6.2832); cx.fill();
    if(!t.confirmed) cx.setLineDash([2,2]);
    cx.stroke(); cx.setLineDash([]);
    if(t.stddev*scale > 8){
      cx.strokeStyle = colorOf(t.label); cx.globalAlpha = fade*0.35;
      cx.beginPath(); cx.arc(X,Y,t.stddev*scale,0,6.2832); cx.stroke();
      cx.globalAlpha = fade;
    }
    if(remembered){
      cx.strokeStyle = colorOf(t.label); cx.lineWidth = 1;
      cx.setLineDash([3,3]); cx.globalAlpha = fade*0.7;
      cx.beginPath(); cx.arc(X,Y,11,0,6.2832); cx.stroke();
      cx.setLineDash([]); cx.globalAlpha = fade;
    }
    cx.fillStyle = PAL.fg; cx.font = '11px ui-monospace,monospace';
    cx.fillText('#'+t.id+' '+(t.label||'unknown'), X+14, Y-4);
    if(b.ok) cx.fillText(fmt(t.range,1)+' m', X+14, Y+8);
    if(remembered){ cx.fillStyle = PAL.muted;
      cx.fillText('seen '+ago(t.unseen)+' ago', X+14, Y+20); }
    cx.globalAlpha = 1;
  });
  if(b.ok){
    var ap = px(b.x, b.y);
    cx.save(); cx.translate(ap[0], ap[1]);
    cx.rotate(b.heading*Math.PI/180 - viewRot);
    cx.fillStyle = b.ok ? PAL.boat : PAL.muted;
    cx.beginPath(); cx.moveTo(0,-12); cx.lineTo(7,9); cx.lineTo(0,4);
    cx.lineTo(-7,9); cx.closePath(); cx.fill(); cx.restore();
  }
  /* `barpx`, not `px` — that name is the projection function now, and `var`
     hoists, so a local of the same name made every px() call in this function
     a call on undefined. */
  var step2 = ringStep(), barpx = step2*scale, x0 = 14, y0 = H-20;
  cx.strokeStyle = PAL.dim; cx.lineWidth = 2; cx.beginPath();
  cx.moveTo(x0,y0); cx.lineTo(x0+barpx,y0);
  cx.moveTo(x0,y0-4); cx.lineTo(x0,y0+4);
  cx.moveTo(x0+barpx,y0-4); cx.lineTo(x0+barpx,y0+4); cx.stroke();
  cx.fillStyle = PAL.dim; cx.font = '11px ui-monospace,monospace';
  cx.fillText(step2+' m', x0+barpx+8, y0+4);
  /* The north arrow turns with the view. In bow-up it is the only thing on
     screen still telling you which way north is, so it is the one marker that
     must NOT be drawn at a fixed angle. */
  var nx = W-28, ny = 30;
  cx.save(); cx.translate(nx, ny); cx.rotate(-viewRot);
  cx.strokeStyle = PAL.dim; cx.fillStyle = PAL.dim; cx.lineWidth = 2;
  cx.beginPath(); cx.moveTo(0,12); cx.lineTo(0,-12); cx.stroke();
  cx.beginPath(); cx.moveTo(0,-16); cx.lineTo(-4,-8); cx.lineTo(4,-8);
  cx.closePath(); cx.fill();
  cx.restore();
  cx.fillStyle = PAL.dim;
  cx.fillText('N', nx-3, ny+30);
  paintLegend(nClusters, nSectors);
}

/* ---------------- render + poll ---------------- */
function render(){
  if(!S) return;
  el('rtt').textContent = rtt===null ? '— ms' : rtt.toFixed(0)+' ms';
  el('rtt').style.color = (rtt!==null && rtt<400) ? 'var(--ok)' : 'var(--bad)';
  el('armed').textContent = S.fcu.ok ? (S.fcu.armed?'ARMED':'disarmed') : 'no fcu';
  el('armed').style.color = S.fcu.ok && S.fcu.armed ? 'var(--warn)' : 'var(--dim)';

  if(tab==='nodes') renderNodes();
  if(tab==='tel') renderTel();
  renderViewer('p-cam', S.tabs.camera, 'cam');
  renderViewer('p-lidar', S.tabs.lidar, 'lidar');
  if(tab==='tune') renderTune();
  if(tab==='rec') renderRec();
  if(tab==='logs') renderLogs();
  if(tab==='sys' && !document.activeElement.matches('#confirm')) renderSys();
  if(tab==='map') draw();

  var msg = '';
  if(!S.boat.ok) msg = 'POSE STALE — vessel position is NOT current';
  else if(!S.boat.att_ok) msg = 'ATTITUDE STALE — target positions are uncompensated';
  var bn = el('banner');
  bn.textContent = msg; bn.style.display = msg ? 'block' : 'none';
}

/* One poll in flight at a time: setInterval fires on a wall clock and does not
   care whether the last request returned, so on a slow link requests pile up
   faster than they drain, hit the per-host connection cap, and freeze the page
   on stale data while the banner waits behind the queue. */
function poll(){
  if(inflight) return;
  inflight = true;
  var t0 = performance.now();
  /* The layer list rides in the query string, so a page with both layers off
     asks for exactly what it always did. */
  var on = [];
  if(layers.clusters) on.push('clusters');
  if(layers.prox) on.push('prox');
  var url = on.length ? '/state?layers=' + on.join(',') : '/state';
  fetch(url).then(function(r){ return r.json(); }).then(function(j){
    inflight = false; rtt = performance.now() - t0; S = j; render();
  }).catch(function(){
    inflight = false;
    var bn = el('banner');
    bn.textContent = 'NO CONNECTION TO THE BOAT'; bn.style.display = 'block';
  });
}
document.querySelectorAll('#bar button.tab').forEach(function(b){
  b.onclick = function(){ show(b.dataset.t); }; });
/* A dashboard behind a chart window, or on a second monitor the operator is
   not looking at, was streaming the whole time. The stream resumes on return
   because streamOn is untouched — this suspends it, it does not switch it
   off, so coming back to the tab does not need another click. */
document.addEventListener('visibilitychange', function(){
  pageHidden = document.hidden;
  render();
});
el('theme').onclick = function(){
  setTheme(document.documentElement.getAttribute('data-theme') === 'day'
           ? 'night' : 'day'); };
setTheme(document.documentElement.getAttribute('data-theme') === 'day'
         ? 'day' : 'night');
el('b_follow').className = 'go';
show('nodes');
poll();
setInterval(poll, POLL);
setInterval(pollLogs, 700);
</script></body></html>"""


def render(poll_ms: int) -> bytes:
    """The page, with the poll period baked in. UTF-8, and the server must say
    so in the Content-Type — degree signs and em-dashes render as mojibake
    under a browser's latin-1 fallback, and a readout that looks broken is a
    readout nobody trusts."""
    return PAGE.replace("__POLL_MS__", str(int(poll_ms))).encode("utf-8")
