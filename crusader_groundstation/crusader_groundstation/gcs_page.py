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
 /* The Task 1 pane is the lake panel in a frame, under a strip of its own
    controls and the warning about leaving. Padding 0 and a column, so the frame
    takes everything the strip does not. The stale-pose banner is moved to the
    bottom while it is up: at the top it would sit on the strip. */
 #p-task1.on{padding:0;height:100%;display:flex;flex-direction:column}
 #p-task1.on ~ #banner{top:auto;bottom:56px}
 #t1bar{display:flex;gap:6px;align-items:center;flex-wrap:wrap;padding:6px 10px;
   background:var(--panel);border-bottom:1px solid var(--line)}
 #t1warn{padding:5px 10px;font-size:12px;color:var(--warn);
   border-bottom:1px solid var(--line)}
 #t1rig{display:flex;gap:8px;align-items:center;flex-wrap:wrap;padding:6px 10px;
   background:var(--panel);border-bottom:1px solid var(--line)}
 #t1rig label{display:flex;gap:4px;align-items:center}
 #t1out{padding:0 10px;border-bottom:1px solid var(--line)}
 #t1log{max-height:180px;overflow:auto;font-size:11px;margin:4px 0;white-space:pre-wrap}
 #t1body{flex:1;min-height:0;position:relative;overflow:auto}
 #t1body iframe{display:block;width:100%;height:100%;border:0;background:var(--bg)}
 /* The controls sit BESIDE the picture, not on another tab. The whole reason
    the camera was four stops under for weeks is that nobody could see what a
    setting did while they were setting it; a knob and its result on two
    different screens is the same problem with extra steps. */
 #camview{flex:1;min-width:0;display:flex}
 #camtune{flex:0 0 340px;width:340px;overflow:auto;padding:10px 12px;
          border-left:1px solid var(--line);background:var(--panel)}
 #camtune .row{flex-wrap:wrap;gap:4px}
 #camtune .nm{flex:0 0 130px;font-size:12px}
 @media (max-width:820px){
   #p-cam.on{flex-direction:column}
   #camtune{flex:0 0 auto;width:auto;max-height:46%;
            border-left:0;border-top:1px solid var(--line)}
 }
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
 #navbadge{font-weight:600}
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
  <button class="tab" data-t="task1">Task 1</button>
  <button class="tab" data-t="tune">Tuning</button>
  <button class="tab" data-t="rec">Record</button>
  <button class="tab" data-t="logs">Logs</button>
  <button class="tab" data-t="radio">Radio</button>
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
    <div id="maplegend"><div id="navbadge"></div><span id="maplegendtxt"></span></div></div>
  </div>
  <div class="pane" id="p-task1"></div>
  <div class="pane" id="p-tune"></div>
  <div class="pane" id="p-rec"></div>
  <div class="pane" id="p-logs"></div>
  <div class="pane" id="p-radio"></div>
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
var radioSeq = 0, radioRows = [], radioBusy = false, radioStats = null;
var radioDir = '', radioWho = '', radioHb = false;

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
function toast(msg, bad, ms){
  var t = el('toast');
  t.textContent = msg;
  t.style.borderColor = bad ? 'var(--danger)' : 'var(--line)';
  t.style.display = 'block';
  clearTimeout(t._h);
  t._h = setTimeout(function(){ t.style.display='none'; }, ms || 4500);
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
  ['nodes','tel','cam','lidar','map','task1','tune','rec','logs','radio','sys'].forEach(function(p){
    el('p-'+p).className = 'pane' + (p===t ? ' on' : ''); });
  document.querySelectorAll('#bar button.tab').forEach(function(b){
    b.className = 'tab' + (b.dataset.t===t ? ' on' : ''); });
  /* Viewer tabs mount their <img> only while visible. mjpeg_server counts
     clients and skips rendering when nobody is watching, so an unopened camera
     tab genuinely costs the Jetson nothing — but only if the img is gone, not
     merely hidden. A display:none <img> keeps its connection open. */
  if(t!=='logs') el('p-logs').innerHTML = '';   /* rebuild with fresh nodes */
  if(t!=='radio') el('p-radio').innerHTML = '';  /* same: fresh system list */
  /* The Task 1 frame exists only while its tab is in front: see task1Leave. */
  if(t==='task1') task1Enter(); else task1Leave();
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
  /* The dock detector's own latency, from crsd/dock_view_health. Shown only
     while dock_view runs; a stale report reads "not running", never as numbers
     that were true a minute ago. Green is a rough budget for the Jetson, not a
     measured limit: detector under 100 ms, chain under 200, frame under 300. */
  var d = S.dock_latency;
  if(d && d.ok){
    rows.push(['Dock detector', 'YOLO only, mean / worst in 1 s',
               fmt(d.det_ms,0)+' / '+fmt(d.det_max_ms,0)+' ms', d.det_ms < 100]);
    rows.push(['Dock chain', 'geometry + colour + plane',
               fmt(d.chain_ms,0)+' ms', d.chain_ms < 200]);
    rows.push(['Dock frame total', 'frame in -> DockObservation out',
               fmt(d.total_ms,0)+' ms  ('+fmt(d.fps,1)+' fps)', d.total_ms < 300]);
  } else {
    rows.push(['Dock detector', 'crsd/dock_view_health',
               d && d.age!==null ? 'stale '+fmt(d.age,0)+' s' : 'not running', false]);
  }
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
  /* The chosen view rides in the URL, so switching it reloads the iframe and
     the old MJPEG connection closes with it -- the producer stops encoding the
     view nobody is looking at any more. */
  var view = viewChoice[tabName] || '';
  var url = (cfg && cfg.source && live)
    ? location.protocol+'//'+location.hostname+':'+cfg.port+'/'
      + (view ? '?view=' + encodeURIComponent(view) : '') : '';
  /* The guard keys on the whole rendered STATE, not just the url. There are
     four states now and three of them have no url — keying on the url alone
     left the tab showing "not running" with Start buttons after the node had
     started, because both states compared equal and the early return skipped
     the rebuild. */
  var key = url || (cfg && cfg.source ? 'paused:' + cfg.source + ':' + view
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
      + viewPicker(cfg, tabName)
      + '<div style="position:absolute;top:8px;right:8px;z-index:5">'
      + '<button class="on" onclick="toggleStream(\''+tabName+'\')">'
      + '&#9632; stop stream</button></div>'
      + '<iframe src="'+url+'" style="width:100%;'
      + 'height:100%;border:0;background:var(--bg)"></iframe></div>';
    /* Wired here rather than inline: the view name is data from the node, and
       an onclick attribute would put it through a second layer of quoting for
       nothing. */
    e.querySelectorAll('button[data-view]').forEach(function(btn){
      btn.onclick = function(){
        var parts = btn.dataset.view.split('|');
        setView(parts[0], parts[1]);
      };
    });
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
/* Which view each viewer tab is showing, by tab name. Empty means "whatever
   the producer serves by default", which is every viewer except the detector.
   Per tab, not global: the camera and the LiDAR do not offer the same views. */
var viewChoice = {};

function setView(tabName, name){
  viewChoice[tabName] = name;
  paint();
}

/* Buttons only when there is something to choose. One view is not a choice,
   and a picker with a single disabled button is furniture. */
function viewPicker(cfg, tabName){
  if(!cfg || !cfg.views || cfg.views.length < 2) return '';
  var cur = viewChoice[tabName] || cfg.views[0].name;
  return '<div style="position:absolute;top:8px;left:8px;z-index:2;display:flex;'
       + 'gap:6px">'
       + cfg.views.map(function(v){
           return '<button data-view="' + esc(tabName) + '|' + esc(v.name) + '"'
                + (v.name === cur ? ' class="go"' : '') + '>'
                + esc(v.label) + '</button>'; }).join('')
       + '</div>';
}

var recTopics = null, recSel = {}, recBusy = false, recErr = '';

/* Capture rate, PER VIEWER and PER SESSION. Sent with Start; it is not a
   parameter write, so the YAML default comes back on the next boot and a 30 fps
   afternoon does not become the boat's new normal.

   Lives in JavaScript for the same reason the checkboxes do, and one reason
   more: the status block above repaints five times a second, and a number box
   rebuilt under the cursor loses the digits you are part-way through typing.
   So this pane is painted only when the value changes or a recording starts. */
var recFps = null, recWasLive = null;
var FPS_PRESETS = [0.5, 1, 2, 5, 10, 30];

/* Both of these come from the snapshot, which reads them off the node's own
   PARAM_SPEC and FRAME_SOURCES — so the boxes and the node can never disagree
   about what is allowed or about which viewers exist. The fallbacks are only
   for the first paint, before any snapshot has landed. */
function recCfg(name, dflt){
  return (S.record && S.record[name]) || dflt;
}
function recFpsRange(){ return recCfg('frame_hz_range', [0.05, 60]); }
function recFpsKeys(){ return recCfg('frame_keys', ['camera', 'lidar']); }
function recSetFps(key, hz){
  var r = recFpsRange();
  hz = Number(hz);
  if(!isFinite(hz)) return;
  recFps[key] = Math.min(r[1], Math.max(r[0], hz));
  paintRecFps();
  paintRecStatus();
}
function recFpsSummary(){
  if(!recFps) return fmt(S.record.frame_hz, 2) + ' fps';
  var up = S.record.sources || [];
  if(!up.length) return 'no viewer';
  return up.map(function(k){
    return k + ' ' + fmt(recFps[k], 2) + ' fps'; }).join(', ');
}

function paintRecFps(){
  var box = el('recfps');
  if(!box || !S.record) return;
  var keys = recFpsKeys(), rng = recFpsRange();
  if(!recFps){
    recFps = {};
    keys.forEach(function(k){ recFps[k] = S.record.frame_hz; });
  }
  /* Both branches below open the row the same way — a lit dot when the thing
     exists, then the viewer's name. Written once so the two halves of this
     pane cannot drift into different-looking rows. */
  function rowHead(k, lit){
    return '<div class="row"><span class="dot' + (lit ? ' up' : '') + '">'
         + '</span><span class="nm">' + esc(k) + '</span>';
  }
  var live = S.record.live && S.record.live.recording ? S.record.live : null;
  var out = '<h3>Frame rate</h3>';
  if(live){
    /* The rates a running session is ACTUALLY pulling at, read back off the
       pullers by the node — not what this page asked for, which would still
       say 30 if the session had started before you changed the box. */
    out += '<div class="hint">Fixed for the running session. Stop and start '
        +  'again to change it.</div>';
    keys.forEach(function(k){
      var hz = (live.frame_hz || {})[k];
      out += rowHead(k, hz != null)
          +  '<span class="meta" style="flex:1">'
          +  (hz == null ? 'not being recorded — no viewer was up'
                         : fmt(hz,2) + ' fps · '
                           + ((live.frames||{})[k] || 0) + ' frames saved')
          +  '</span></div>';
    });
    box.innerHTML = out;
    return;
  }
  var up = {};
  (S.record.sources || []).forEach(function(k){ up[k] = true; });
  out += '<div class="hint">Per viewer, and for this session only — it '
      +  'does not change the saved default. The camera stream is 640&times;400 '
      +  'JPEG at quality 60 and the pipeline runs at about 30 fps, so '
      +  '<b>30 saves every frame it sends, at roughly 1 MB/s — about 3.6 '
      +  'GB an hour</b>. The LiDAR view is redrawn from a 10 Hz sensor, so '
      +  'anything above 10 there is copies.</div>';
  keys.forEach(function(k){
    var v = recFps[k];
    out += rowHead(k, up[k])
        +  '<input type="number" data-fps="' + esc(k) + '" value="' + v + '"'
        +  ' min="' + rng[0] + '" max="' + rng[1] + '" step="0.5"'
        +  ' style="width:72px"><span class="meta">fps</span>';
    FPS_PRESETS.forEach(function(p){
      out += '<button data-fps-key="' + esc(k) + '" data-fps-val="' + p + '"'
          +  (Math.abs(v - p) < 1e-6 ? ' class="go"' : '') + '>' + p + '</button>';
    });
    out += '<span class="meta" style="flex:1">'
        +  (up[k] ? '' : 'no viewer running — nothing to pull')
        +  '</span></div>';
  });
  box.innerHTML = out;
  box.querySelectorAll('button[data-fps-key]').forEach(function(b){
    b.onclick = function(){ recSetFps(b.dataset.fpsKey, b.dataset.fpsVal); };
  });
  /* onchange, NOT oninput: clamping on every keystroke turns typing "30" into
     "3" the instant the first digit lands, because 3 is inside the range and
     the box gets rewritten under you. */
  box.querySelectorAll('input[data-fps]').forEach(function(x){
    x.onchange = function(){ recSetFps(x.dataset.fps, x.value); };
  });
}

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
    el('p-rec').innerHTML = '<div id="recstatus"></div><div id="recfps"></div>'
                          + '<div id="rectopics"></div>';
    recLoadTopics();
  }
  /* Repainted on exactly two events: the first snapshot (which carries the
     default this has no other way to learn) and each start/stop transition
     (because a running session shows measured rates instead of boxes). Never
     on the 5 Hz path — see recFps. */
  var live = !!(S.record && S.record.live && S.record.live.recording);
  if(recFps === null || live !== recWasLive){ recWasLive = live; paintRecFps(); }
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
        +  '</b> at ' + esc(recFpsSummary()) + ', telemetry at '
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
function recStart(){
  post('/record/start', {topics: recSelected(), frame_hz: recFps || {}});
}
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

/* ---------------- tab: radio ----------------
   What crossed the RFD900 mesh, as rxl_link_node saw it: what the boat put on
   the air, and what it heard from anyone else.

   THE COUNTS AND THE RATES ARE THE SERVER'S, not this page's. A tab that was
   closed missed nothing, and a page reloaded mid-test does not restart the
   estimate. Incremental like the Logs tab, for the same reason.

   TWO RATES, ON PURPOSE. The aircraft may be sending BUOY_MAP (its own TUNNEL
   format, 1 Hz) or SAFE_PASSAGE (the RXL design, 0.2 Hz). Scoring both is what
   separates "the aircraft is quiet" from "the aircraft is talking in the other
   format", which is not a distinction an operator should have to make by
   reading raw frames. */
function pollRadio(){
  if(tab !== 'radio' || radioBusy) return;
  radioBusy = true;
  fetch('/radio', {method:'POST', headers:{'Content-Type':'application/json'},
                   body: JSON.stringify({since:radioSeq, limit:400})})
    .then(function(r){ return r.json(); })
    .then(function(j){
      radioBusy = false;
      if(!j.rows) return;
      radioSeq = j.newest; radioStats = j;
      radioRows = radioRows.concat(j.rows).slice(-800);
      paintRadio();
    })
    .catch(function(){ radioBusy = false; });
}

/* The system filter is rebuilt only when the set of systems changes: rebuilding
   it every second would reset the operator's selection while they read. */
function syncRadioWho(systems){
  var sel = el('radiowho');
  if(!sel) return;
  var want = systems.map(function(s){ return s.name; });
  if(sel._have && sel._have.join('\x1f') === want.join('\x1f')) return;
  sel._have = want;
  sel.innerHTML = '<option value="">all systems</option>'
    + want.map(function(n){ return '<option>' + esc(n) + '</option>'; }).join('');
  sel.value = radioWho;
}

function paintRadio(){
  var body = el('radiobody');
  if(!body || !radioStats) return;
  var sys = radioStats.systems || [], streams = radioStats.streams || [];
  syncRadioWho(sys);

  function card(k, v, cls, note){
    return '<div class="card"><div class="k">' + k + '</div>'
         + '<div class="v' + (cls ? ' ' + cls : '') + '">' + v + '</div>'
         + (note ? '<div class="meta">' + note + '</div>' : '') + '</div>';
  }
  var out = '<div class="grid">';
  if(!sys.length) out += card('Nothing on the mesh yet', '&mdash;', 'stale',
                              'no frame sent or heard since the link node started');
  sys.forEach(function(v){
    out += card(esc(v.name) + ' · ' + v.sys,
                v.heard_s === null ? 'not heard' : ago(v.heard_s) + ' ago',
                (v.heard_s === null || v.heard_s > 30) ? 'stale' : '',
                v.rx + ' heard · ' + v.tx + ' sent'
                + (v.last ? ' · last ' + esc(v.last) : ''));
  });
  out += '</div>';

  out += '<h3>Aircraft, by format</h3><div class="hint">An <b>estimate</b>. '
       + 'Neither format carries a sequence number, so a lost frame cannot be '
       + 'counted — only a rate below the expected one can be seen.</div>'
       + '<div class="grid">';
  streams.forEach(function(s){
    var v = s.rate_hz === null ? 'never heard'
          : fmt(s.rate_hz, 2) + ' /s of ' + fmt(s.expected_hz, 1);
    var cls = s.pct === null ? 'stale' : (s.pct >= 90 ? '' : 'stale');
    var note = s.pct === null ? esc(s.who)
             : fmt(s.pct, 0) + '% · longest silence '
               + fmt(s.longest_gap_s, 1) + 's · ' + esc(s.who);
    out += card(esc(s.name), v, cls, note);
  });
  out += '</div>';
  el('radiosum').innerHTML = out;

  var atBottom = body.scrollTop + body.clientHeight >= body.scrollHeight - 30;
  var rows = radioRows.filter(function(r){
    if(radioDir && r.dir !== radioDir) return false;
    if(radioWho && r.who !== radioWho) return false;
    if(!radioHb && r.name === 'HEARTBEAT') return false;
    return true;
  });
  body.innerHTML = rows.map(function(r){
    var d = new Date(r.t * 1000).toTimeString().slice(0, 8);
    return '<div><span style="color:var(--muted)">' + d + '</span> '
         + '<span style="color:' + (r.dir === 'TX' ? 'var(--vstrong)' : 'var(--ok)')
         + '">' + (r.dir === 'TX' ? 'SENT →' : 'HEARD ←') + '</span> '
         + '<span style="color:var(--accentline)">' + esc(r.who) + '</span> '
         + '<span style="color:var(--muted)">' + r.src + ':' + r.comp + '→'
         + r.dst + '</span> ' + esc(r.name)
         + (r.summary ? '  ' + esc(r.summary) : '')
         + ' <span style="color:var(--muted)">' + r.bytes + 'B</span></div>';
  }).join('') || '<div style="color:var(--muted)">no frames match</div>';
  if(atBottom) body.scrollTop = body.scrollHeight;
  var d = el('radiodrop');
  if(d) d.textContent = radioStats.dropped
      ? (radioStats.dropped + ' frames dropped (buffer full)') : '';
}

function renderRadio(){
  if(el('radiobody')) return;          /* built once, then painted in place */
  el('p-radio').innerHTML =
      '<div class="hint">Every frame <b>rxl_link_node</b> put on the RFD900 mesh '
    + 'or heard on it — including formats this boat does not speak, which are '
    + 'listed by their TUNNEL type number rather than hidden. The autopilot link '
    + 'is not here: that is telemetry_bridge’s, on a different device.</div>'
    + '<div style="display:flex;gap:6px;align-items:center;margin:8px 0;flex-wrap:wrap">'
    + '<select id="radiodir"><option value="">sent and heard</option>'
    + '<option value="TX">sent</option><option value="RX">heard</option></select>'
    + '<select id="radiowho"><option value="">all systems</option></select>'
    + '<label class="meta"><input type="checkbox" id="radiohb"> heartbeats</label>'
    + '<button id="radiotest">Send test frame</button>'
    + '<button id="radioclear">Clear</button>'
    + '<span id="radiodrop" class="meta stale" style="margin-left:auto"></span></div>'
    + '<div id="radiosum"></div>'
    + '<pre id="radiobody" style="max-height:none;height:calc(100vh - 380px)"></pre>';
  el('radiodir').value = radioDir;
  el('radiohb').checked = radioHb;
  el('radiodir').onchange = function(){ radioDir = this.value; paintRadio(); };
  el('radiowho').onchange = function(){ radioWho = this.value; paintRadio(); };
  el('radiohb').onchange = function(){ radioHb = this.checked; paintRadio(); };
  /* The test frame is a TUNNEL neither vehicle acts on. The link node refuses
     it when nothing has been heard yet, because there is no address to send to
     then, and says so rather than pretending it went out. */
  el('radiotest').onclick = function(){ post('/radio/send_test', {}); };
  el('radioclear').onclick = function(){
    post('/radio/clear', {}); radioRows = []; radioSeq = 0; radioStats = null;
    el('radiobody').innerHTML = ''; };
  paintRadio();
  pollRadio();
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
/* What a widget currently holds, or undefined if it holds nothing usable.
   Separate from the posting so the Camera tab reads its own boxes with the
   same rules — including the empty-box rule, which is the one worth having in
   exactly one place. */
function readControl(p, prefix){
  var box = el(prefix + p.name);
  if(!box) return undefined;
  if(p.type === 'bool') return box.checked;
  if(p.type === 'string') return box.value;
  var v = Number(box.value);
  /* Caught here only to stop an empty box being sent as 0, which the node
     would accept. The RANGE is deliberately not checked here: the node owns
     that rule and refuses in its own words, and a second copy of the bounds
     in this page is a copy that goes stale. */
  if(box.value === '' || !isFinite(v)){
    toast(p.name + ': not a number', true);
    return undefined;
  }
  return v;
}

function tuneApply(name){
  var p = tuneRowOf(name);
  if(!p) return;
  var v = readControl(p, 'tv_');
  if(v !== undefined) tunePost(name, v);
}
function tuneRevert(name){
  var p = tuneRowOf(name);
  if(p) tunePost(name, p['default']);
}
/* THE one /params/set caller. A profile recall is this with several values in
   it rather than one, which is why there is no /camera/profile/load endpoint:
   the same path, the same per-value refusals, the same log lines per knob. */
function paramPost(node, values, after, done){
  fetch('/params/set', {method:'POST', headers:{'Content-Type':'application/json'},
                        body: JSON.stringify({node: node, values: values})})
    .then(function(r){ return r.json(); })
    .then(function(j){
      var msg = j.message || (j.ok ? 'applied' : 'refused');
      /* A planner knob is accepted at once and used at the next START. Saying so
         here, where the operator reads the answer, is what keeps "applied" from
         being heard as "the boat is doing that now". */
      if(j.ok && node === PLANNER_NODE && Object.keys(values).some(isPlannerName))
        msg += ' — planner: applies at the next START';
      toast(msg, !j.ok);
      /* Re-read rather than assume. A node may accept a set and clamp it, and
         the value worth showing is the one it is now running on. */
      if(after) after();
      if(done) done(j);
    })
    .catch(function(err){ toast('request failed: ' + err, true); });
}

function tunePost(name, v){
  var values = {};
  values[name] = v;
  paramPost(tuneNode, values, function(){ tuneLoad(); });
}

/* `prefix` namespaces the input ids. Both the Tuning tab and the Camera tab
   render the same parameters, both panes stay in the DOM once visited, and
   getElementById returns the FIRST match — so one prefix would mean the Camera
   tab quietly reading and writing the Tuning tab's boxes. */
function tuneControl(p, prefix){
  var id = prefix + p.name;
  if(!p.editable)
    return '<span class="meta" style="min-width:120px;text-align:right">'
         + esc(tuneFmt(p.value)) + '</span>';
  var set = ' <button data-apply="' + esc(p.name) + '">Set</button>';
  if(p.type === 'bool')
    return '<input type="checkbox" id="' + esc(id) + '"'
         + (p.value ? ' checked' : '') + ' style="width:15px;height:15px">' + set;
  /* A node that advertises its accepted values gets a dropdown, so a mode
     name is picked rather than spelled. The list rides in the descriptor's
     additional_constraints, which is the field ROS provides for it, so this
     needs no per-node knowledge and a node that grows a mode offers it here
     the moment it restarts. */
  if(p.choices && p.choices.length)
    return '<select id="' + esc(id) + '" style="width:180px">'
         + p.choices.map(function(c){
             return '<option' + (String(p.value) === c ? ' selected' : '')
                  + '>' + esc(c) + '</option>'; }).join('')
         + '</select>' + set;
  if(p.type === 'string')
    return '<input id="' + esc(id) + '" value="' + esc(p.value)
         + '" style="width:180px">' + set;
  return '<input type="number" id="' + esc(id) + '" value="' + esc(p.value)
       + '" step="' + (p.type === 'integer' ? '1' : 'any') + '"'
       + (p.lo === null ? '' : ' min="' + p.lo + '"')
       + (p.hi === null ? '' : ' max="' + p.hi + '"')
       + ' style="width:96px">' + set;
}

function tuneRow(p, prefix){
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
       + mark + tuneControl(p, prefix) + '</div>';
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
  /* bt_runner_node's planner knobs get their own group, ABOVE the rest, because
     they do not behave like the rest: see plannerSection. */
  var planner = tuneNode === PLANNER_NODE ? tuneRows.filter(isPlannerRow) : [];
  var dyn = tuneRows.filter(function(p){
    return p.editable && planner.indexOf(p) < 0; });
  var fixed = tuneRows.filter(function(p){ return !p.editable; });
  if(planner.length && !plProf.loaded) plannerProfilesLoad();

  b.innerHTML = plannerSection(planner) + '<h3>Tunable while running</h3>'
    + '<div class="hint">Applied to the running node immediately and <b>lost on '
    + 'restart</b>. crusader_params.yaml is the source of truth; a row showing '
    + '<span style="color:var(--warn)">yaml &lt;value&gt;</span> is one to write '
    + 'back into it before the next run.</div>'
    + (dyn.map(function(p){ return tuneRow(p, 'tv_'); }).join('')
       || '<div class="hint">none</div>')
    + '<h3>Fixed at startup</h3>'
    + '<div class="hint">Structural: topic names, mounting geometry, freshness '
    + 'budgets. Change them in crusader_params.yaml and restart the node. Shown '
    + 'rather than hidden because a knob you cannot turn and one you cannot find '
    + 'are different problems.</div>'
    + (fixed.map(function(p){ return tuneRow(p, 'tv_'); }).join('')
       || '<div class="hint">none</div>');

  b.querySelectorAll('button[data-apply]').forEach(function(x){
    x.onclick = function(){ tuneApply(x.dataset.apply); }; });
  b.querySelectorAll('button[data-revert]').forEach(function(x){
    x.onclick = function(){ tuneRevert(x.dataset.revert); }; });
  /* Enter applies the row you are in. Reaching for the mouse after every number
     is the difference between sweeping a gate and giving up on it. The profile
     name box has its own Enter (plannerWire), and no tv_ id. */
  b.querySelectorAll('input').forEach(function(x){
    if(x.type === 'checkbox' || x.id.indexOf('tv_') !== 0) return;
    x.onkeydown = function(ev){
      if(ev.key === 'Enter'){ ev.preventDefault(); tuneApply(x.id.slice(3)); } };
  });
  plannerWire(b);
}

/* ---------------- the planner group of the Tuning tab ----------------
   bt_runner_node's nav_* knobs are the one set of parameters that is both
   tuned on the water and NOT applied the moment it is set. The node copies them
   into the planner when a mission goal is ACCEPTED, so a change lands at the
   next START and never mid-mission. Before that was true they were copied once
   at startup, and a set here "worked" -- the number on this page changed -- while
   the planner kept running on the old one. The group says so, in its heading,
   because the difference between "applied" and "applies next START" is the
   difference between a result and a wasted lap.

   THE ONE PLACE THIS PAGE NAMES A NODE. Every other row is built from the
   descriptors and this page is told nothing; the grouping is by node name and
   prefix because a descriptor has no field for "applies at the next START" and
   inventing one in the page would be the second copy of a rule. What the page
   does NOT assume is that the node is new enough: bt_runner_node's descriptors
   carry PLANNER_MARK once it re-reads at goal accept, and a node without it is
   an old build on which a set here still changes nothing -- that is shown, in
   red, rather than left to be found out on the water.

   Profiles are files (planner_profiles.py): ONLY bt_runner_node's nav_* keys are
   applied, through the ordinary /params/set; the rest is listed as skipped. */
var PLANNER_NODE = '/bt_runner_node', PLANNER_PREFIX = 'nav_';
var PLANNER_MARK = 'applies at the next START';
var plProf = {profiles:[], sources:[], err:'', busy:false, loaded:false};
var plSel = '', plName = '', plLast = '';

function isPlannerName(name){ return name.indexOf(PLANNER_PREFIX) === 0; }
function isPlannerRow(p){ return isPlannerName(p.name) && p.editable; }
function plannerStale(rows){
  return rows.length > 0 && !rows.some(function(p){
    return (p.description || '').indexOf(PLANNER_MARK) >= 0; });
}
function plannerId(p){ return p.origin + '/' + p.name; }
function plannerCurrent(){
  var ok = plProf.profiles.filter(function(p){ return !p.error; });
  return ok.filter(function(p){ return plannerId(p) === plSel; })[0] || ok[0];
}

/* One POST of a JSON body, answered with the parsed reply. The planner group makes
   two such calls and the page already had a dozen spelled out in full; a helper
   here keeps the next one from being a fourteenth. */
function postJson(path, body){
  return fetch(path, {method:'POST', headers:{'Content-Type':'application/json'},
                      body: JSON.stringify(body || {})})
    .then(function(r){ return r.json(); });
}
/* A hint line. `color` is a CSS value, or none for the default dim text. */
function hintDiv(html, color){
  return '<div class="hint"' + (color ? ' style="color:' + color + '"' : '') + '>'
       + html + '</div>';
}
function plural(n, word){ return n + ' ' + word + (n === 1 ? '' : 's'); }

function plannerProfilesTake(j){
  plProf.busy = false; plProf.loaded = true;
  plProf.err = j.ok === false ? (j.message || 'no answer') : '';
  if(j.profiles) plProf.profiles = j.profiles;
  if(j.sources) plProf.sources = j.sources;
  paintTune();
}
function plannerProfilesLoad(){
  if(plProf.busy) return;
  plProf.busy = true;
  postJson('/planner/profile/list')
    .then(plannerProfilesTake)
    .catch(function(err){
      plannerProfilesTake({ok:false, message:'request failed: ' + err}); });
}

/* What loading the picked profile would do, shown BEFORE the button is pressed:
   the keys it applies, and every key it leaves alone with the reason. */
function plannerPreview(p){
  if(!p) return '';
  var keys = Object.keys(p.values);
  var out = (p.about ? esc(p.about) + '<br>' : '')
    + 'Applies <b>' + plural(keys.length, 'planner key') + '</b>'
    + (keys.length ? ': ' + keys.map(esc).join(', ') : '') + '.';
  if(p.skipped.length)
    out += '<br><span style="color:var(--warn)">Skips ' + p.skipped.length + ':</span> '
      + p.skipped.map(function(x){
          return esc(x.key) + ' <span style="color:var(--muted)">(' + esc(x.why)
               + ')</span>'; }).join('; ');
  return hintDiv(out);
}

function plannerSection(planner){
  if(!planner.length) return '';
  var out = '<h3>Planner (applies at the next START)</h3>'
    + hintDiv('bt_runner_node reads these when a mission goal is accepted. '
    + 'A change is <b>kept now and used at the next START</b> &mdash; never part-way '
    + 'through a mission &mdash; and is lost when the node restarts. Nav2\u2019s own '
    + 'settings (inflation, planner_server) are not here: they are fixed when the rig '
    + 'launches.');
  if(plannerStale(planner))
    out += hintDiv('<b>This bt_runner_node is an old build.</b> It copies these once at '
      + 'startup, so a set here changes the number below and NOT the planner. Rebuild '
      + 'crusader_bt and restart it before trusting any of them.', 'var(--bad)');

  /* profiles */
  var ps = plProf.profiles, cur = plannerCurrent();
  out += '<div class="row" style="flex-wrap:wrap;gap:6px"><label class="meta">profile</label>';
  if(ps.length)
    out += '<select id="plsel" style="max-width:320px">' + ps.map(function(p){
        return '<option value="' + esc(plannerId(p)) + '"' + (p.error ? ' disabled' : '')
          + ((cur && plannerId(p) === plannerId(cur)) ? ' selected' : '') + '>'
          + esc(p.title) + ' (' + esc(p.origin) + ')'
          + (p.error ? ' \u2014 unreadable' : '') + '</option>'; }).join('') + '</select>'
      + '<button class="go" id="plload">Load profile</button>';
  else
    out += '<span class="meta">' + (plProf.loaded ? 'none found' : 'reading\u2026')
        + '</span>';
  out += '<button id="plreload" title="read the profile directories again">&#8635;</button>'
      + '</div>';
  if(plProf.err) out += hintDiv(esc(plProf.err), 'var(--bad)');
  /* A directory that is missing is the answer to "why is the list empty". */
  plProf.sources.filter(function(x){ return !x.ok; }).forEach(function(x){
    out += hintDiv(esc(x.origin) + ' profiles: ' + esc(x.reason), 'var(--warn)'); });
  ps.filter(function(p){ return p.error; }).forEach(function(p){
    out += hintDiv(esc(plannerId(p)) + ': ' + esc(p.error), 'var(--bad)'); });
  out += plannerPreview(cur);
  if(plLast) out += hintDiv(plLast, 'var(--strong)');

  var saved = plProf.sources.filter(function(x){ return x.origin === 'saved'; })[0];
  var drifted = planner.filter(tuneDrift).length;
  out += '<div class="row" style="flex-wrap:wrap;gap:6px">'
    + '<label class="meta">save</label>'
    + '<input id="plname" maxlength="40" placeholder="name" value="' + esc(plName)
    + '" style="width:150px">'
    + '<button id="plsave">Save as profile</button>'
    + '<span class="meta">the ' + plural(drifted, 'planner key')
    + ' that differ from the YAML'
    + (saved ? ' \u2192 ' + esc(saved.dir) + '/&lt;name&gt;.yaml' : '')
    + ' (<code>LAKE_TUNING=</code> takes that file)</span></div>';
  return out + planner.map(function(p){ return tuneRow(p, 'tv_'); }).join('');
}

function plannerLoadProfile(){
  var p = plannerCurrent();
  if(!p) return;
  var keys = Object.keys(p.values);
  if(!keys.length){ toast('that profile has no planner keys', true); return; }
  paramPost(PLANNER_NODE, p.values, function(){ tuneLoad(); }, function(j){
    var res = j.results || [], bad = res.filter(function(r){ return !r.ok; });
    plLast = 'Loaded <b>' + esc(p.title) + '</b>: ' + (res.length - bad.length) + ' of '
      + keys.length + ' keys applied (they take effect at the next START).'
      + bad.slice(0, 3).map(function(r){
          return ' <span class="stale">' + esc(r.name) + ' refused: ' + esc(r.reason)
               + '</span>'; }).join('')
      + (bad.length > 3 ? ' <span class="stale">(+' + (bad.length - 3) + ' more refused)</span>' : '')
      + (p.skipped.length ? '<br>Skipped ' + p.skipped.length + ' (Nav2 keys need a rig '
        + 'restart): ' + p.skipped.map(function(x){ return esc(x.key); }).join(', ') : '');
    paintTune();
  });
}

function plannerSaveProfile(){
  var name = plName.trim();
  if(!name){ toast('name the profile first', true); return; }
  if(plProf.profiles.some(function(x){ return x.origin === 'saved' && x.name === name; })
     && !confirm('Overwrite the saved profile "' + name + '"?')) return;
  /* Only the NAME goes up: the node reads the live values itself (see
     gcs_node._planner_profile_save). */
  postJson('/planner/profile/save', {name: name})
    .then(function(j){
      toast(j.message || (j.ok ? 'saved' : 'refused'), !j.ok, j.ok ? 9000 : 0);
      if(j.ok){ plName = ''; plSel = 'saved/' + name; plLast = ''; }
      plannerProfilesTake(j);
    })
    .catch(function(err){ toast('request failed: ' + err, true); });
}

function plannerWire(b){
  var sel = b.querySelector('#plsel');
  if(sel) sel.onchange = function(){ plSel = this.value; paintTune(); };
  var load = b.querySelector('#plload');
  if(load) load.onclick = plannerLoadProfile;
  var rl = b.querySelector('#plreload');
  if(rl) rl.onclick = function(){ plProf.loaded = false; plannerProfilesLoad(); };
  var save = b.querySelector('#plsave'), box = b.querySelector('#plname');
  if(save) save.onclick = plannerSaveProfile;
  if(box){
    box.oninput = function(){ plName = this.value; };
    box.onkeydown = function(ev){
      if(ev.key === 'Enter'){ ev.preventDefault(); plannerSaveProfile(); } };
  }
}

/* ---------------- tab: Task 1 (the lake panel, in a frame) ----------------
   The lake panel (crusader_sim task1_panel --lake, :8095 on the Jetson) is where
   a Task 1 run is driven: the operator plays the UAV, builds the field, answers
   the boat's checkpoints. It is its own server and stays one; this tab only
   frames it, so there is one page to open on the day and nothing to keep in step.
   It sends no X-Frame-Options and no CSP (task1_panel.py's _send), so framing
   works, and the URL is a field because the panel may later be served from the
   laptop instead.

   THE FRAME EXISTS ONLY WHILE THIS TAB IS IN FRONT, and that is a safety
   decision before it is a bandwidth one. The panel's DEAD-MAN counts browser
   polls: it resends the field every 5 s only while a browser has polled it in the
   last 3 s, and the boat's own plan-freshness guard aborts the mission about 15 s
   after the last field. A frame that stayed loaded behind another tab would keep
   that heartbeat going while the operator cannot see the panel's banners -- the
   dead-man state, a checkpoint waiting for an answer, the ABORT notice -- and
   whether it kept polling at all would be up to the browser's throttling of
   hidden frames, which is not a rule anybody can state. So leaving the tab
   removes the frame, the heartbeat stops, and the tab says so in advance and
   again on the way out. Deterministic and said out loud beats convenient and
   sometimes true. It also means an operator on another tab adds no load to a
   long WiFi link.

   REACHABLE OR NOT is asked with a HEAD to the panel's root, no-cors: that
   resolves on ANY answer and rejects only when nothing answered, which is the
   one thing a cross-origin page can learn about a refused connection (the frame
   itself fires `load` for the browser's error page too). It is NOT a poll of the
   panel's state route, so it cannot feed the dead-man. */
var T1_KEY = 'crsd-task1-url', T1_PORT = 8095, T1_PROBE_MS = 4000;
var t1Token = 0;
var T1_START = 'LAKE_DATUM=&lt;lat,lon&gt; bash /root/robotx_ws/src/rx26_asv/'
             + 'crusader_sim/scripts/lake_rig_up.sh';

function t1Default(){
  return location.protocol + '//' + location.hostname + ':' + T1_PORT + '/'; }
function t1Saved(){
  try{ return localStorage.getItem(T1_KEY) || ''; }catch(e){ return ''; } }
/* Only a URL that is NOT the default is remembered: the default follows the host
   this page was opened from, and a stored copy of it would go on pointing at the
   old host after the Jetson changes address. */
function t1Store(u){
  try{
    if(u && u !== t1Default()) localStorage.setItem(T1_KEY, u);
    else localStorage.removeItem(T1_KEY);
  }catch(e){}
}
/* http(s) only, normalised; '' for anything else (a javascript: URL typed into a
   box that becomes a frame's src is not a feature). */
function t1Valid(u){
  try{
    var x = new URL(u);
    return (x.protocol === 'http:' || x.protocol === 'https:') ? x.href : '';
  }catch(e){ return ''; }
}

function t1Probe(u){
  var ctl = ('AbortController' in window) ? new AbortController() : null;
  var timer = setTimeout(function(){ if(ctl) ctl.abort(); }, T1_PROBE_MS);
  return fetch(u, {method:'HEAD', mode:'no-cors', cache:'no-store',
                   signal: ctl ? ctl.signal : undefined})
    .then(function(){ clearTimeout(timer); return true; })
    .catch(function(){ clearTimeout(timer); return false; });
}

function t1Build(){
  if(el('t1body')) return;
  el('p-task1').innerHTML =
      '<div id="t1bar"><label class="meta">panel</label>'
    + '<input id="t1url" spellcheck="false" style="flex:1;min-width:220px;max-width:460px">'
    + '<button id="t1go">Load</button>'
    + '<button id="t1reset" title="back to this host, port ' + T1_PORT + '">default</button>'
    + '<a id="t1open" class="meta" target="_blank" rel="noopener">open on its own</a>'
    + '<span class="meta" id="t1st" style="margin-left:auto"></span></div>'
    + '<div id="t1rig"><b>rig</b><span id="t1rst" class="meta">…</span>'
    + '<label class="meta" title="POOL=1: camera only, no LiDAR or Nav2 in the plan, tight-field profile">'
    + '<input type="checkbox" id="t1pool">pool (camera only)</label>'
    + '<label class="meta" title="PUBLISH=1: the tree sends setpoints once the pilot arms, selects GUIDED and START is pressed">'
    + '<input type="checkbox" id="t1pub">setpoints ON</label>'
    + '<label class="meta" title="use the boat\'s position even if a field in progress is within 1 km (a new site)">'
    + '<input type="checkbox" id="t1new">new datum here</label>'
    + '<button class="go" id="t1up">START RIG</button><button id="t1down">STOP RIG</button>'
    + '<span class="meta" id="t1next" style="margin-left:auto"></span></div>'
    + '<details id="t1out"><summary class="meta">rig output (lake_rig_up.sh / lake_rig_down.sh)</summary>'
    + '<pre id="t1log"></pre></details>'
    + '<div id="t1warn"><b>Leaving this tab stops the UAV heartbeat: the boat aborts a '
    + 'running mission about 15 s later.</b> The panel resends the field only while a '
    + 'browser is polling it. Keep this tab open and in front during a run. The RC SB '
    + 'switch is the only e-stop; nothing here is one. STOP UAV asks you to confirm: if '
    + 'it does nothing, your browser is refusing dialogs from a framed page &mdash; use '
    + '<b>open on its own</b>.</div>'
    + '<div id="t1body"></div>';
  el('t1go').onclick = function(){ t1Load(); };
  el('t1reset').onclick = function(){
    el('t1url').value = t1Default(); t1Store(''); t1Load(); };
  el('t1url').onkeydown = function(ev){
    if(ev.key === 'Enter'){ ev.preventDefault(); t1Load(); } };
  el('t1up').onclick = function(){
    var pub = el('t1pub').checked;
    if(pub && !confirm('Start the rig with SETPOINTS ON?\n\nOnce the pilot arms, selects GUIDED '
                       + 'and START is pressed in the panel, the boat drives itself. The RC SB '
                       + 'switch is the only e-stop.')) return;
    if(S && S.lake && S.lake.up
       && !confirm('Restart the rig? The panel below goes away for up to a minute.')) return;
    post('/lake/start', {pool: el('t1pool').checked, publish: pub,
                         new_datum: el('t1new').checked});
  };
  el('t1down').onclick = function(){
    if(!confirm('Stop the lake rig? The Task 1 panel goes away.')) return;
    post('/lake/stop', {});
  };
}

/* ---- the rig strip ----
   START RIG runs crusader_sim's lake_rig_up.sh ON THE BOAT (the ground station is in the
   same container) with the datum taken from the boat: the field in progress when the boat
   is within 1 km of it, so a restart between capturing the field and the runs brings the
   pinned buoys back; otherwise the boat's position now (lake_rig.py's header says why).
   It starts the rig, not a mission: START in the panel below still needs the pilot to arm
   and select GUIDED, and the node refuses START/STOP RIG while armed in GUIDED or AUTO.
   When a start or stop finishes, the tab looks for the panel again, so the frame follows. */
var t1RigWas = null;
function t1LL(lat, lon){ return lat.toFixed(7) + ', ' + lon.toFixed(7); }

function renderT1Rig(){
  var L = S.lake, st = el('t1rst');
  if(!L || !st) return;
  var last = L.last, txt;
  if(L.busy) txt = L.busy + '… (up to a minute)';
  else if(L.up && last && last.op === 'start' && last.ok)
    txt = 'UP — datum ' + t1LL(last.datum[0], last.datum[1])
        + (last.pool ? ' · POOL' : '') + (last.publish ? ' · SETPOINTS ON' : ' · setpoints off');
  else if(L.up) txt = 'UP (started outside this page: its banner is in the rig output if it was started here)';
  else txt = 'down';
  if(!L.busy && last && !last.ok) txt += ' — last ' + last.op + ' FAILED (exit ' + last.rc + '): open the rig output';
  st.textContent = txt;
  st.style.color = L.busy ? 'var(--warn)' : L.up ? 'var(--ok)' : (last && !last.ok) ? 'var(--bad)' : 'var(--dim)';

  var nx = L.next;
  if(el('t1new').checked)
    nx = S.boat.ok ? {lat: S.boat.lat, lon: S.boat.lon, why: "the boat's position (new datum asked for)"} : null;
  el('t1next').textContent = L.busy ? '' : nx ? 'START RIG uses ' + t1LL(nx.lat, nx.lon) + ': ' + nx.why
                                             : 'START RIG needs a fresh boat pose (GPS yaw resolved?)';
  el('t1up').textContent = L.up ? 'RESTART RIG' : 'START RIG';
  el('t1up').disabled = !!L.busy || !nx;
  el('t1down').disabled = !!L.busy || !(L.procs && L.procs.length);
  var log = last ? last.tail.join('\n') : '';
  if(el('t1log').textContent !== log) el('t1log').textContent = log;

  if(t1RigWas && !L.busy && tab === 'task1') t1Load();
  t1RigWas = L.busy;
}

function t1Status(msg){ var e = el('t1st'); if(e) e.textContent = msg; }

function t1Mount(u){
  var b = el('t1body'), f = document.createElement('iframe');
  /* Built through the DOM, not innerHTML: the URL is whatever was typed. */
  f.id = 't1frame';
  f.src = u;
  b.innerHTML = '';
  b.appendChild(f);
  t1Status('framing ' + u);
}

function t1Unreachable(u){
  el('t1body').innerHTML = viewerPanel(
    'The Task 1 panel did not answer',
    'Nothing is listening at <b>' + esc(u) + '</b>. The panel is not part of core: it '
      + 'is the lake rig\'s. Start it with <b>START RIG</b> above: it runs the rig on the '
      + 'boat with the datum taken from the boat (tick <b>pool</b> for the camera-only pool '
      + 'test, <b>setpoints ON</b> for runs). By hand instead: on the Jetson host '
      + 'run <code>docker exec -it asv bash</code>, then in asv:'
      + '<pre style="text-align:left">' + T1_START + '</pre>'
      + 'Use the same <b>LAKE_DATUM</b> every time you restart it. A panel on another '
      + 'machine: type its address above.',
    '<button class="go" id="t1retry">Retry</button> '
      + '<button id="t1force" title="frame it anyway: the probe can be wrong from '
      + 'a laptop-hosted panel">Load anyway</button>');
  el('t1retry').onclick = function(){ t1Load(); };
  el('t1force').onclick = function(){ t1Load(true); };
  t1Status('no answer from ' + u);
}

function t1Load(force){
  var box = el('t1url'), u = t1Valid(box.value.trim());
  if(!u){ toast('not an http(s) address', true); return; }
  box.value = u;
  t1Store(u);
  el('t1open').href = u;
  var mine = ++t1Token;
  if(force){ t1Mount(u); return; }
  t1Status('looking for the panel\u2026');
  t1Probe(u).then(function(ok){
    /* Answered after the operator left, or after a newer Load: a frame mounted
       now would be a heartbeat on a tab nobody is looking at. */
    if(mine !== t1Token || tab !== 'task1') return;
    if(ok) t1Mount(u); else t1Unreachable(u);
  });
}

function task1Enter(){
  t1Build();
  /* Clicking the tab you are already on must not reload a running panel. */
  if(el('t1body').querySelector('iframe')) return;
  if(!el('t1url').value) el('t1url').value = t1Saved() || t1Default();
  t1Load();
}

function task1Leave(){
  t1Token++;                                  /* a probe still in flight must not mount */
  var b = el('t1body');
  if(!b || !b.querySelector('iframe')) return;
  b.innerHTML = '';
  t1Status('');
  toast('Task 1 panel closed: the UAV heartbeat has stopped. A running mission aborts '
        + 'about 15 s after its last field. Open the Task 1 tab to resume it.', true, 12000);
}

/* ---------------- the camera pane ----------------
   Two fetches, both on demand and neither on the 5 Hz path:

     /params/list on oak_detector   the live values, ranges, choices, and the
                                    comparison against crusader_params.yaml
     /camera/profile/list           the saved profiles, and the GROUPING ONLY

   The split is deliberate and the node's docstring says why: ranges, choices
   and editability describe the node that is actually running, so they come
   from the node. Sending them twice is how the page ends up showing a bound
   the node does not enforce.

   Painted only when something changes. The pane holds number boxes an operator
   is part-way through typing into, and a repaint under the cursor loses the
   digits — the same trap the Record tab's topic list carries a comment about. */
/* WHICH node the pane tunes: whichever camera node is running, first match
   wins. dock_view (tools/dock_view.py, the Task 3 dock model) declares the same
   oak_controls parameters as oak_detector for exactly this reason, so a saved
   profile applies to either. Nothing running: oak_detector, as before. */
var CAM_NODES = ['/oak_detector', '/dock_view'];
function camNode(){
  var items = (S && S.nodes && S.nodes.items) || [];
  for(var i = 0; i < CAM_NODES.length; i++){
    var row = items.filter(function(n){ return '/' + n.name === CAM_NODES[i]; })[0];
    if(row && row.running) return CAM_NODES[i];
  }
  return CAM_NODES[0];
}
var camRows = [], camProfiles = [], camStore = null, camGroups = [];
var camErr = '', camProfErr = '', camBusy = false, camBuilt = false;

function camLoadParams(){
  if(camBusy) return;
  camBusy = true;
  fetch('/params/list', {method:'POST',
                         headers:{'Content-Type':'application/json'},
                         body: JSON.stringify({node: camNode()})})
    .then(function(r){ return r.json(); })
    .then(function(j){
      camBusy = false;
      camErr = j.ok ? '' : (j.message || 'no answer');
      camRows = j.ok ? (j.params || []) : [];
      paintCam();
    })
    .catch(function(err){
      camBusy = false; camErr = 'request failed: ' + err;
      camRows = []; paintCam();
    });
}

/* Every profile endpoint answers with the whole listing and the store status,
   so every one of them refreshes the pane the same way and none of them has to
   guess at what its own write did. */
function camTakeProfiles(j){
  if(j.profiles) camProfiles = j.profiles;
  if(j.store) camStore = j.store;
  if(j.controls) camGroups = j.controls;
  camProfErr = j.controls_error || (j.store && j.store.error) || '';
  paintCam();
}

function camLoadProfiles(){
  fetch('/camera/profile/list', {method:'POST',
                                 headers:{'Content-Type':'application/json'},
                                 body:'{}'})
    .then(function(r){ return r.json(); })
    .then(camTakeProfiles)
    .catch(function(err){ camProfErr = 'request failed: ' + err; paintCam(); });
}

function camRowOf(name){
  return camRows.filter(function(p){ return p.name === name; })[0];
}

/* Straight off the node table the page already polls. Unknown until the first
   snapshot lands, and treated as running then, so the pane does not flash
   "not running" at every reload before it knows. */
function camRunning(){
  var items = (S.nodes && S.nodes.items) || [];
  if(!items.length) return true;
  var row = items.filter(function(n){
    return '/' + n.name === camNode(); })[0];
  return row ? !!row.running : false;
}

/* Set, revert and Enter-in-a-box all arrive here. Written once because they
   differ only in where the value came from, and three copies of "wrap it in an
   object and re-read afterwards" is three places to forget the re-read. */
function camSet(name, value){
  var values = {};
  values[name] = value;
  paramPost(camNode(), values, camLoadParams);
}

function camApplyOne(name){
  var p = camRowOf(name);
  if(!p) return;
  var v = readControl(p, 'cv_');
  if(v !== undefined) camSet(name, v);
}

function camApplyProfile(name){
  var row = camProfiles.filter(function(x){ return x.name === name; })[0];
  if(!row) return;
  /* The whole block at once, not knob by knob. A profile is only reproducible
     because it is complete: applied piecemeal, a refusal partway through
     leaves the camera in a state that is neither the old profile nor the new
     one, and nothing on screen would say so. */
  paramPost(camNode(), row.values, camLoadParams);
}

function camSaveProfile(){
  var box = el('camsavename'), name = box ? box.value.trim() : '';
  if(!name){ toast('name the profile first', true); return; }
  if(camProfiles.some(function(x){ return x.name === name; })
     && !confirm('Overwrite the profile "' + name + '"?')) return;
  /* Only the NAME goes up. The node reads the live values itself, because what
     this page is displaying may be a poll old, or another browser's set, or a
     value the node clamped on the way in — and a profile is a claim about what
     the camera was actually doing when the picture looked right. */
  fetch('/camera/profile/save', {method:'POST',
                                 headers:{'Content-Type':'application/json'},
                                 body: JSON.stringify({name: name,
                                                       node: camNode()})})
    .then(function(r){ return r.json(); })
    .then(function(j){
      toast(j.message || (j.ok ? 'saved' : 'refused'), !j.ok);
      if(j.ok && box) box.value = '';
      camTakeProfiles(j);
    })
    .catch(function(err){ toast('request failed: ' + err, true); });
}

function camDeleteProfile(name){
  if(!confirm('Delete the profile "' + name + '"? This cannot be undone.')) return;
  fetch('/camera/profile/delete', {method:'POST',
                                   headers:{'Content-Type':'application/json'},
                                   body: JSON.stringify({name: name})})
    .then(function(r){ return r.json(); })
    .then(function(j){
      toast(j.message || (j.ok ? 'deleted' : 'refused'), !j.ok);
      camTakeProfiles(j);
    })
    .catch(function(err){ toast('request failed: ' + err, true); });
}

function camProfileOrigin(origin){
  /* stock = shipped and in git; saved = yours; override = yours, shadowing a
     shipped name of the same name. Worth distinguishing, because "delete"
     means "revert to the shipped block" for an override and "it is gone" for
     a saved one. */
  if(origin === 'stock')
    return '<span class="meta">shipped</span>';
  if(origin === 'override')
    return '<span class="meta" style="color:var(--warn)">yours, over the '
         + 'shipped one</span>';
  return '<span class="meta" style="color:var(--ok)">yours</span>';
}

function paintCam(){
  var b = el('camtune');
  if(!b) return;
  var out = '<h3>Profiles</h3>';

  if(camProfErr) out += '<div class="hint stale">' + esc(camProfErr) + '</div>';
  if(camStore && !camStore.writable)
    out += '<div class="hint stale">' + esc(camStore.local_path)
        +  ' is not writable \u2014 tuning still works, saving will not.</div>';

  if(!camProfiles.length){
    out += '<div class="hint">no profiles</div>';
  } else {
    camProfiles.forEach(function(row){
      out += '<div class="row">'
          +  '<span class="nm" style="flex:0 0 120px">' + esc(row.name) + '</span>'
          +  '<span style="flex:1;min-width:80px">'
          +  camProfileOrigin(row.origin) + '</span>'
          +  '<button class="go" data-camuse="' + esc(row.name) + '">Apply</button>'
          +  (row.origin === 'stock' ? ''
              : '<button class="danger" data-camdel="' + esc(row.name)
                + '">Delete</button>')
          +  '</div>';
    });
  }
  out += '<div class="row" style="margin-top:6px">'
      +  '<input id="camsavename" placeholder="name this profile" '
      +  'style="flex:1;min-width:120px">'
      +  '<button id="camsave">Save live values</button></div>'
      +  '<div class="hint">Saved from what the node reports, not from these '
      +  'boxes. Applying one is an ordinary parameter set, so a value the node '
      +  'refuses is refused in its own words.</div>';

  out += '<h3>Controls</h3>';
  /* "not in the ROS graph" is the node's honest answer and a useless one to
     act on, and it arrives as an ERROR — so the actionable line was unreachable
     in exactly the case it was written for. Decided from the node table this
     page already polls rather than by matching on the message text, which
     would be a copy of the node's wording kept here to go stale. */
  if(!camRunning()){
    out += '<div class="hint">oak_detector is not running \u2014 start it on '
        +  'the Nodes tab, or with the Start button in the viewer beside this. '
        +  'The profiles above are still readable; applying one needs the node.'
        +  '</div>';
  } else if(camErr){
    out += '<div class="hint stale">' + esc(camErr) + '</div>';
  } else if(!camRows.length){
    out += '<div class="hint">'
        +  (camBusy ? 'reading the camera\u2026'
                    : 'oak_detector declares no parameters') + '</div>';
  } else {
    /* Grouped and ordered by the node's own table (oak_controls), never by a
       list kept here: add a control there and it appears, in its group, with
       no change to this page. Anything the table does not mention still gets
       shown, under "other", so a knob can never go missing from this pane
       just because the grouping did not know about it. */
    var seen = {};
    camGroups.forEach(function(g){
      var rows = g.controls.map(camRowOf).filter(Boolean);
      rows.forEach(function(p){ seen[p.name] = 1; });
      if(!rows.length) return;
      out += '<div class="hint" style="margin-top:8px;color:var(--strong)">'
          +  esc(g.group) + '</div>'
          +  rows.map(function(p){ return tuneRow(p, 'cv_'); }).join('');
    });
    /* The safety net is for a CAMERA control that outran the grouping, not for
       ROS's own parameters: use_sim_time is declared by every node in the
       graph, has nothing to do with the sensor, and sitting next to ISO in a
       tuning column it reads as something worth trying. */
    var rest = camRows.filter(function(p){
      return p.editable && !seen[p.name] && p.name !== 'use_sim_time'; });
    if(rest.length)
      out += '<div class="hint" style="margin-top:8px;color:var(--strong)">'
          +  'other</div>'
          +  rest.map(function(p){ return tuneRow(p, 'cv_'); }).join('');
  }

  b.innerHTML = out;
  b.querySelectorAll('button[data-camuse]').forEach(function(x){
    x.onclick = function(){ camApplyProfile(x.dataset.camuse); }; });
  b.querySelectorAll('button[data-camdel]').forEach(function(x){
    x.onclick = function(){ camDeleteProfile(x.dataset.camdel); }; });
  if(el('camsave')) el('camsave').onclick = camSaveProfile;
  b.querySelectorAll('button[data-apply]').forEach(function(x){
    x.onclick = function(){ camApplyOne(x.dataset.apply); }; });
  b.querySelectorAll('button[data-revert]').forEach(function(x){
    x.onclick = function(){
      var p = camRowOf(x.dataset.revert);
      if(p) camSet(p.name, p['default']);
    }; });
  /* Enter applies the row you are in \u2014 reaching for the mouse after every
     number is the difference between sweeping a setting and giving up on it. */
  b.querySelectorAll('input').forEach(function(x){
    if(x.type === 'checkbox' || x.id === 'camsavename') return;
    x.onkeydown = function(ev){
      if(ev.key === 'Enter'){ ev.preventDefault(); camApplyOne(x.id.slice(3)); } };
  });
  if(el('camsavename'))
    el('camsavename').onkeydown = function(ev){
      if(ev.key === 'Enter'){ ev.preventDefault(); camSaveProfile(); } };
}

var camWasRunning = null, camNextTry = 0, camWasNode = null;
var CAM_RETRY_MS = 2000;

function renderCam(){
  /* The pane is painted on CHANGE, not on the 5 Hz poll — so a node that
     starts or stops has to nudge it, or you press Start in the viewer and the
     column beside it goes on saying the camera is not running. Only the
     transition repaints, so a half-typed number still survives the tick. */
  var running = camRunning();
  /* The camera node changed (oak_detector stopped, dock_view started): the
     rows on screen belong to the other one, so drop them and ask again. */
  var node = camNode();
  if(camBuilt && node !== camWasNode){
    camWasNode = node; camRows = []; camErr = '';
    if(running) camLoadParams(); else paintCam();
  }
  if(camBuilt && running !== camWasRunning){
    camWasRunning = running;
    if(running) camLoadParams(); else paintCam();
  }
  /* AND KEEP TRYING while it is up but has told us nothing. The two facts come
     from different places and they do not arrive together: "running" is the
     PROCESS TABLE (proc_scan), while /params/list needs the node to be in the
     ROS GRAPH, which the node station rescans on its own timer. Start the
     camera and for a few seconds it is a live process that the parameter
     bridge has never heard of — so a single load on the transition lands on
     "not in the ROS graph" and, because this pane only repaints on change,
     that stale refusal would sit there until you changed tabs.

     Only while the list is EMPTY, and no faster than CAM_RETRY_MS: once there
     are rows this stops, so nothing is repainting under a value you are
     typing. */
  if(camBuilt && running && !camRows.length && !camBusy
     && Date.now() >= camNextTry){
    camNextTry = Date.now() + CAM_RETRY_MS;
    camLoadParams();
  }
  if(!camBuilt){
    camWasRunning = running;
    el('p-cam').innerHTML = '<div id="camview"></div><aside id="camtune"></aside>';
    camBuilt = true;
    paintCam();
    camLoadProfiles();
    camLoadParams();
  }
  renderViewer('camview', S.tabs.camera, 'cam');
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

/* The planned leg from bt_runner (S.nav, built from /crsd/nav/leg_status). The
   server already converted it into the trail's own frame, and blanks it when the
   status is stale, so ok:false means draw nothing, not the last path. Colour is
   the leg's state: FOLLOWING green, BLOCKED red, PLANNING/DEGRADED yellow,
   STRAIGHT (a leg that never touches the costmap) muted; FAILED is red too. */
function navColour(state){
  if(state === 'FOLLOWING') return PAL.green;
  if(state === 'BLOCKED' || state === 'FAILED') return PAL.red;
  if(state === 'PLANNING' || state === 'DEGRADED') return PAL.yellow;
  return PAL.muted;
}

/* Start a new canvas path through pts, [[x, y], ...] in world metres; the caller
   strokes it. The trail and the planned path are both drawn this way. */
function tracePath(pts){
  cx.beginPath();
  pts.forEach(function(q, i){
    var p = px(q[0], q[1]);
    if(i === 0) cx.moveTo(p[0], p[1]); else cx.lineTo(p[0], p[1]);
  });
}

function drawNav(){
  var n = S.nav, badge = el('navbadge');
  if(!n || !n.ok){ if(badge) badge.textContent = ''; return; }
  var col = navColour(n.state), path = n.path || [];
  cx.strokeStyle = col; cx.lineWidth = 2;
  if(path.length > 1){
    cx.setLineDash([6,4]); tracePath(path); cx.stroke(); cx.setLineDash([]);
  }
  if(n.target){                                   /* the carrot the boat is steering at */
    var t = px(n.target[0], n.target[1]);
    cx.beginPath(); cx.arc(t[0], t[1], 5, 0, 6.2832); cx.stroke();
  }
  if(n.goal){                                     /* where the leg really ends */
    var g = px(n.goal[0], n.goal[1]);
    cx.beginPath();
    cx.moveTo(g[0]-5, g[1]-5); cx.lineTo(g[0]+5, g[1]+5);
    cx.moveTo(g[0]-5, g[1]+5); cx.lineTo(g[0]+5, g[1]-5);
    cx.stroke();
  }
  if(badge){
    badge.style.color = col;
    badge.textContent = 'NAV ' + (n.state || '?')
      + (n.blocked_s > 0 ? ' ' + n.blocked_s.toFixed(1) + 's' : '')
      + (n.why ? ' ' + n.why : '');
  }
}

function paintLegend(nClusters, nSectors){
  var g = el('maplegendtxt');
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
    cx.strokeStyle = PAL.trail; cx.lineWidth = 2;
    tracePath(tr); cx.stroke();
  }
  drawNav();      /* over the trail, under the clusters and targets */
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
  /* Only the VIEWER half is torn down when you leave: renderViewer clears the
     element it is given, and handing it the whole pane would take the controls
     and their half-typed values with it. */
  if(tab === 'cam') renderCam();
  else if(camBuilt) renderViewer('camview', S.tabs.camera, 'cam');
  renderViewer('p-lidar', S.tabs.lidar, 'lidar');
  if(tab==='tune') renderTune();
  if(tab==='rec') renderRec();
  if(tab==='logs') renderLogs();
  if(tab==='radio') renderRadio();
  if(tab==='sys' && !document.activeElement.matches('#confirm')) renderSys();
  if(tab==='map') draw();
  if(tab==='task1') renderT1Rig();

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
setInterval(pollRadio, 1000);
</script></body></html>"""


def render(poll_ms: int) -> bytes:
    """The page, with the poll period baked in. UTF-8, and the server must say
    so in the Content-Type — degree signs and em-dashes render as mojibake
    under a browser's latin-1 fallback, and a readout that looks broken is a
    readout nobody trusts."""
    return PAGE.replace("__POLL_MS__", str(int(poll_ms))).encode("utf-8")
