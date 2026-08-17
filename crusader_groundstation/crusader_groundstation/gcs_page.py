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
 :root{--bg:#111;--panel:#1b1b1b;--card:#202020;--line:#333;--dim:#777;
   --fg:#ccc;--ok:#5fbf6a;--warn:#d8a13a;--bad:#e05252;--accent:#2d5a7a}
 html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);
   font:13px/1.5 ui-monospace,Menlo,Consolas,monospace;overflow:hidden}
 #app{display:flex;flex-direction:column;height:100%}
 #bar{display:flex;align-items:center;gap:4px;padding:6px 10px;
   background:var(--panel);border-bottom:1px solid var(--line);flex-wrap:wrap}
 #bar button.tab{background:transparent;color:var(--dim);border:1px solid transparent;
   border-radius:4px;padding:5px 12px;font:inherit;cursor:pointer}
 #bar button.tab:hover{background:#262626;color:var(--fg)}
 #bar button.tab.on{background:var(--accent);color:#fff;border-color:#3d7aa5}
 #link{margin-left:auto;display:flex;align-items:center;gap:8px;font-size:12px}
 .pill{padding:2px 8px;border-radius:10px;background:#262626}
 main{flex:1;overflow:auto;position:relative}
 .pane{display:none;padding:12px 14px}
 .pane.on{display:block}
 #p-map.on,#p-cam.on,#p-lidar.on{padding:0;height:100%;display:flex}
 button{background:#262626;color:var(--fg);border:1px solid #3a3a3a;
   border-radius:4px;padding:4px 12px;font:inherit;cursor:pointer}
 button:hover:not(:disabled){background:#303030}
 button:disabled{opacity:.4;cursor:not-allowed}
 button.danger{border-color:#7a3030;color:#e79a9a}
 button.go{border-color:#3d7aa5;color:#cfe6f5}
 h3{font-size:12px;text-transform:uppercase;letter-spacing:.08em;color:var(--dim);
   margin:16px 0 6px;font-weight:600}
 h3:first-child{margin-top:0}
 .hint{color:var(--dim);font-size:12px;margin:4px 0 0}
 .row{display:flex;align-items:center;gap:10px;padding:6px 0;
   border-bottom:1px solid #262626}
 .dot{width:9px;height:9px;border-radius:50%;flex:none;background:#555}
 .dot.up{background:var(--ok)}.dot.dead{background:var(--bad)}
 .nm{flex:1;color:#eee}
 .meta{color:var(--dim);font-size:11px}
 .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
 .card{background:var(--card);border-radius:6px;padding:9px 12px}
 .card .k{font-size:11px;color:var(--dim)}
 .card .v{font-size:19px;color:#fff;margin-top:2px}
 .stale{color:var(--bad)!important}
 #banner{position:absolute;top:8px;left:50%;transform:translateX(-50%);
   background:var(--bad);color:#fff;padding:5px 14px;border-radius:4px;
   font-weight:600;display:none;z-index:5}
 canvas{display:block;width:100%;height:100%}
 #mapwrap{flex:1;position:relative}
 #mapctl{position:absolute;top:8px;right:8px;display:flex;gap:4px;z-index:5}
 .viewer{flex:1;display:flex;align-items:center;justify-content:center;
   flex-direction:column;gap:10px;padding:20px}
 .viewer img{max-width:100%;max-height:100%;object-fit:contain}
 .empty{border:1px dashed #3a3a3a;border-radius:6px;padding:34px 24px;
   text-align:center;color:var(--dim);max-width:520px}
 pre{background:#0c0c0c;border:1px solid var(--line);border-radius:4px;
   padding:8px;margin:6px 0 0;font-size:11px;max-height:150px;overflow:auto;
   white-space:pre-wrap;color:#999}
 input{background:#0c0c0c;border:1px solid #3a3a3a;color:var(--fg);
   border-radius:4px;padding:5px 8px;font:inherit}
 .lock{background:#2a1a1a;border:1px solid #7a3030;border-radius:6px;padding:12px}
 #toast{position:absolute;bottom:12px;left:50%;transform:translateX(-50%);
   background:#262626;border:1px solid var(--line);border-radius:4px;
   padding:7px 14px;display:none;z-index:6;max-width:80%}
</style></head><body>
<div id="app">
 <div id="bar">
  <button class="tab" data-t="nodes">Nodes</button>
  <button class="tab" data-t="tel">Telemetry</button>
  <button class="tab" data-t="cam">Camera</button>
  <button class="tab" data-t="lidar">LiDAR</button>
  <button class="tab" data-t="map">Map</button>
  <button class="tab" data-t="rec">Record</button>
  <button class="tab" data-t="logs">Logs</button>
  <button class="tab" data-t="sys">System</button>
  <span id="link"><span class="pill" id="rtt">— ms</span>
   <span class="pill" id="armed">—</span></span>
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
     <button onclick="post('/trail/clear',{})">clear trail</button>
    </div></div>
  </div>
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
  t.style.borderColor = bad ? '#7a3030' : '#333';
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

function show(t){
  tab = t;
  ['nodes','tel','cam','lidar','map','rec','logs','sys'].forEach(function(p){
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

/* ---------------- tabs 3 and 4: the viewers ---------------- */
function renderViewer(pane, cfg, tabName){
  var e = el(pane);
  if(tab !== tabName){ e.innerHTML = ''; return; }   /* drop the connection */
  if(cfg && cfg.source){
    var url = location.protocol+'//'+location.hostname+':'+cfg.port+'/';
    if(e.dataset.src === url) return;                /* don't restart a stream */
    e.dataset.src = url;
    e.innerHTML = '<div class="viewer"><iframe src="'+url+'" style="width:100%;'
      + 'height:100%;border:0;background:#111"></iframe></div>';
  } else {
    e.dataset.src = '';
    var btns = (cfg && cfg.candidates || []).map(function(c){
      return '<button class="go" onclick="startNode(\''+c.name+'\')">Start '
           + esc(c.label)+'</button>'; }).join(' ');
    e.innerHTML = '<div class="viewer"><div class="empty">'
      + '<div style="font-size:15px;color:#bbb;margin-bottom:6px">'
      + esc(cfg ? cfg.title : 'Viewer not running')+'</div>'
      + '<div style="margin-bottom:14px">'+esc(cfg?cfg.hint:'')+'</div>'
      + btns + '</div></div>';
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
    out += '<div class="lock"><div style="color:#e79a9a;margin-bottom:6px">'
        +  'Power helper unreachable</div><div class="hint">'+esc(p.reason)
        +  '</div></div>';
  } else if(p.locked){
    out += '<div class="lock"><div style="color:#e79a9a;margin-bottom:8px">'
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

/* ---------------- tab 6: record ---------------- */
function renderRec(){
  var r = S.record, live = r.live, out = '';
  out += '<h3>Session</h3>';
  if(live && live.recording){
    out += '<div class="card" style="border:1px solid #7a5a30">'
        +  '<div class="k">recording ' + esc(live.name) + '</div><div class="v">'
        +  ago(live.elapsed_s) + ' &middot; ' + live.samples + ' samples &middot; '
        +  fmt(live.size_mb,1) + ' MB</div>'
        +  '<div class="hint">frames: ' + esc(JSON.stringify(live.frames))
        +  ' &middot; ' + fmt(live.free_gb,1) + ' GB free</div>'
        +  (live.error ? '<div class="hint stale">' + esc(live.error) + '</div>' : '')
        +  '</div><div style="margin-top:8px">'
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
        +  fmt(r.telemetry_hz,1) + ' Hz. Stops itself below '
        +  fmt(r.min_free_gb,1) + ' GB free.</div>'
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
  el('p-rec').innerHTML = out;
  document.querySelectorAll('#p-rec button[data-del]').forEach(function(b){
    b.onclick = function(){ delSession(b.dataset.del); };
  });
}
function recStart(){ post('/record/start', {}); }
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
  var colors = {DEBUG:'#666', INFO:'#bbb', WARN:'var(--warn)',
                ERROR:'var(--bad)', FATAL:'var(--bad)'};
  body.innerHTML = logRows.map(function(r){
    var d = new Date(r.t*1000).toTimeString().slice(0,8);
    return '<div><span style="color:#555">' + d + '</span> '
         + '<span style="color:' + (colors[r.level_name]||'#bbb') + '">'
         + r.level_name + '</span> '
         + '<span style="color:#7fa8c9">' + esc(r.node) + '</span> '
         + esc(r.msg) + '</div>';
  }).join('') || '<div style="color:#666">nothing on /rosout yet</div>';
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
  var sel = 'background:#0c0c0c;color:var(--fg);border:1px solid #3a3a3a;'
          + 'border-radius:4px;padding:4px';
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

/* ---------------- tab 5: the map ---------------- */
var cv = el('c'), cx = cv.getContext('2d');
var scale = 6, follow = true, view = {x:0,y:0}, drag = null, W = 0, H = 0;

function colorOf(label){
  var L = (label||'').toLowerCase();
  if(L.indexOf('red')===0) return '#e05252';
  if(L.indexOf('green')===0) return '#5fbf6a';
  if(L.indexOf('yellow')===0) return '#d8c33a';
  if(L.indexOf('blue')===0) return '#4c8fd8';
  if(L.indexOf('black')===0) return '#8a8a8a';
  return '#9a9a9a';
}
function sx(x){ return W/2 + (x - view.x)*scale; }
function sy(y){ return H/2 - (y - view.y)*scale; }
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
  view.x -= (e.clientX-drag.x)/scale; view.y += (e.clientY-drag.y)/scale;
  drag = {x:e.clientX,y:e.clientY}; draw();
});
cv.addEventListener('wheel', function(e){ e.preventDefault();
  zoom(e.deltaY<0?1.1:1/1.1); }, {passive:false});

function ringStep(){
  var t = 90/scale, p = Math.pow(10, Math.floor(Math.log(t)/Math.LN10)), m = t/p;
  return (m<2?1:m<5?2:5)*p;
}
function draw(){
  if(!W || !S) return;
  cx.fillStyle = '#111'; cx.fillRect(0,0,W,H);
  var b = S.boat;
  if(follow && b.ok){ view.x = b.x; view.y = b.y; }
  if(b.ok){
    var step = ringStep(), maxr = Math.hypot(W,H)/scale;
    cx.strokeStyle = '#232323'; cx.fillStyle = '#4a4a4a'; cx.lineWidth = 1;
    cx.font = '11px ui-monospace,monospace';
    for(var r=step; r<=maxr; r+=step){
      cx.beginPath(); cx.arc(sx(b.x), sy(b.y), r*scale, 0, 6.2832); cx.stroke();
      cx.fillText(r+' m', sx(b.x)+4, sy(b.y)-r*scale-3);
    }
  }
  var tr = S.trail;
  if(tr && tr.length>1){
    cx.strokeStyle = '#3a5f7a'; cx.lineWidth = 2; cx.beginPath();
    cx.moveTo(sx(tr[0][0]), sy(tr[0][1]));
    for(var i=1;i<tr.length;i++) cx.lineTo(sx(tr[i][0]), sy(tr[i][1]));
    cx.stroke();
  }
  (S.targets.items||[]).forEach(function(t){
    var X = sx(t.x), Y = sy(t.y);
    var remembered = t.unseen > 2.0;
    var fade = Math.max(0.55, 1 - t.unseen/60);
    cx.globalAlpha = fade;
    cx.fillStyle = colorOf(t.label);
    cx.strokeStyle = t.confirmed ? '#fff' : '#666';
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
    cx.fillStyle = '#ddd'; cx.font = '11px ui-monospace,monospace';
    cx.fillText('#'+t.id+' '+(t.label||'unknown'), X+14, Y-4);
    if(b.ok) cx.fillText(fmt(t.range,1)+' m', X+14, Y+8);
    if(remembered){ cx.fillStyle = '#8a8a8a';
      cx.fillText('seen '+ago(t.unseen)+' ago', X+14, Y+20); }
    cx.globalAlpha = 1;
  });
  if(b.ok){
    cx.save(); cx.translate(sx(b.x), sy(b.y)); cx.rotate(b.heading*Math.PI/180);
    cx.fillStyle = b.ok ? '#f0f0f0' : '#666';
    cx.beginPath(); cx.moveTo(0,-12); cx.lineTo(7,9); cx.lineTo(0,4);
    cx.lineTo(-7,9); cx.closePath(); cx.fill(); cx.restore();
  }
  var step2 = ringStep(), px = step2*scale, x0 = 14, y0 = H-20;
  cx.strokeStyle = '#888'; cx.lineWidth = 2; cx.beginPath();
  cx.moveTo(x0,y0); cx.lineTo(x0+px,y0); cx.moveTo(x0,y0-4); cx.lineTo(x0,y0+4);
  cx.moveTo(x0+px,y0-4); cx.lineTo(x0+px,y0+4); cx.stroke();
  cx.fillStyle = '#aaa'; cx.font = '11px ui-monospace,monospace';
  cx.fillText(step2+' m', x0+px+8, y0+4);
  var nx = W-28, ny = 30;
  cx.strokeStyle = '#888'; cx.fillStyle = '#aaa'; cx.lineWidth = 2;
  cx.beginPath(); cx.moveTo(nx,ny+12); cx.lineTo(nx,ny-12); cx.stroke();
  cx.beginPath(); cx.moveTo(nx,ny-16); cx.lineTo(nx-4,ny-8); cx.lineTo(nx+4,ny-8);
  cx.closePath(); cx.fill();
  cx.fillText('N', nx-3, ny+24);
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
  fetch('/state').then(function(r){ return r.json(); }).then(function(j){
    inflight = false; rtt = performance.now() - t0; S = j; render();
  }).catch(function(){
    inflight = false;
    var bn = el('banner');
    bn.textContent = 'NO CONNECTION TO THE BOAT'; bn.style.display = 'block';
  });
}
document.querySelectorAll('#bar button.tab').forEach(function(b){
  b.onclick = function(){ show(b.dataset.t); }; });
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
