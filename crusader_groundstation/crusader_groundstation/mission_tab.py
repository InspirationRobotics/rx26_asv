"""mission_tab — markup, style and script of the Mission planning tab.

Kept out of gcs_page.py only because the tab is a block editor of about a thousand
lines and the page is already 2500. gcs_page.render() splices the three strings in at
the __MISSION_*__ tokens, so what the browser gets is still ONE self-contained page:
no CDN, no second file to install, nothing to fetch on a competition field with no
internet.

THE FILE FORMAT IS THE EDITOR'S MODEL. A mission is BT.CPP v4 XML; the tab parses it
with DOMParser into a plain node tree (tag, attributes, children, the comments around
it, any text) and serialises it back itself, so a node the catalogue has never heard of
survives a load and a save unchanged. The catalogue (/mission/catalogue) only decorates
that tree: colours, labels, ports, which blocks may hold children. It never decides what
is in the file.

THE PAGE DOES NOT DECIDE WHAT IS VALID. Validation is /mission/validate, and /mission/start
is validated again by the node that runs it.
"""

CSS = r"""
 :root{--onblk:#fff;--onblk-dk:#111;--blkshadow:rgba(0,0,0,.35)}
 html[data-theme="day"]{--blkshadow:rgba(0,0,0,.45)}
 #p-mission.on{padding:0;height:100%;display:flex;flex-direction:column;overflow:hidden}
 #p-mission.on ~ #banner{top:auto;bottom:56px}
 #mtb{display:flex;gap:6px;align-items:center;flex-wrap:wrap;padding:6px 10px;
   background:var(--panel);border-bottom:1px solid var(--line)}
 #mtb select{max-width:230px}
 #mtb .sep{width:1px;align-self:stretch;background:var(--line);margin:0 2px}
 #mname{font-weight:600;color:var(--strong);margin-left:4px}
 #mdesc{flex:1;min-width:160px;max-width:380px}
 #mstat{margin-left:auto;font-size:12px;color:var(--dim)}
 #mstat.live{color:var(--ok);font-weight:600}
 #missues{padding:4px 10px;border-bottom:1px solid var(--line);font-size:12px;
   max-height:84px;overflow:auto;display:none}
 #missues .e{color:var(--bad)}#missues .w{color:var(--warn)}#missues .ok{color:var(--ok)}
 #missues .e,#missues .w{cursor:pointer}
 #mbody{flex:1;min-height:0;display:flex}
 #mpal{flex:0 0 236px;width:236px;overflow:auto;border-right:1px solid var(--line);
   background:var(--panel);padding:8px}
 #mpalctl{display:flex;gap:6px;align-items:center;margin-bottom:6px;flex-wrap:wrap}
 #mpalctl input[type=text]{flex:1;min-width:90px}
 .mcat{margin-bottom:4px}
 .mcat>.ch{display:flex;align-items:center;gap:6px;font-weight:600;font-size:12px;
   cursor:pointer;padding:3px 0;color:var(--strong)}
 .mcat>.ch .sw{width:10px;height:10px;border-radius:3px;background:var(--bc);flex:none}
 .mpi{margin:3px 0;padding:4px 8px;border-radius:7px;background:var(--bc);color:var(--bt);
   cursor:grab;user-select:none;touch-action:none;box-shadow:0 1px 0 var(--blkshadow)}
 .mpi .l{font-weight:600}
 .mpi .d{font-size:10.5px;line-height:1.3;opacity:.92}
 #mws{flex:1;min-width:0;display:flex;flex-direction:column;position:relative}
 #mtabs{display:flex;gap:4px;padding:6px 8px 0;border-bottom:1px solid var(--line);
   flex-wrap:wrap;align-items:flex-end}
 .mtab{padding:4px 10px;border:1px solid var(--btnline);border-bottom:0;
   border-radius:6px 6px 0 0;background:var(--btn);cursor:pointer;display:flex;gap:6px;
   align-items:center;color:var(--fg)}
 .mtab.on{background:var(--accent);color:var(--onaccent);border-color:var(--accentline)}
 .mtab .x{opacity:.7;cursor:pointer;padding:0 2px}
 .mtab .x:hover{opacity:1}
 #mbin{margin-left:auto;margin-bottom:3px;padding:3px 10px;border:1px dashed var(--btnline);
   border-radius:6px;color:var(--dim);font-size:12px}
 #mbin.hot{border-color:var(--bad);color:var(--bad);background:var(--lockbg)}
 #mwork{flex:1;overflow:auto;padding:12px 14px;position:relative;background:var(--bg)}
 .mhat{display:inline-block;padding:3px 12px;border-radius:12px 12px 0 0;background:var(--accent);
   color:var(--onaccent);font-weight:600;font-size:12px}
 .mroot{border:1px solid var(--line);border-radius:0 6px 6px 6px;padding:6px;
   background:var(--card);min-height:52px}
 .slot>.kids{min-height:0}
 .mblk{margin:3px 0;border-radius:8px;background:var(--bc);color:var(--bt);
   position:relative;box-shadow:0 1px 0 var(--blkshadow)}
 .mblk.sel{box-shadow:0 0 0 2px var(--vstrong)}
 .mblk.flag{box-shadow:0 0 0 2px var(--bad)}
 .mblk.ghosted{opacity:.35}
 .mblk.wrapt{outline:3px dashed var(--accentline);outline-offset:2px}
 .mblk>.hd{padding:5px 10px;display:flex;gap:8px;align-items:baseline;flex-wrap:wrap;
   cursor:grab;user-select:none;touch-action:none}
 .mblk .lb{font-weight:700}
 .mblk .nm{font-style:italic;opacity:.95;flex:none;color:inherit}
 .mblk .sm{font-size:12px;opacity:.95;word-break:break-all}
 .mblk .tag{font-size:10px;opacity:.75;margin-left:auto}
 .mblk.c>.slot{margin:0 6px 0 14px;padding:3px 4px;background:var(--card);
   border-radius:4px;min-height:28px}
 .mblk.c>.ft{height:12px}
 .hole{font-size:11px;color:var(--dim);border:1px dashed var(--btnline);border-radius:5px;
   padding:5px 8px;text-align:center}
 #mline{position:fixed;height:4px;background:var(--accentline);border-radius:2px;
   pointer-events:none;display:none;z-index:70}
 #mghost{position:fixed;pointer-events:none;opacity:.88;z-index:90;display:none;
   padding:4px 10px;border-radius:7px;background:var(--bc);color:var(--bt);font-weight:600;
   box-shadow:0 3px 8px var(--blkshadow)}
 #mright{flex:0 0 320px;width:320px;display:flex;flex-direction:column;
   border-left:1px solid var(--line);min-width:0}
 #minsp{flex:0 1 auto;max-height:56%;overflow:auto;padding:8px 10px;background:var(--panel);
   border-bottom:1px solid var(--line)}
 #minsp .ttl{font-weight:700;color:var(--strong)}
 #minsp .pr{margin:6px 0 2px}
 #minsp .pr label{display:flex;gap:6px;align-items:baseline;font-size:12px;color:var(--strong)}
 #minsp .pr label .ty{color:var(--dim);font-size:10.5px}
 #minsp .pr input,#minsp .pr select{width:100%;box-sizing:border-box}
 #minsp .pr .ds{font-size:10.5px;color:var(--dim);line-height:1.3}
 #minsp .pr input.bad{border-color:var(--bad)}
 #minsp .acts{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}
 #minsp .raw{display:flex;gap:4px;margin:3px 0}
 #minsp .raw input{min-width:0}
 #mmapwrap{flex:1;min-height:210px;position:relative;background:var(--bg)}
 #mmapwrap canvas{cursor:crosshair}
 #mmapctl{position:absolute;top:6px;right:6px;display:flex;gap:4px;z-index:2;flex-wrap:wrap;
   justify-content:flex-end;max-width:92%}
 #mmapctl button{padding:2px 8px;font-size:12px}
 #mmapnote{position:absolute;left:8px;top:8px;font-size:11px;color:var(--dim);
   pointer-events:none;z-index:2;max-width:55%}
 #mdlg{position:fixed;left:0;right:0;top:0;bottom:0;background:var(--blkshadow);display:none;
   z-index:95;align-items:center;justify-content:center}
 #mdlg .box{background:var(--panel);border:1px solid var(--line);border-radius:8px;
   padding:14px 16px;max-width:min(760px,94vw);max-height:92vh;overflow:auto;color:var(--fg)}
 #mdlg h3{margin:0 0 8px}
 #mdlg .mbt{display:flex;gap:8px;justify-content:flex-end;margin-top:12px}
 #mdlg .err{color:var(--bad);font-size:12px;margin-top:6px;white-space:pre-wrap}
 #mdlg textarea{width:min(720px,86vw);height:min(460px,56vh);box-sizing:border-box;
   background:var(--field);color:var(--fieldfg);border:1px solid var(--btnline);
   border-radius:4px;font:12px/1.4 ui-monospace,Menlo,Consolas,monospace}
 #mdlg .frm{display:grid;grid-template-columns:auto 1fr;gap:6px 10px;align-items:center}
 @media (max-width:1060px){
   #mpal{flex-basis:200px;width:200px}
   #mright{flex-basis:270px;width:270px}
 }
"""

PANE = r"""
 <div id="mtb">
  <select id="mpick" title="pre-built missions ship with the code and are read-only; saved ones are yours"></select>
  <button id="mload">Load</button>
  <button id="mnew">New</button>
  <span class="sep"></span>
  <button id="msave">Save</button>
  <button id="msaveas">Save as</button>
  <button id="mval">Validate</button>
  <button id="mxml">XML</button>
  <button id="mdown">Download</button>
  <button id="mup">Upload</button>
  <input type="file" id="mfile" accept=".xml,text/xml,application/xml" style="display:none">
  <span class="sep"></span>
  <button class="go" id="mstart">Start</button>
  <button class="danger" id="mstop">Stop</button>
  <span id="mname"></span>
  <input type="text" id="mdesc" placeholder="mission description (saved as the MISSION: comment)" spellcheck="false">
  <span id="mstat">&mdash;</span>
 </div>
 <div id="missues"></div>
 <div id="mbody">
  <div id="mpal">
   <div id="mpalctl"><input type="text" id="mq" placeholder="search blocks" spellcheck="false">
    <label class="meta" title="show every registered leaf, not just the common ones">
     <input type="checkbox" id="madv">advanced</label></div>
   <div id="mpalbody"></div>
  </div>
  <div id="mws">
   <div id="mtabs"></div>
   <div id="mwork"></div>
  </div>
  <div id="mright">
   <div id="minsp"></div>
   <div id="mmapwrap"><canvas id="mmap"></canvas><div id="mmapnote"></div>
    <div id="mmapctl">
     <button id="mm_boat" title="centre the map on the boat">centre on boat</button>
     <button id="mm_fit" title="fit the mission's points and the boat">fit</button>
     <button id="mm_mode" title="auto: a click moves the selected waypoint, otherwise adds one after the selected block. add: a click always adds one (Shift+click does it for a single click)">click: auto</button>
    </div>
   </div>
  </div>
 </div>
 <div id="mline"></div><div id="mghost"></div>
 <div id="mdlg"><div class="box"></div></div>
"""

JS = r"""
/* ---------------- tab: Mission (the block editor) ----------------
   See mission_tab.py's header. Everything is inside one function so none of its helper
   names can collide with the page's own (el, esc, post, S, tab, toast are the page's). */
(function(){
var SEQ = 0, UNDO_MAX = 50, STAT_MS = 2000, VALID_MS = 1300;
var CAT = null, CATMSG = '';
var M = null;                       /* {pre:[comment], root:node} the open mission */
var cur = {name:'', source:'', dirty:false};
var treeId = '', selId = '', idx = {};
var undo = [], redo = [], lastJson = '';
var missions = [], issues = null, run = null, statTimer = null, validTimer = null, validSeq = 0;
var showAdv = false, query = '', closed = {};
var palMap = {};
var drag = null, mp = null, mpTheme = '';
var MV = {clat:null, clon:null, mpp:0.25, mode:'auto', W:0, H:0, dpr:1, pan:null};
var NEW_XML = '<root BTCPP_format="4" main_tree_to_execute="Main">'
  + '<BehaviorTree ID="Main"><ReactiveSequence name="guard"><IsAutonomous/>'
  + '<Sequence name="mission"><WaitForRunStart/></Sequence></ReactiveSequence></BehaviorTree></root>';
var FALLBACK_CAT = {version:1, categories:[], nodes:[], presets:[], subtrees:[]};

function $(id){ return document.getElementById(id); }
function mk(tag, attrs, kids){
  return {id:'n'+(++SEQ), tag:tag, attrs:attrs||{}, kids:kids||[], pre:[], tail:[], text:''};
}
function clone(o){ return JSON.parse(JSON.stringify(o)); }
function api(path, body){
  return fetch(path, {method:'POST', headers:{'Content-Type':'application/json'},
                      body: JSON.stringify(body||{})})
    .then(function(r){ return r.json(); })
    .catch(function(e){ return {ok:false, message:'request failed: '+e}; });
}
function say(j, okMsg){
  var m = (j && j.message) || (j && j.ok ? okMsg : '') || '';
  if(m) toast(m, !(j && j.ok), j && j.ok ? 3500 : 7000);
  return j;
}
function isNum(v){ return v !== '' && v != null && isFinite(Number(v)); }
function isBB(v){ return /^\{.*\}$/.test(String(v)); }

/* ---------------- the XML model ---------------- */
function conv(el){
  var n = mk(el.tagName, {}, []), pend = [], i, a, c, txt = '';
  for(i = 0; i < el.attributes.length; i++){ a = el.attributes[i]; n.attrs[a.name] = a.value; }
  for(i = 0; i < el.childNodes.length; i++){
    c = el.childNodes[i];
    if(c.nodeType === 8) pend.push(c.data);
    else if(c.nodeType === 1){ var k = conv(c); k.pre = pend; pend = []; n.kids.push(k); }
    else if(c.nodeType === 3 || c.nodeType === 4) txt += c.data;
  }
  n.tail = pend;
  n.text = n.kids.length ? '' : txt.trim();
  return n;
}
function parseXml(text){
  var doc = new DOMParser().parseFromString(text, 'application/xml');
  var err = doc.getElementsByTagName('parsererror');
  if(err.length){
    var why = /error on line[^\n]*/.exec(err[0].textContent);
    throw new Error('not well-formed XML: ' + (why ? why[0] : err[0].textContent.trim().slice(0, 120)));
  }
  var pre = [], rootEl = null, i, c;
  for(i = 0; i < doc.childNodes.length; i++){
    c = doc.childNodes[i];
    if(c.nodeType === 8) pre.push(c.data);
    else if(c.nodeType === 1) rootEl = c;
  }
  if(!rootEl || rootEl.tagName !== 'root') throw new Error('not a BT.CPP file: the top element must be <root>');
  return {pre:pre, root:conv(rootEl)};
}
function xa(v){ return String(v).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/"/g,'&quot;')
  .replace(/\n/g,'&#10;').replace(/\r/g,'&#13;').replace(/\t/g,'&#9;'); }
function xt(v){ return String(v).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;'); }
/* BT.CPP 4.10 types a SubTree literal by its text, so a double must carry a decimal point ("6" would
   load as an int and fail) and an int must not ("8.0" would load as a double). */
function normNum(type, v){
  if(type === 'double'){
    if(/^[+-]?\d+$/.test(v)) return v + '.0';
    if(/^[+-]?\d+\.$/.test(v)) return v + '0';
  }
  if(type === 'int' && /^[+-]?\d+\.0*$/.test(v)) return v.replace(/\.0*$/, '');
  return v;
}
function intBad(type, v){ return type === 'int' && v !== '' && !/^\{.*\}$/.test(v) && !/^[+-]?\d+$/.test(normNum(type, v)); }
/* A SubTree element as written: every declared tunable present (BT.CPP ignores model defaults), numbers
   normalised. Only inside a BehaviorTree: the same tag in TreeNodesModel is a declaration, not a call. */
function subAttrs(n){
  var a = {}, ports = CAT ? defOf(n).ports : [];
  Object.keys(n.attrs).forEach(function(k){ a[k] = n.attrs[k]; });
  ports.forEach(function(p){
    if(p.direction === 'out') return;
    if(a[p.name] == null){ if(p['default'] != null) a[p.name] = normNum(p.type, String(p['default'])); }
    else a[p.name] = normNum(p.type, a[p.name]);
  });
  return a;
}
function ser(n, d, inTree){
  var pad = new Array(d + 1).join('  '), out = '', a = '', at = inTree && n.tag === 'SubTree' ? subAttrs(n) : n.attrs;
  inTree = inTree || n.tag === 'BehaviorTree';
  n.pre.forEach(function(c){ out += pad + '<!--' + c + '-->\n'; });
  Object.keys(at).forEach(function(k){ a += ' ' + k + '="' + xa(at[k]) + '"'; });
  if(!n.kids.length && !n.tail.length)
    return out + pad + '<' + n.tag + a + (n.text ? '>' + xt(n.text) + '</' + n.tag + '>\n' : '/>\n');
  out += pad + '<' + n.tag + a + '>\n';
  n.kids.forEach(function(k){ out += ser(k, d + 1, inTree); });
  n.tail.forEach(function(c){ out += pad + '  <!--' + c + '-->\n'; });
  return out + pad + '</' + n.tag + '>\n';
}
function toXml(){
  var out = '<?xml version="1.0"?>\n';      /* the declaration mission_lib writes too */
  M.pre.forEach(function(c){ out += '<!--' + c + '-->\n'; });
  return out + ser(M.root, 0);
}
function trees(){ return M.root.kids.filter(function(k){ return k.tag === 'BehaviorTree'; }); }
function treeById(id){ return trees().filter(function(t){ return t.attrs.ID === id; })[0] || null; }
function mainId(){
  var m = M.root.attrs.main_tree_to_execute;
  return m || (trees()[0] && trees()[0].attrs.ID) || '';
}
/* the TreeNodesModel element (null when the file has none) */
function modelNode(){ return M.root.kids.filter(function(k){ return k.tag === 'TreeNodesModel'; })[0] || null; }
function walk(n, fn){ fn(n); n.kids.forEach(function(k){ walk(k, fn); }); }
function blockEl(id){ return document.querySelector('#mwork .mblk[data-id="' + id + '"]'); }
function unmark(cls){ document.querySelectorAll('#mwork .' + cls).forEach(function(b){ b.classList.remove(cls); }); }
function curTree(){ return treeById(treeId) || treeById(mainId()) || trees()[0] || null; }
function reindex(){
  idx = {};
  function walk(p, t){ p.kids.forEach(function(k, i){ idx[k.id] = {node:k, parent:p, i:i, tree:t}; walk(k, t); }); }
  trees().forEach(function(t){ idx[t.id] = {node:t, parent:null, i:0, tree:t, isTree:true}; walk(t, t); });
}
function descOf(){
  var all = M.pre.concat(M.root.kids.reduce(function(a, k){ return a.concat(k.pre); }, []));
  for(var i = 0; i < all.length; i++){
    var m = /^\s*MISSION:\s*([\s\S]*?)\s*$/.exec(all[i]);
    if(m) return m[1].replace(/\s*\n\s*/g, ' ');
  }
  return '';
}
function setDesc(text){
  var lists = [M.pre].concat(M.root.kids.map(function(k){ return k.pre; })), i, j;
  text = text.replace(/\s+/g, ' ').trim();
  for(i = 0; i < lists.length; i++) for(j = 0; j < lists[i].length; j++)
    if(/^\s*MISSION:/.test(lists[i][j])){
      if(text) lists[i][j] = ' MISSION: ' + text + ' '; else lists[i].splice(j, 1);
      return;
    }
  if(text) (M.root.kids[0] || M.root).pre.unshift(' MISSION: ' + text + ' ');
}

/* ---------------- the catalogue ---------------- */
function setCatalogue(c){
  CAT = {raw:c, cats:{}, order:[], nodes:{}, presets:c.presets || [], presetById:{}, subtrees:{}};
  (c.categories || []).forEach(function(k){ CAT.cats[k.id] = k; CAT.order.push(k.id); });
  (c.nodes || []).forEach(function(n){ CAT.nodes[n.id] = n; });
  CAT.presets.forEach(function(p){ CAT.presetById[p.id] = p; });
  (c.subtrees || []).forEach(function(s){ CAT.subtrees[s.id] = s; });
}
function portsOfModel(id){
  var model = modelNode(), out = [];
  if(!model) return out;
  model.kids.forEach(function(s){
    if(s.tag !== 'SubTree' || s.attrs.ID !== id) return;
    s.kids.forEach(function(p){
      if(p.tag !== 'input_port' && p.tag !== 'inout_port') return;
      var d = p.attrs['default'];
      out.push({name:p.attrs.name, type: d == null ? 'string' : /^-?\d+$/.test(d) ? 'int' :
                isNum(d) ? 'double' : /^(true|false)$/.test(d) ? 'bool' : 'string',
                'default': d, desc: p.text || p.attrs.description || '', direction:'in'});
    });
  });
  return out;
}
function presetOf(n){
  var best = null, bestN = -1;
  CAT.presets.forEach(function(p){
    if(p.preset_of !== n.tag) return;
    var f = p.fixed || {}, keys = Object.keys(f);
    if(keys.every(function(k){ return n.attrs[k] === String(f[k]); }) && keys.length > bestN){
      best = p; bestN = keys.length; }
  });
  return best;
}
function defOf(n){
  var d, base;
  if(n.tag === 'SubTree'){
    var id = n.attrs.ID, sd = CAT.subtrees[id], own = treeById(id);
    return {kind:'subtree', cat: tasksCat(), label: (sd && sd.label) || id || 'SubTree',
            desc: sd ? sd.desc : own ? 'A subtree of this mission.' : 'No such tree in this mission.',
            ports: (sd && sd.ports && sd.ports.length) ? sd.ports : portsOfModel(id),
            children:'none', known: !!(sd || own)};
  }
  var pr = presetOf(n);
  base = CAT.nodes[n.tag];
  if(pr){
    var all = base ? base.ports : [], show = pr.show_ports;
    return {kind: base ? base.kind : 'action', cat: pr.category, label: pr.label, desc: pr.desc || '',
            ports: show ? show.map(function(s){ return all.filter(function(p){ return p.name === s; })[0]
                    || {name:s, type:'string'}; }) : all,
            children: base ? base.children : 'none', known:true, fixed: pr.fixed || {}};
  }
  if(base) return {kind:base.kind, cat:base.category, label:base.label || base.id, desc:base.desc || '',
                   ports:base.ports || [], children:base.children || 'none', known:true};
  return {kind:'unknown', cat:'', label:n.tag, ports:[], known:false,
          desc:'Not in the catalogue: its attributes are kept exactly as written.',
          children: n.kids.length ? 'many' : 'none'};
}
function tasksCat(){
  return ['tasks', 'task'].filter(function(k){ return CAT.cats[k]; })[0]
    || CAT.order.filter(function(k){ return /task/i.test(k); })[0] || '';
}
function takes(node){
  if(node.tag === 'BehaviorTree') return node.kids.length < 1;
  var c = defOf(node).children;
  return c === 'many' || (c === 'one' && node.kids.length < 1);
}
function portTip(d){
  var t = d.desc || d.label || '';
  (d.ports || []).forEach(function(p){
    t += '\n' + p.name + ' (' + (p.type || 'string') + (p['default'] != null && p['default'] !== '' ?
         ', default ' + p['default'] : '') + (p.required ? ', required' : '') + ')'
         + (p.desc ? ': ' + p.desc : '');
  });
  return t;
}
function lin(c){ c /= 255; return c <= 0.03928 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4); }
function blockStyle(d){
  var c = d.cat && CAT.cats[d.cat] ? String(CAT.cats[d.cat].color || '') : '';
  if(d.kind === 'unknown') return '--bc:var(--muted);--bt:var(--onblk)';
  var m = /^#([0-9a-f]{6})$/i.exec(c), fg = 'var(--onblk)';
  if(!m){ c = 'var(--accent)'; }
  else{
    /* whichever of white and near-black reads better on this colour (WCAG contrast): the same
       in both themes, so day mode needs no second colour table */
    var v = parseInt(m[1], 16), l = 0.2126 * lin(v >> 16 & 255) + 0.7152 * lin(v >> 8 & 255) + 0.0722 * lin(v & 255);
    if((l + 0.05) / 0.0556 > 1.05 / (l + 0.05)) fg = 'var(--onblk-dk)';
  }
  return '--bc:' + c + ';--bt:' + fg;
}

/* ---------------- block summaries ---------------- */
function fmtLL(v){ return isNum(v) ? Number(v).toFixed(5) : String(v); }
function secs(v){ var s = Number(v) / 1000; return (Math.round(s * 100) / 100) + ' s'; }
function summaryOf(n, d){
  var a = n.attrs, parts = [], skip = {name:1, ID:1, _autoremap:1}, f = d.fixed || {};
  Object.keys(f).forEach(function(k){ skip[k] = 1; });
  if(a.lat != null && a.lon != null){ parts.push(fmtLL(a.lat) + ', ' + fmtLL(a.lon)); skip.lat = skip.lon = 1; }
  else if(d.fixed && isGeo(n)) parts.push('no point yet: click the map');
  var keys = d.ports.map(function(p){ return p.name; });
  if(d.kind !== 'subtree' && d.known) keys = keys.concat(Object.keys(a));
  else keys = Object.keys(a);
  keys.forEach(function(k){
    if(skip[k] || a[k] == null || a[k] === '') return;
    skip[k] = 1;
    if(/(^|_)(msec|ms)$/.test(k) && isNum(a[k])) parts.push(secs(a[k]));
    else parts.push(k + ' ' + a[k]);
  });
  var s = parts.join(' · ');
  if(!parts.length && d.kind === 'unknown') s = '';
  return s.length > 90 ? s.slice(0, 88) + '…' : s;
}
function blockHtml(n){
  var d = defOf(n), cont = d.children !== 'none' || n.kids.length, sm = summaryOf(n, d);
  var head = '<div class="hd" title="' + esc(portTip(d)) + '"><span class="lb">' + esc(d.label) + '</span>'
    + (n.attrs.name ? '<span class="nm">&ldquo;' + esc(n.attrs.name) + '&rdquo;</span>' : '')
    + (sm ? '<span class="sm">' + esc(sm) + '</span>' : '')
    + (d.kind === 'unknown' ? '<span class="tag">' + esc(n.tag) + ' (not in catalogue)</span>' : '') + '</div>';
  var cls = 'mblk' + (cont ? ' c' : '') + (n.id === selId ? ' sel' : '');
  var st = '" style="' + blockStyle(d) + '" data-id="' + n.id;
  if(!cont) return '<div class="' + cls + st + '">' + head + '</div>';
  return '<div class="' + cls + st + '">' + head + slotHtml(n) + '<div class="ft"></div></div>';
}
function slotHtml(p){
  var inner = p.kids.map(blockHtml).join('');
  return '<div class="slot" data-slot="' + p.id + '"><div class="kids">' + inner + '</div>'
    + (p.kids.length ? '' : '<div class="hole">drop blocks here</div>') + '</div>';
}

/* ---------------- rendering ---------------- */
function renderTabs(){
  var main = mainId(), h = '';
  /* the tree that runs first, then the rest in file order */
  trees().sort(function(a, b){ return (b.attrs.ID === main) - (a.attrs.ID === main); }).forEach(function(t){
    var id = t.attrs.ID, isMain = id === main;
    h += '<div class="mtab' + (id === treeId ? ' on' : '') + '" data-tree="' + esc(id) + '" title="'
      + (isMain ? 'the tree that runs' : 'double-click to rename') + '">'
      + esc(isMain && id === 'Main' ? 'Main' : id) + (isMain && id !== 'Main' ? ' (main)' : '')
      + (isMain ? '' : '<span class="x" data-del="' + esc(id) + '" title="delete this subtree">&times;</span>')
      + '</div>';
  });
  h += '<div class="mtab" id="maddtree" title="add an empty subtree to this mission">+ subtree</div>'
     + '<div id="mbin" title="drop a block here to delete it (or select it and press Delete)">&#128465; delete</div>';
  $('mtabs').innerHTML = h;
}
function renderWork(){
  var t = curTree(), w = $('mwork');
  if(!t){ w.innerHTML = '<div class="hint">This file has no BehaviorTree.</div>'; return; }
  treeId = t.attrs.ID;
  w.innerHTML = '<div class="mhat">' + esc(t.attrs.ID) + '</div><div class="mroot">' + slotHtml(t) + '</div>';
}
function renderBar(){
  $('mname').textContent = (cur.name ? cur.name : 'unsaved mission')
    + (cur.source === 'prebuilt' ? ' (pre-built, read-only)' : '') + (cur.dirty ? ' *' : '');
  var active = !!(run && run.active);
  $('mstart').disabled = active || !M;
  $('mstop').disabled = !active;
  var st = $('mstat'), r = run;
  st.className = active ? 'live' : '';
  if(!r) st.textContent = '—';
  else if(r.ok === false) st.textContent = 'run status unavailable: ' + (r.message || '?');
  else if(active){
    var f = r.feedback || {};
    st.textContent = 'RUNNING ' + (r.name || '') + ' · ' + (r.state || '') + (f.phase ? ' · ' + f.phase : '')
      + (f.progress != null ? ' ' + Math.round(f.progress * 100) + '%' : '') + (f.warning ? ' · ' + f.warning : '');
  }
  else if(r.result) st.textContent = 'last run: ' + r.result.outcome + (r.result.detail ? ' — ' + r.result.detail : '');
  else st.textContent = 'no mission running';
}
function renderPicker(){
  var sel = $('mpick'), keep = sel.value, h = '';
  ['prebuilt', 'saved'].forEach(function(src){
    var list = missions.filter(function(m){ return m.source === src; });
    if(!list.length) return;
    h += '<optgroup label="' + (src === 'prebuilt' ? 'Pre-built' : 'Saved') + '">';
    list.forEach(function(m){
      h += '<option value="' + esc(src + ':' + m.name) + '" title="' + esc(m.description || '') + '">'
        + esc(m.name) + '</option>'; });
    h += '</optgroup>';
  });
  sel.innerHTML = h || '<option value="">(no missions found)</option>';
  var want = cur.source ? cur.source + ':' + cur.name : keep;
  if(want) sel.value = want;
}
function paletteData(){
  var q = query.trim().toLowerCase(), out = [], seen = {};
  function ok(label, desc, id){ return !q || (label + ' ' + desc + ' ' + id).toLowerCase().indexOf(q) >= 0; }
  function sec(catId){
    if(!seen[catId]){ seen[catId] = {cat:catId, items:[]}; out.push(seen[catId]); }
    return seen[catId];
  }
  CAT.order.forEach(sec);
  CAT.presets.forEach(function(p){
    if(ok(p.label, p.desc || '', p.id)) sec(p.category).items.push({k:'p:' + p.id, label:p.label, desc:p.desc || '',
      src:{t:'preset', id:p.id}, tip: portTip({desc:p.desc, ports:(CAT.nodes[p.preset_of] || {ports:[]}).ports
        .filter(function(x){ return !p.show_ports || p.show_ports.indexOf(x.name) >= 0; })})}); });
  Object.keys(CAT.nodes).forEach(function(id){
    var n = CAT.nodes[id];
    if(n.kind === 'subtree' || (n.advanced && !showAdv)) return;
    if(ok(n.label || id, n.desc || '', id)) sec(n.category).items.push({k:'n:' + id, label:n.label || id,
      desc:n.desc || '', src:{t:'node', id:id}, tip:portTip(n)}); });
  var tc = tasksCat() || 'tasks';
  (CAT.raw.subtrees || []).forEach(function(s){
    if(ok(s.label, s.desc || '', s.id)) sec(tc).items.push({k:'s:' + s.id, label:s.label, desc:s.desc || '',
      src:{t:'sub', id:s.id}, tip:portTip({desc:s.desc, ports:s.ports})}); });
  if(M) trees().forEach(function(t){
    var id = t.attrs.ID;
    if(id === mainId() || id === treeId || CAT.subtrees[id]) return;
    if(ok(id, 'subtree of this mission', id)) sec(tc).items.push({k:'t:' + id, label:id,
      desc:'A subtree of this mission.', src:{t:'tree', id:id}, tip:id}); });
  return out.filter(function(s){ return s.items.length; });
}
function renderPalette(){
  var h = '';
  palMap = {};
  if(!CAT){ $('mpalbody').innerHTML = '<div class="hint">' + esc(CATMSG || 'loading the block catalogue…') + '</div>'; return; }
  paletteData().forEach(function(s){
    var c = CAT.cats[s.cat] || {label: s.cat === 'tasks' || !s.cat ? 'Tasks' : s.cat, color:''};
    var fake = {cat:s.cat, kind:'action'}, shut = closed[s.cat] && !query;
    h += '<div class="mcat" style="' + blockStyle(fake) + '"><div class="ch" data-cat="' + esc(s.cat) + '">'
      + '<span class="sw"></span>' + (shut ? '&#9656; ' : '&#9662; ') + esc(c.label || s.cat) + '</div>';
    if(!shut) s.items.forEach(function(it){
      palMap[it.k] = it.src;
      h += '<div class="mpi" data-k="' + esc(it.k) + '" title="' + esc(it.tip) + '"><div class="l">' + esc(it.label)
        + '</div><div class="d">' + esc(it.desc) + '</div></div>'; });
    h += '</div>';
  });
  var bad = (CAT.raw.problems || []);
  if(bad.length) h = '<div class="hint" style="color:var(--warn)" title="' + esc(bad.join('\n')) + '">&#9888; '
    + bad.length + ' library file(s) could not be read (hover)</div>' + h;
  $('mpalbody').innerHTML = h || '<div class="hint">nothing matches</div>';
}

/* The inspector is rebuilt only when the SELECTION changes, never on an edit: it holds the
   field being typed in. */
function badValue(p, v){
  var t = p.type || 'string';
  return v != null && v !== '' && ((t === 'double' && !isNum(v) && !isBB(v)) || intBad(t, v) || (t === 'bb' && !isBB(v)));
}
function portRow(n, p, locked){
  var v = n.attrs[p.name], has = v != null, ph = (p['default'] != null && p['default'] !== '') ?
      String(p['default']) : (p.required ? 'required' : ''), t = p.type || 'string', inp;
  if(p.name === 'lat' || p.name === 'lon') ph = 'degrees (required)';
  if(n.tag === 'SubTree' && p['default'] === '') ph = 'nav_* default';      /* blank = use the nav_* parameter */
  if(t === 'bb' && !ph) ph = '{name}';
  var bad = badValue(p, v);
  var da = ' data-port="' + esc(p.name) + '"';
  if(t === 'bool' || t === 'enum'){
    var opts = t === 'bool' ? ['true', 'false'] : (p.choices || []);
    if(has && v !== '' && opts.indexOf(v) < 0) opts = opts.concat([v]);
    inp = '<select' + da + '><option value="">' + esc(ph ? '(default: ' + ph + ')' : '(default)') + '</option>'
      + opts.map(function(o){ return '<option value="' + esc(o) + '"' + (has && v === o ? ' selected' : '') + '>'
        + esc(o) + '</option>'; }).join('') + '</select>';
  }
  else inp = '<input type="text" spellcheck="false"' + da + ' placeholder="' + esc(ph) + '" value="'
      + esc(has ? v : '') + '"' + (bad ? ' class="bad"' : '') + (locked ? ' disabled' : '') + '>';
  return '<div class="pr"><label>' + esc(p.name) + ' <span class="ty">' + esc(t)
    + (p.direction === 'out' ? ' · out' : '') + (p.required ? ' · required' : '') + '</span></label>' + inp
    + (p.desc ? '<div class="ds">' + esc(p.desc) + '</div>' : '') + '</div>';
}
function renderInsp(){
  var box = $('minsp'), e = selId && idx[selId], h;
  if(!e || e.isTree){
    box.innerHTML = '<div class="hint">Select a block to edit its ports. Drag blocks from the palette; ' +
      'Delete removes the selected block; Ctrl+Z undoes. Click the map to add or move a GPS point.</div>';
    return;
  }
  var n = e.node, d = defOf(n);
  h = '<div class="ttl">' + esc(d.label) + '</div><div class="hint">' + esc(d.desc || '') + '</div>';
  h += '<div class="pr"><label>name <span class="ty">optional label</span></label><input type="text" data-port="name" value="'
    + esc(n.attrs.name || '') + '" spellcheck="false"></div>';
  if(n.tag === 'SubTree'){
    h += '<div class="pr"><label>subtree <span class="ty">' + esc(n.attrs.ID || '') + '</span></label>'
      + (treeById(n.attrs.ID) ? '<button data-act="open">open this subtree</button>' : '') + '</div>';
    if(d.ports.length) h += '<h3>Tunables</h3>';
  }
  d.ports.forEach(function(p){ h += portRow(n, p, false); });
  if(d.kind === 'unknown' || !d.known){
    h += '<h3>Attributes</h3>';
    Object.keys(n.attrs).forEach(function(k){
      if(k === 'name' && d.kind !== 'unknown') return;
      h += '<div class="raw"><input value="' + esc(k) + '" disabled style="flex:0 0 38%"><input data-raw="'
        + esc(k) + '" value="' + esc(n.attrs[k]) + '" spellcheck="false"><button data-rm="' + esc(k)
        + '" title="remove">&times;</button></div>'; });
    h += '<div class="raw"><input id="mnewk" placeholder="attribute" style="flex:0 0 38%"><input id="mnewv" placeholder="value">'
      + '<button data-act="addattr">add</button></div>';
  }
  h += '<div class="acts"><button data-act="dup">Duplicate</button><button class="danger" data-act="del">Delete</button></div>';
  box.innerHTML = h;
}

/* ---------------- editing: every mutation ends in commit() ----------------
   commit() compares the whole model with the last committed one, so undo is exact for
   whatever kind of change it was and costs one JSON.stringify of a few KB. */
function commit(opts){
  opts = opts || {};
  var j = JSON.stringify(M);
  if(j !== lastJson){
    undo.push(lastJson); if(undo.length > UNDO_MAX) undo.shift();
    redo = []; lastJson = j; cur.dirty = true; scheduleValidate();
  }
  refresh(opts);
}
function refresh(opts){
  opts = opts || {};
  reindex();
  if(selId && !idx[selId]) selId = '';
  renderTabs(); renderWork(); renderBar(); renderPalette();
  if(!opts.keepInsp) renderInsp();
  drawMap();
}
function restore(json){
  M = JSON.parse(json); lastJson = json; reindex();
  if(!treeById(treeId)) treeId = mainId();
  refresh({}); scheduleValidate();
}
function doUndo(){ if(!undo.length) return; redo.push(lastJson); restore(undo.pop()); cur.dirty = true; renderBar(); }
function doRedo(){ if(!redo.length) return; undo.push(lastJson); restore(redo.pop()); cur.dirty = true; renderBar(); }
function select(id){
  selId = id || '';
  unmark('sel');
  var b = id && blockEl(id);
  if(b) b.classList.add('sel');
  renderInsp(); drawMap();
}
function detach(n){
  var e = idx[n.id];
  e.parent.kids.splice(e.i, 1);
  return e;
}
function removeBlock(id){
  var e = idx[id]; if(!e || e.isTree) return;
  detach(e.node); if(selId === id) selId = '';
  commit();
}
function reid(n){ n.id = 'n' + (++SEQ); n.kids.forEach(reid); return n; }
function dupBlock(id){
  var e = idx[id]; if(!e || e.isTree) return;
  var c = reid(clone(e.node));
  e.parent.kids.splice(e.i + 1, 0, c);
  selId = c.id; commit();
}
function ensureModel(){
  var m = modelNode();
  if(!m){ m = mk('TreeNodesModel'); M.root.kids.push(m); }
  return m;
}
function addTreeNode(t){
  var at = modelNode();
  M.root.kids.splice(at ? M.root.kids.indexOf(at) : M.root.kids.length, 0, t);
}
/* Inline a catalogue subtree (and any catalogue subtree it calls) so the saved mission is
   self-contained, with its tunables declared in TreeNodesModel for the next load. */
function ensureTree(sd){
  if(!sd || treeById(sd.id)) return;
  var doc = parseXml('<root>' + sd.xml + '</root>');
  doc.root.kids.filter(function(k){ return k.tag === 'BehaviorTree'; }).forEach(function(t){
    if(!treeById(t.attrs.ID)) addTreeNode(t); });
  var model = ensureModel();
  if(!model.kids.some(function(s){ return s.tag === 'SubTree' && s.attrs.ID === sd.id; })){
    var s = mk('SubTree', {ID:sd.id});
    (sd.ports || []).forEach(function(p){
      var a = {name:p.name};
      if(p['default'] != null && p['default'] !== '') a['default'] = String(p['default']);
      var ip = mk(p.direction === 'out' ? 'output_port' : 'input_port', a); ip.text = p.desc || '';
      s.kids.push(ip); });
    model.kids.push(s);
  }
  var seen = {};
  walk(treeById(sd.id) || mk('x'), function(n){
    if(n.tag === 'SubTree' && CAT.subtrees[n.attrs.ID] && !seen[n.attrs.ID]){
      seen[n.attrs.ID] = 1; ensureTree(CAT.subtrees[n.attrs.ID]); }
  });
}
function subBlock(id, lib){
  var a = {ID:id, _autoremap:'true'}, sd = CAT.subtrees[id], ports = [];
  if(lib && sd){ ensureTree(sd); ports = sd.ports || []; }
  else ports = portsOfModel(id);
  ports.forEach(function(p){ if(p.direction !== 'out' && p['default'] != null) a[p.name] = normNum(p.type, String(p['default'])); });
  return mk('SubTree', a);
}
/* A required choice with no default (TaskAttempt's token, SetTier's tier) is written with its
   first choice rather than left for the validator to refuse. */
function requiredChoices(tag, attrs){
  ((CAT.nodes[tag] || {}).ports || []).forEach(function(p){
    if(p.required && p.type === 'enum' && p['default'] == null && p.choices && p.choices.length
       && attrs[p.name] == null) attrs[p.name] = p.choices[0];
  });
  return attrs;
}
function makeNew(src){
  if(src.t === 'preset'){ var p = CAT.presetById[src.id], a = {};
    Object.keys(p.fixed || {}).forEach(function(k){ a[k] = String(p.fixed[k]); });
    return mk(p.preset_of, requiredChoices(p.preset_of, a)); }
  if(src.t === 'node') return mk(src.id, requiredChoices(src.id, {}));
  if(src.t === 'sub') return subBlock(src.id, true);
  if(src.t === 'tree') return subBlock(src.id, false);
  return null;
}
function insertNew(src, parent, i){
  var n = makeNew(src); if(!n) return null;
  parent.kids.splice(i == null ? parent.kids.length : i, 0, n);
  selId = n.id; commit(); return n;
}
function isWithin(id, ancestorId){
  var e = idx[id];
  while(e){ if(e.node.id === ancestorId) return true; e = e.parent && idx[e.parent.id]; }
  return false;
}
/* Place `n` (new or detached) at tgt = {pid, i} or wrap tgt.wrap. */
function applyDrop(src, tgt){
  if(tgt.bin){ if(src.t === 'move') removeBlock(src.id); return; }
  var n, old = null;
  if(src.t === 'move'){ n = idx[src.id].node; old = {p:idx[src.id].parent, i:idx[src.id].i}; }
  else{ n = makeNew(src); if(!n) return; }
  if(tgt.wrap){
    var w = idx[tgt.wrap]; if(!w || w.isTree) return;
    n.kids = [w.node]; w.parent.kids[w.i] = n;
  }
  else{
    var P = idx[tgt.pid].node, i = tgt.i;
    if(old){ detach(n); if(old.p === P && old.i < i) i--; }
    P.kids.splice(i, 0, n);
  }
  selId = n.id; commit();
}

/* ---------------- drag and drop (pointer events, so it works with a mouse or a finger) ---------------- */
function slotGeom(){
  var out = [];
  document.querySelectorAll('#mwork .slot').forEach(function(s){
    var kidsEl = s.querySelector(':scope > .kids'), items = [], r = s.getBoundingClientRect(), ys = [];
    if(kidsEl) for(var i = 0; i < kidsEl.children.length; i++)
      if(kidsEl.children[i].classList.contains('mblk')) items.push(kidsEl.children[i]);
    if(!items.length) ys = [(r.top + r.bottom) / 2];
    else items.forEach(function(it, k){
      var ir = it.getBoundingClientRect();
      if(k === 0) ys.push(ir.top - 2);
      ys.push(k < items.length - 1 ? (ir.bottom + items[k + 1].getBoundingClientRect().top) / 2 : ir.bottom + 2);
    });
    out.push({pid:s.dataset.slot, x0:r.left + 4, x1:r.right, ys:ys});
  });
  return out;
}
function dragCanWrap(src){
  var n = src.t === 'move' ? idx[src.id].node : null, c;
  if(n) return n.kids.length === 0 && defOf(n).children !== 'none';
  c = src.t === 'preset' ? (CAT.nodes[CAT.presetById[src.id].preset_of] || {}).children
    : src.t === 'node' ? (CAT.nodes[src.id] || {}).children : 'none';
  return c === 'one' || c === 'many';
}
function pickTarget(px, py){
  var br = $('mbin').getBoundingClientRect();
  if(px >= br.left && px <= br.right && py >= br.top && py <= br.bottom) return {bin:true};
  var hds = document.querySelectorAll('#mwork .mblk > .hd'), i, best = null;
  if(drag.canWrap) for(i = 0; i < hds.length; i++){
    var r = hds[i].getBoundingClientRect(), id = hds[i].parentNode.dataset.id;
    if(px >= r.left && px <= r.right && py >= r.top + r.height * 0.3 && py <= r.top + r.height * 0.7
       && !(drag.src.t === 'move' && isWithin(id, drag.src.id))) return {wrap:id, el:hds[i].parentNode};
  }
  drag.geom.forEach(function(g){
    var host = idx[g.pid] && idx[g.pid].node;
    if(!host) return;
    if(drag.src.t === 'move' && isWithin(g.pid, drag.src.id)) return;
    if(!takes(host)) return;
    g.ys.forEach(function(y, k){
      var dx = px < g.x0 ? g.x0 - px : px > g.x1 ? px - g.x1 : 0, sc = Math.abs(py - y) + dx * 0.6;
      if(!best || sc < best.sc) best = {sc:sc, pid:g.pid, i:k, y:y, x0:g.x0, x1:g.x1};
    });
  });
  return best && best.sc < 70 ? best : null;
}
function showTarget(t){
  var ln = $('mline'), bin = $('mbin');
  unmark('wrapt');
  bin.classList.toggle('hot', !!(t && t.bin));
  if(t && t.wrap){ t.el.classList.add('wrapt'); ln.style.display = 'none'; return; }
  if(!t || t.bin){ ln.style.display = 'none'; return; }
  ln.style.display = 'block'; ln.style.left = t.x0 + 'px'; ln.style.top = (t.y - 2) + 'px';
  ln.style.width = Math.max(40, t.x1 - t.x0) + 'px';
}
/* The block and palette handlers cancel pointerdown (no text selection while dragging), which also
   stops the browser blurring a field in the inspector, so its change would never commit. */
function settle(){
  var a = document.activeElement;
  if(a && a !== document.body && $('minsp').contains(a)){
    a.blur();
    if(M && JSON.stringify(M) !== lastJson) commit({keepInsp:true});
  }
}
function dragStart(ev, src, label, styleText){
  settle();
  drag = {src:src, sx:ev.clientX, sy:ev.clientY, on:false, label:label, style:styleText, tgt:null};
}
function dragMove(ev){
  if(!drag) return;
  if(!drag.on){
    if(Math.abs(ev.clientX - drag.sx) + Math.abs(ev.clientY - drag.sy) < 6) return;
    drag.on = true; drag.geom = slotGeom(); drag.canWrap = dragCanWrap(drag.src);
    var g = $('mghost'); g.textContent = drag.label; g.setAttribute('style', drag.style); g.style.display = 'block';
    if(drag.src.t === 'move'){
      var b = blockEl(drag.src.id);
      if(b) b.classList.add('ghosted');
    }
  }
  var gh = $('mghost'); gh.style.left = (ev.clientX + 12) + 'px'; gh.style.top = (ev.clientY + 6) + 'px';
  drag.tgt = pickTarget(ev.clientX, ev.clientY); showTarget(drag.tgt);
}
function dragEnd(ev, cancel){
  if(!drag) return;
  var d = drag; drag = null;
  $('mghost').style.display = 'none'; showTarget(null);
  unmark('ghosted');
  if(!d.on){                                       /* a click, not a drag */
    if(cancel) return;
    if(d.src.t === 'move') select(d.src.id); else clickAdd(d.src);
    return;
  }
  if(!cancel && d.tgt) applyDrop(d.src, d.tgt);
}
/* A click on a palette block adds it after the selected block, or at the end of the
   selected container, or at the end of the tree's first Sequence. */
function insertionPoint(){
  var e = selId && idx[selId];
  if(e && !e.isTree){
    if(takes(e.node)) return {p:e.node, i:e.node.kids.length};
    return takes(e.parent) ? {p:e.parent, i:e.i + 1} : null;
  }
  var t = curTree(); if(!t) return null;
  var found = null;
  walk(t, function(n){ if(!found && n.tag === 'Sequence') found = n; });
  if(found) return {p:found, i:found.kids.length};
  var r = t.kids[0];
  if(r && takes(r)) return {p:r, i:r.kids.length};
  if(!r) return {p:t, i:0};
  return null;
}
function clickAdd(src){
  var at = insertionPoint();
  if(!at){ toast('nothing to add it to: select a container block (or drag it in)', true); return; }
  insertNew(src, at.p, at.i);
}

/* ---------------- inspector events ---------------- */
function setAttr(n, name, value){
  var port = defOf(n).ports.filter(function(p){ return p.name === name; })[0];
  if(value === '' || value == null){
    if(n.tag === 'SubTree' && port && port['default'] != null) value = String(port['default']);
    else{ delete n.attrs[name]; return true; }
  }
  if(port && !isBB(value)){
    if(intBad(port.type, value)) return false;
    value = normNum(port.type, value);
  }
  n.attrs[name] = value;
  return true;
}
function inspInput(ev, final){
  var t = ev.target, n = selId && idx[selId] && idx[selId].node;
  if(!n) return;
  if(t.dataset.port != null){
    var port = defOf(n).ports.filter(function(p){ return p.name === t.dataset.port; })[0];
    var took = setAttr(n, t.dataset.port, t.value);
    if(t.tagName === 'INPUT') t.classList.toggle('bad', !!(port && badValue(port, t.value)));
    if(final && t.tagName === 'INPUT'){          /* show what was kept: "6" -> "6.0", a refused decimal reverts */
      if(!took) toast(t.dataset.port + ' is a whole-number port: no decimal point', true);
      t.value = n.attrs[t.dataset.port] != null ? n.attrs[t.dataset.port] : '';
      t.classList.remove('bad');
    } }
  else if(t.dataset.raw != null) n.attrs[t.dataset.raw] = t.value;
  else return;
  if(final) commit({keepInsp:true});
  else{ cur.dirty = true; renderBar(); renderWork(); drawMap(); }
}
function inspClick(ev){
  var b = ev.target.closest('button'); if(!b || !selId) return;
  var n = idx[selId] && idx[selId].node;
  if(b.dataset.act === 'del') removeBlock(selId);
  else if(b.dataset.act === 'dup') dupBlock(selId);
  else if(b.dataset.act === 'open' && n){ treeId = n.attrs.ID; selId = ''; refresh({}); }
  else if(b.dataset.act === 'addattr'){
    var k = $('mnewk').value.trim(), v = $('mnewv').value;
    if(!/^[A-Za-z_][\w.:-]*$/.test(k)){ toast('that is not a valid attribute name', true); return; }
    n.attrs[k] = v; commit();
  }
  else if(b.dataset.rm != null && n){ delete n.attrs[b.dataset.rm]; commit(); }
}

/* ---------------- subtree tabs ---------------- */
function uniqueId(base){
  var id = base, k = 1;
  while(treeById(id)) id = base + (++k);
  return id;
}
function askText(title, label, value, check){
  return dlg({title:title, body:'<div class="frm"><label>' + esc(label) + '</label><input id="mtx" spellcheck="false" value="'
      + esc(value || '') + '" style="min-width:260px"></div>', focus:'mtx',
    buttons:[{id:'ok', label:'OK', cls:'go', check:function(box){ return check ? check(box.querySelector('#mtx').value.trim()) : null; }},
             {id:'no', label:'Cancel'}]})
    .then(function(r){ return r.id === 'ok' ? r.box.querySelector('#mtx').value.trim() : null; });
}
function idCheck(old){
  return function(v){
    if(!/^[A-Za-z_][A-Za-z0-9_]*$/.test(v)) return 'use letters, digits and underscores, not starting with a digit';
    if(v !== old && treeById(v)) return 'there is already a tree called ' + v;
    return null;
  };
}
function addTree(){
  askText('New subtree', 'name', uniqueId('Sub'), idCheck('')).then(function(v){
    if(!v) return;
    addTreeNode(mk('BehaviorTree', {ID:v}));
    treeId = v; selId = ''; commit();
  });
}
function renameTree(id){
  askText('Rename tree', 'new name', id, idCheck(id)).then(function(v){
    if(!v || v === id) return;
    trees().forEach(function(t){
      if(t.attrs.ID === id) t.attrs.ID = v;
      walk(t, function(n){ if(n.tag === 'SubTree' && n.attrs.ID === id) n.attrs.ID = v; });
    });
    var model = modelNode();
    if(model) model.kids.forEach(function(s){ if(s.tag === 'SubTree' && s.attrs.ID === id) s.attrs.ID = v; });
    if(M.root.attrs.main_tree_to_execute === id) M.root.attrs.main_tree_to_execute = v;
    if(treeId === id) treeId = v;
    commit();
  });
}
function deleteTree(id){
  var t = treeById(id), users = 0;
  if(!t || id === mainId()) return;
  trees().forEach(function(x){ walk(x, function(n){ if(n.tag === 'SubTree' && n.attrs.ID === id) users++; }); });
  if(users){ toast(id + ' is used by ' + users + ' SubTree block(s): remove those first', true); return; }
  dlg({title:'Delete subtree', body:'Delete the tree ' + esc(id) + ' and everything in it?',
       buttons:[{id:'ok', label:'Delete', cls:'danger'}, {id:'no', label:'Cancel'}]}).then(function(r){
    if(r.id !== 'ok') return;
    M.root.kids.splice(M.root.kids.indexOf(t), 1);
    var model = modelNode();
    if(model) model.kids = model.kids.filter(function(s){ return !(s.tag === 'SubTree' && s.attrs.ID === id); });
    if(treeId === id) treeId = mainId();
    selId = ''; commit();
  });
}

/* ---------------- dialogs ---------------- */
function dlg(o){
  return new Promise(function(resolve){
    var host = $('mdlg'), box = host.querySelector('.box'), h = '<h3>' + esc(o.title) + '</h3><div>' + o.body + '</div>'
      + '<div class="err" id="mdlgerr"></div><div class="mbt">';
    o.buttons.forEach(function(b){ h += '<button data-b="' + b.id + '"' + (b.cls ? ' class="' + b.cls + '"' : '') + '>'
      + esc(b.label) + '</button>'; });
    box.innerHTML = h + '</div>'; host.style.display = 'flex';
    function done(id){
      host.style.display = 'none'; host.onclick = null; document.removeEventListener('keydown', onkey, true);
      resolve({id:id, box:box});
    }
    function press(id){
      var b = o.buttons.filter(function(x){ return x.id === id; })[0], why = b && b.check ? b.check(box) : null;
      if(why){ $('mdlgerr').textContent = why; return; }
      done(id);
    }
    function onkey(ev){
      if(ev.key === 'Escape'){ ev.preventDefault(); ev.stopPropagation(); done(''); }
      else if(ev.key === 'Enter' && ev.target.tagName === 'INPUT'){ ev.preventDefault(); press(o.buttons[0].id); }
    }
    box.onclick = function(ev){ var b = ev.target.closest('button[data-b]'); if(b) press(b.dataset.b); };
    host.onclick = function(ev){ if(ev.target === host) done(''); };
    document.addEventListener('keydown', onkey, true);
    var f = o.focus && box.querySelector('#' + o.focus); if(f){ f.focus(); if(f.select) f.select(); }
  });
}

/* ---------------- toolbar actions ---------------- */
function confirmDiscard(){
  if(!cur.dirty) return Promise.resolve(true);
  return dlg({title:'Unsaved changes', body:'The open mission has changes that are not saved. Discard them?',
              buttons:[{id:'ok', label:'Discard', cls:'danger'}, {id:'no', label:'Keep editing'}]})
    .then(function(r){ return r.id === 'ok'; });
}
function openXml(xml, name, source){
  var doc = parseXml(xml);
  M = doc; cur = {name:name || '', source:source || '', dirty:false};
  reindex(); treeId = mainId(); selId = ''; undo = []; redo = []; lastJson = JSON.stringify(M);
  issues = null; $('mdesc').value = descOf();
  refresh({}); renderIssues(); scheduleValidate(0);
  var t = curTree(); MV.clat = null; if(t) mapFit();
}
function loadPicked(){
  var v = $('mpick').value; if(!v) return;
  var src = v.split(':')[0], name = v.slice(src.length + 1);
  confirmDiscard().then(function(ok){
    if(!ok) return;
    api('/mission/load', {name:name, source:src}).then(function(j){
      if(!j.ok){ say(j); return; }
      try{ openXml(j.xml, j.name || name, j.source || src); toast('loaded ' + name, false, 2500); }
      catch(e){ toast('cannot open ' + name + ': ' + e.message, true, 8000); }
    });
  });
}
function newMission(){
  confirmDiscard().then(function(ok){ if(ok) openXml(NEW_XML, '', ''); });
}
function refreshList(){
  return api('/mission/list', {}).then(function(j){
    missions = (j && j.ok && j.missions) || []; renderPicker();
    if(j && !j.ok) toast(j.message || 'cannot list missions', true);
  });
}
function doSave(name, overwrite){
  api('/mission/save', {name:name, xml:toXml(), overwrite:overwrite}).then(function(j){
    say(j, 'saved ' + name);
    if(j.ok){ cur.name = name; cur.source = 'saved'; cur.dirty = false; refreshList().then(renderBar); }
  });
}
function saveMission(asNew){
  if(!M) return;
  if(!asNew && cur.source === 'saved' && cur.name){ doSave(cur.name, true); return; }
  var def = cur.name ? (cur.source === 'prebuilt' ? cur.name + '_copy' : cur.name) : 'my_mission';
  askText(cur.source === 'prebuilt' ? 'Save as (pre-built missions are read-only)' : 'Save mission as', 'name', def,
    function(v){ return /^[A-Za-z0-9_-]{1,64}$/.test(v) ? null : 'letters, digits, _ and - only, up to 64'; })
    .then(function(v){
      if(!v) return;
      var exists = missions.some(function(m){ return m.source === 'saved' && m.name === v; });
      if(!exists){ doSave(v, false); return; }
      dlg({title:'Overwrite?', body:'A saved mission called ' + esc(v) + ' already exists. Replace it?',
           buttons:[{id:'ok', label:'Overwrite', cls:'danger'}, {id:'no', label:'Cancel'}]}).then(function(r){
        if(r.id === 'ok') doSave(v, true); });
    });
}
function validateNow(){
  var seq = ++validSeq;
  return api('/mission/validate', {xml:toXml()}).then(function(j){
    if(seq !== validSeq) return;
    issues = j; renderIssues(); markFlags();
  });
}
function scheduleValidate(ms){
  clearTimeout(validTimer);
  if(M) validTimer = setTimeout(validateNow, ms == null ? VALID_MS : ms);
}
function renderIssues(){
  var box = $('missues'), j = issues, h = '';
  if(!j){ box.style.display = 'none'; return; }
  if(j.ok === false && !j.errors){ h = '<div class="e">' + esc(j.message || 'validation unavailable') + '</div>'; }
  else{
    (j.errors || []).forEach(function(m){ h += '<div class="e">&#10007; ' + esc(m) + '</div>'; });
    (j.warnings || []).forEach(function(m){ h += '<div class="w">&#9888; ' + esc(m) + '</div>'; });
    if(!h) h = '<div class="ok">&#10003; valid, no warnings</div>';
  }
  box.innerHTML = h; box.style.display = 'block';
}
/* A message starts with the block's path: "Main > guard > mission > NavigateTo: ...", the tree's
   ID, then each ancestor's name="" or tag (a SubTree as "SubTree <ID>"). Siblings the path cannot
   tell apart are all returned. */
function segMatches(k, seg){
  return k.attrs.name === seg || k.tag === seg || (k.tag === 'SubTree' && seg === 'SubTree ' + k.attrs.ID);
}
function issueBlocks(msg){
  var m = /^(.+?): /.exec(msg), segs = m ? m[1].split(' > ') : [], level = [];
  if(segs.length < 2 || !treeById(segs[0])) return [];
  level = [treeById(segs[0])];
  for(var i = 1; i < segs.length && level.length; i++){
    var next = [];
    level.forEach(function(p){ p.kids.forEach(function(k){ if(segMatches(k, segs[i])) next.push(k); }); });
    level = next;
  }
  return level;
}
function focusIssue(text){
  var hit = issueBlocks(text)[0];
  if(!hit){ toast('that message does not name a block', false, 2500); return; }
  treeId = idx[hit.id].tree.attrs.ID; selId = hit.id; refresh({});
  var b = blockEl(hit.id);
  if(b) b.scrollIntoView({block:'nearest'});
}
function markFlags(){
  unmark('flag');
  if(!issues) return;
  (issues.errors || []).forEach(function(m){
    issueBlocks(m).forEach(function(k){
      var b = blockEl(k.id);
      if(b) b.classList.add('flag'); }); });
}
function showXml(){
  if(!M) return;
  dlg({title:'Mission XML (BT.CPP v4)', body:'<textarea id="mxt" spellcheck="false"></textarea>', focus:'mxt',
       buttons:[{id:'ok', label:'Apply', cls:'go', check:function(box){
         try{ parseXml(box.querySelector('#mxt').value); return null; }catch(e){ return e.message; } }},
         {id:'no', label:'Close'}]}).then(function(r){
    if(r.id !== 'ok') return;
    var keepName = cur.name, keepSrc = cur.source;
    M = parseXml(r.box.querySelector('#mxt').value); reindex();
    if(!treeById(treeId)) treeId = mainId();
    $('mdesc').value = descOf(); selId = ''; commit();
    cur.name = keepName; cur.source = keepSrc; renderBar();
  });
  $('mxt').value = toXml();
}
function downloadXml(){
  if(!M) return;
  var blob = new Blob([toXml()], {type:'application/xml'}), a = document.createElement('a');
  a.href = URL.createObjectURL(blob); a.download = (cur.name || 'mission') + '.xml';
  document.body.appendChild(a); a.click();
  setTimeout(function(){ URL.revokeObjectURL(a.href); a.remove(); }, 500);
}
function uploadXml(file){
  if(!file) return;
  confirmDiscard().then(function(ok){
    if(!ok) return;
    var rd = new FileReader();
    rd.onload = function(){
      try{ openXml(String(rd.result), file.name.replace(/\.xml$/i, ''), ''); cur.dirty = true; renderBar();
           toast('opened ' + file.name + ' (not saved yet)', false, 3500); }
      catch(e){ toast('cannot open ' + file.name + ': ' + e.message, true, 8000); }
    };
    rd.readAsText(file);
  });
}
function usesTag(tag){
  var hit = false;
  trees().forEach(function(t){ walk(t, function(n){ if(n.tag === tag) hit = true; }); });
  return hit;
}
function startDialog(){
  if(!M) return;
  var waits = usesTag('WaitForRunStart');
  dlg({title:'Start mission', focus:'mtier', body:'<div class="frm"><label>mission</label><b>' + esc(cur.name || 'unsaved mission')
    + '</b><label>tier</label><select id="mtier"><option value="core">core</option><option value="advanced">advanced</option>'
    + '<option value="disruptive">disruptive</option></select><label>timeout (s)</label>'
    + '<input id="mtmo" value="1800" style="width:90px"></div><p style="margin:10px 0 0;max-width:420px">'
    + (waits ? 'This mission <b>waits for RoboCommand</b>: it holds position until RunStart arrives.'
             : 'This mission does <b>not</b> wait for RoboCommand (no Wait for RunStart block): it starts moving as soon as the boat is armed in GUIDED.')
    + '</p>' + (issues && issues.ok === false && (issues.errors || []).length ?
        '<p style="color:var(--bad);max-width:420px">This mission has ' + issues.errors.length
        + ' validation error(s) listed under the toolbar: Start will be refused.</p>' : '')
    + '<p class="hint" style="max-width:420px">The boat must be armed and in GUIDED. The RC SB switch is the only e-stop.</p>',
    buttons:[{id:'ok', label:'Start', cls:'go', check:function(box){
      var t = Number(box.querySelector('#mtmo').value);
      return (isFinite(t) && t >= 1 && t <= 86400) ? null : 'timeout must be between 1 and 86400 seconds'; }},
      {id:'no', label:'Cancel'}]}).then(function(r){
    if(r.id !== 'ok') return;
    api('/mission/start', {xml:toXml(), name:cur.name || 'unsaved', tier:r.box.querySelector('#mtier').value,
      timeout_s:Number(r.box.querySelector('#mtmo').value)}).then(function(j){ say(j, 'started'); pollRun(); });
  });
}
function stopRun(){ api('/mission/stop', {}).then(function(j){ say(j, 'stop requested'); pollRun(); }); }
function pollRun(){
  api('/mission/status', {}).then(function(j){ run = j; renderBar(); });
}

/* ---------------- the map (offline: no tiles, a local equirectangular projection) ---------------- */
function readPal(){
  var th = document.documentElement.getAttribute('data-theme') || 'night';
  if(mp && th === mpTheme) return;
  var s = getComputedStyle(document.documentElement);
  function v(n){ return s.getPropertyValue(n).trim(); }
  mp = {bg:v('--bg'), grid:v('--rowline'), dim:v('--dim'), fg:v('--strong'), accent:v('--accentline'), ok:v('--ok'),
        warn:v('--warn'), bad:v('--bad'), boat:v('--vstrong'),
        lbl:{red:v('--lbl-red'), green:v('--lbl-green'), yellow:v('--lbl-yellow'), blue:v('--lbl-blue'),
             black:v('--lbl-black'), none:v('--lbl-none')}};
  mpTheme = th;
}
/* A block whose position is a literal lat/lon: it has both ports, and where the node has a
   selector (NavigateTo's target, CircleBuoy's anchor) the selector says "fix". */
var FIX_SELECTOR = {NavigateTo:'target', CircleBuoy:'anchor'};
function isGeo(n){
  var ports = defOf(n).ports, sel = FIX_SELECTOR[n.tag];
  return ports.some(function(p){ return p.name === 'lat'; }) && ports.some(function(p){ return p.name === 'lon'; })
    && (!sel || n.attrs[sel] === 'fix');
}
function geoList(){
  var t = curTree(), out = [];
  if(!t) return out;
  walk(t, function(n){
    if(isGeo(n) && isNum(n.attrs.lat) && isNum(n.attrs.lon))
      out.push({node:n, lat:Number(n.attrs.lat), lon:Number(n.attrs.lon), orbit: n.tag === 'CircleBuoy',
                r: isNum(n.attrs.radius) ? Number(n.attrs.radius) : 0});
  });
  return out;
}
function boatFix(){
  var b = (typeof S !== 'undefined' && S) ? S.boat : null;
  return (b && b.ok && b.lat != null && b.lon != null) ? b : null;
}
function ll2px(lat, lon){
  var k = Math.cos(MV.clat * Math.PI / 180);
  return [MV.W / 2 + (lon - MV.clon) * 111320 * k / MV.mpp, MV.H / 2 - (lat - MV.clat) * 110574 / MV.mpp];
}
function px2ll(x, y){
  var k = Math.cos(MV.clat * Math.PI / 180);
  return [MV.clat - (y - MV.H / 2) * MV.mpp / 110574, MV.clon + (x - MV.W / 2) * MV.mpp / (111320 * k)];
}
function mapCentre(lat, lon){ MV.clat = lat; MV.clon = lon; }
function mapBoat(){
  var b = boatFix();
  if(!b){ toast('no fresh boat position yet', true); return; }
  mapCentre(b.lat, b.lon); drawMap();
}
function mapFit(){
  var pts = geoList().map(function(g){ return [g.lat, g.lon]; }), b = boatFix();
  if(b) pts.push([b.lat, b.lon]);
  if(!pts.length) return;
  var la = pts.map(function(p){ return p[0]; }), lo = pts.map(function(p){ return p[1]; });
  var a0 = Math.min.apply(null, la), a1 = Math.max.apply(null, la), o0 = Math.min.apply(null, lo), o1 = Math.max.apply(null, lo);
  mapCentre((a0 + a1) / 2, (o0 + o1) / 2);
  var k = Math.cos(MV.clat * Math.PI / 180);
  var wm = (o1 - o0) * 111320 * k, hm = (a1 - a0) * 110574, W = MV.W || 300, H = MV.H || 240;
  MV.mpp = Math.max(0.05, Math.max(wm / (W * 0.7), hm / (H * 0.7), 0.1));
  drawMap();
}
function sizeMap(){
  var c = $('mmap'), w = $('mmapwrap'), dpr = window.devicePixelRatio || 1;
  var W = w.clientWidth, H = w.clientHeight;
  if(!W || !H) return false;
  if(MV.W !== W || MV.H !== H || MV.dpr !== dpr){
    MV.W = W; MV.H = H; MV.dpr = dpr; c.width = Math.round(W * dpr); c.height = Math.round(H * dpr);
  }
  return true;
}
function niceLen(m){
  var p = Math.pow(10, Math.floor(Math.log(m) / Math.LN10)), f = m / p;
  return (f >= 5 ? 5 : f >= 2 ? 2 : 1) * p;
}
function drawMap(){
  if(tab !== 'mission' || !M || !CAT || !sizeMap()) return;
  readPal();
  var c = $('mmap'), g = c.getContext('2d'), note = $('mmapnote'), b = boatFix(), pts = geoList();
  if(MV.clat == null){
    if(b) mapCentre(b.lat, b.lon); else if(pts.length) mapFit();
  }
  g.setTransform(MV.dpr, 0, 0, MV.dpr, 0, 0);
  g.fillStyle = mp.bg; g.fillRect(0, 0, MV.W, MV.H);
  if(MV.clat == null){
    note.textContent = 'No boat position and no points yet. Type a latitude and longitude into a GPS waypoint, '
      + 'or wait for a boat fix; the map has no tiles, so it needs one reference.';
    return;
  }
  note.textContent = b ? '' : 'no fresh boat position (not drawn)';
  g.font = '11px ui-monospace,monospace';
  var step = niceLen(MV.mpp * 90), k = Math.cos(MV.clat * Math.PI / 180);
  var tl = px2ll(0, 0), br = px2ll(MV.W, MV.H), x, y;
  g.strokeStyle = mp.grid; g.lineWidth = 1; g.beginPath();
  var e0 = Math.floor(((tl[1] - MV.clon) * 111320 * k) / step), e1 = Math.ceil(((br[1] - MV.clon) * 111320 * k) / step);
  var n0 = Math.floor(((br[0] - MV.clat) * 110574) / step), n1 = Math.ceil(((tl[0] - MV.clat) * 110574) / step);
  for(x = e0; x <= e1; x++){
    var gx = MV.W / 2 + x * step / MV.mpp; g.moveTo(gx, 0); g.lineTo(gx, MV.H); }
  for(y = n0; y <= n1; y++){
    var gy = MV.H / 2 - y * step / MV.mpp; g.moveTo(0, gy); g.lineTo(MV.W, gy); }
  g.stroke();
  /* known buoys, as the targets tab sees them */
  ((typeof S !== 'undefined' && S && S.targets && S.targets.items) || []).forEach(function(t){
    if(t.lat == null || t.lon == null) return;
    var p = ll2px(t.lat, t.lon);
    g.fillStyle = mp.lbl[t.label] || mp.lbl.none; g.beginPath(); g.arc(p[0], p[1], 5, 0, 6.2832); g.fill();
    g.fillStyle = mp.dim; g.fillText('#' + t.id + ' ' + (t.label || ''), p[0] + 8, p[1] + 4);
  });
  /* the mission's points, in execution order, joined */
  var sel = selId;
  g.strokeStyle = mp.accent; g.lineWidth = 2; g.setLineDash([6, 4]); g.beginPath();
  pts.forEach(function(q, i){ var p = ll2px(q.lat, q.lon); if(i) g.lineTo(p[0], p[1]); else g.moveTo(p[0], p[1]); });
  g.stroke(); g.setLineDash([]);
  pts.forEach(function(q, i){
    var p = ll2px(q.lat, q.lon), on = q.node.id === sel;
    if(q.orbit && q.r > 0){
      g.strokeStyle = mp.accent; g.lineWidth = 1; g.beginPath(); g.arc(p[0], p[1], q.r / MV.mpp, 0, 6.2832); g.stroke(); }
    g.fillStyle = on ? mp.fg : mp.accent; g.beginPath(); g.arc(p[0], p[1], on ? 11 : 9, 0, 6.2832); g.fill();
    g.fillStyle = on ? mp.bg : mp.bg; g.font = 'bold 11px ui-monospace,monospace';
    g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(String(i + 1) + (q.orbit ? 'o' : ''), p[0], p[1] + 0.5);
    g.textAlign = 'start'; g.textBaseline = 'alphabetic';
  });
  if(b){
    var bp = ll2px(b.lat, b.lon), hd = (b.heading || 0) * Math.PI / 180;
    g.save(); g.translate(bp[0], bp[1]); g.rotate(hd); g.fillStyle = mp.boat; g.strokeStyle = mp.bg; g.lineWidth = 1.5;
    g.beginPath(); g.moveTo(0, -12); g.lineTo(7, 9); g.lineTo(0, 4); g.lineTo(-7, 9); g.closePath(); g.fill(); g.stroke();
    g.restore();
  }
  var bar = step / MV.mpp; g.strokeStyle = mp.dim; g.fillStyle = mp.dim; g.lineWidth = 2; g.font = '11px ui-monospace,monospace';
  g.beginPath(); g.moveTo(10, MV.H - 14); g.lineTo(10 + bar, MV.H - 14); g.moveTo(10, MV.H - 18); g.lineTo(10, MV.H - 10);
  g.moveTo(10 + bar, MV.H - 18); g.lineTo(10 + bar, MV.H - 10); g.stroke();
  g.fillText(step + ' m', 16 + bar, MV.H - 10);
}
function mapAdd(lat, lon, forceAdd){
  var ll = [lat.toFixed(7), lon.toFixed(7)], e = selId && idx[selId], n = e && !e.isTree ? e.node : null;
  if(n && isGeo(n) && !forceAdd){ n.attrs.lat = ll[0]; n.attrs.lon = ll[1]; commit(); return; }
  var at = insertionPoint();
  if(!at){ toast('select a container block, or add a Sequence, to hold the new waypoint', true); return; }
  var preset = CAT.presets.filter(function(p){ return p.id === 'GpsWaypoint'; })[0];
  var node = preset ? makeNew({t:'preset', id:'GpsWaypoint'}) : mk('NavigateTo', {target:'fix'});
  node.attrs.lat = ll[0]; node.attrs.lon = ll[1];
  at.p.kids.splice(at.i, 0, node); selId = node.id; commit();
}
function mapPointer(){
  var c = $('mmap');
  c.addEventListener('pointerdown', function(ev){
    if(ev.button !== 0) return;
    MV.pan = {x:ev.clientX, y:ev.clientY, clat:MV.clat, clon:MV.clon, moved:false};
    c.setPointerCapture(ev.pointerId);
  });
  c.addEventListener('pointermove', function(ev){
    var p = MV.pan; if(!p || MV.clat == null) return;
    var dx = ev.clientX - p.x, dy = ev.clientY - p.y;
    if(!p.moved && Math.abs(dx) + Math.abs(dy) < 5) return;
    p.moved = true;
    var k = Math.cos(p.clat * Math.PI / 180);
    MV.clat = p.clat + dy * MV.mpp / 110574; MV.clon = p.clon - dx * MV.mpp / (111320 * k); drawMap();
  });
  c.addEventListener('pointerup', function(ev){
    var p = MV.pan; MV.pan = null; if(!p || p.moved) return;
    if(MV.clat == null){ toast('the map needs a reference first: no boat fix and no point with a lat/lon', true); return; }
    var r = c.getBoundingClientRect(), x = ev.clientX - r.left, y = ev.clientY - r.top, hit = null;
    geoList().forEach(function(q){ var s = ll2px(q.lat, q.lon); if(Math.hypot(s[0] - x, s[1] - y) < 12) hit = q; });
    if(hit){ select(hit.node.id); return; }
    var ll = px2ll(x, y);
    mapAdd(ll[0], ll[1], MV.mode === 'add' || ev.shiftKey);
  });
  c.addEventListener('wheel', function(ev){
    if(MV.clat == null) return;
    ev.preventDefault();
    var r = c.getBoundingClientRect(), x = ev.clientX - r.left, y = ev.clientY - r.top, before = px2ll(x, y);
    MV.mpp = Math.min(20, Math.max(0.02, MV.mpp * Math.pow(1.18, ev.deltaY > 0 ? 1 : -1)));
    var after = px2ll(x, y);
    MV.clat += before[0] - after[0]; MV.clon += before[1] - after[1]; drawMap();
  }, {passive:false});
}

/* ---------------- wiring ---------------- */
function bind(){
  $('mload').onclick = loadPicked; $('mnew').onclick = newMission;
  $('msave').onclick = function(){ saveMission(false); }; $('msaveas').onclick = function(){ saveMission(true); };
  $('mval').onclick = function(){ if(M) validateNow().then(function(){
    if(issues && issues.ok) toast('valid', false, 2000); else toast('the mission has errors (listed under the toolbar)', true); }); };
  $('mxml').onclick = showXml; $('mdown').onclick = downloadXml;
  $('mup').onclick = function(){ $('mfile').value = ''; $('mfile').click(); };
  $('mfile').onchange = function(){ uploadXml($('mfile').files[0]); };
  $('mstart').onclick = startDialog; $('mstop').onclick = stopRun;
  $('missues').addEventListener('click', function(ev){
    var d = ev.target.closest('.e,.w'); if(d) focusIssue(d.textContent.replace(/^\S+\s/, '')); });
  window.addEventListener('beforeunload', function(ev){
    if(M && cur.dirty){ ev.preventDefault(); ev.returnValue = ''; } });
  $('mq').oninput = function(){ query = this.value; renderPalette(); };
  $('madv').onchange = function(){ showAdv = this.checked; renderPalette(); };
  $('mdesc').onchange = function(){ if(M){ setDesc(this.value); commit({keepInsp:true}); } };
  $('mm_boat').onclick = mapBoat; $('mm_fit').onclick = mapFit;
  $('mm_mode').onclick = function(){
    MV.mode = MV.mode === 'auto' ? 'add' : 'auto'; this.textContent = 'click: ' + MV.mode; };
  $('mpalbody').addEventListener('pointerdown', function(ev){
    var it = ev.target.closest('.mpi'), ch = ev.target.closest('.ch');
    if(ch){ closed[ch.dataset.cat] = !closed[ch.dataset.cat]; renderPalette(); return; }
    if(!it || ev.button !== 0 || !M) return;
    ev.preventDefault();
    dragStart(ev, palMap[it.dataset.k], it.querySelector('.l').textContent, it.parentNode.getAttribute('style') || '');
  });
  $('mwork').addEventListener('pointerdown', function(ev){
    if(ev.button !== 0) return;
    var b = ev.target.closest('.mblk');
    if(!b){ settle(); select(''); return; }
    ev.preventDefault();
    dragStart(ev, {t:'move', id:b.dataset.id}, b.querySelector('.lb').textContent, b.getAttribute('style') || '');
  });
  $('mtabs').addEventListener('click', function(ev){
    var x = ev.target.closest('.x'), t = ev.target.closest('.mtab');
    if(x){ deleteTree(x.dataset.del); return; }
    if(!t) return;
    if(t.id === 'maddtree'){ addTree(); return; }
    treeId = t.dataset.tree; selId = ''; refresh({});
  });
  $('mtabs').addEventListener('dblclick', function(ev){
    var t = ev.target.closest('.mtab[data-tree]');
    if(t && t.dataset.tree !== mainId()) renameTree(t.dataset.tree); });
  $('minsp').addEventListener('input', function(ev){ inspInput(ev, false); });
  $('minsp').addEventListener('change', function(ev){ inspInput(ev, true); });
  $('minsp').addEventListener('click', inspClick);
  document.addEventListener('pointermove', dragMove);
  document.addEventListener('pointerup', function(ev){ dragEnd(ev, false); });
  document.addEventListener('pointercancel', function(ev){ dragEnd(ev, true); });
  document.addEventListener('keydown', function(ev){
    if(tab !== 'mission' || $('mdlg').style.display === 'flex') return;
    var t = ev.target, typing = t && (/^(INPUT|TEXTAREA|SELECT)$/.test(t.tagName) || t.isContentEditable);
    var k = ev.key.toLowerCase();
    if(ev.key === 'Escape' && drag){ dragEnd(ev, true); return; }
    if((ev.ctrlKey || ev.metaKey) && k === 's'){ ev.preventDefault(); saveMission(false); return; }
    if(typing) return;
    if((ev.ctrlKey || ev.metaKey) && k === 'z'){ ev.preventDefault(); if(ev.shiftKey) doRedo(); else doUndo(); }
    else if((ev.ctrlKey || ev.metaKey) && k === 'y'){ ev.preventDefault(); doRedo(); }
    else if((ev.ctrlKey || ev.metaKey) && k === 'd'){ ev.preventDefault(); if(selId) dupBlock(selId); }
    else if(ev.key === 'Delete' || ev.key === 'Backspace'){ if(selId){ ev.preventDefault(); removeBlock(selId); } }
    else if(ev.key === 'Escape'){ select(''); }
  });
  mapPointer();
  if(window.ResizeObserver) new ResizeObserver(function(){ drawMap(); }).observe($('mmapwrap'));
}
var booted = false;
function boot(){
  if(booted) return;
  booted = true; bind();
  api('/mission/catalogue', {}).then(function(j){
    if(j && j.ok !== false && j.nodes){ setCatalogue(j); CATMSG = ''; }
    else{ setCatalogue(FALLBACK_CAT); CATMSG = (j && j.message) || 'no block catalogue from the server'; }
    renderPalette();
    if(!M) try{ openXml(NEW_XML, '', ''); }catch(e){ toast('internal: ' + e.message, true); }
    else refresh({});
  });
}
window.misEnter = function(){
  boot(); pollRun(); refreshList();
  clearInterval(statTimer); statTimer = setInterval(pollRun, STAT_MS);
  setTimeout(function(){ drawMap(); }, 0);
};
window.misLeave = function(){ clearInterval(statTimer); statTimer = null; };
window.misTick = function(){ if(CAT && M) drawMap(); };
/* A seam for tests and for anyone driving the editor from the console. */
window.MIS = {
  state:function(){ return {M:M, cur:cur, treeId:treeId, selId:selId, idx:idx, issues:issues, run:run, undo:undo.length}; },
  open:openXml, xml:toXml, select:select, drop:applyDrop, mapAdd:mapAdd, mapClick:function(x, y, add){
    var ll = px2ll(x, y); mapAdd(ll[0], ll[1], !!add); }, undo:doUndo, redo:doRedo, remove:removeBlock,
  validate:validateNow, defOf:defOf, normNum:normNum, geo:geoList, view:function(){ return MV; }, ll2px:ll2px, setCat:setCatalogue
};
})();
"""
