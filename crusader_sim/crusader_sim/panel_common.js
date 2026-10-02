/* panel_common.js - what the two Task 1 pages (task1_panel.html: the sim; lake_panel.html: the REAL boat) share:
   the map (pan/zoom, grid, trail, the boat's arrow, the field's buoys), the boat's layers drawn from S.feed (costmap,
   hazards, tracks, fused passage, planned path), the layer row, the field table, the UAV error card's numbers, the log
   tabs and the 4 Hz poll. Plain script, no module, no libraries; the page loads it BEFORE its own inline script and
   defines what this file calls back for:

     editable()         may the field's positions be edited now (default: no)
     buoys()            [{x, y, ux, uy, staged, sent}] the field Fit and the map draw
     drawMap(boat,dpr)  paint one canvas of MAPS (using drawGrid, drawTrail, drawBuoys, drawBoatLayers, arrowAt)
     emptyClick(w,e), liveClick(w,e)   a click on empty water while editable / while the field is on the air
     render()           S (the /api/state JSON) -> the DOM
     onUnreachable()    the poll failed
     feedIdle(), feedWanted(), fitExtra(), isTruthMap()   optional, defaults below

   The page also sets MAPS=[{cv, boat}], calls useCanvas(MAPS[0].cv), bindMaps(), initPalette(), initLayers(defs, key),
   initLogs(keys), bindUav() and last startPolling(). Moved out of task1_panel.html, unchanged except for those hooks. */

var COL={flash_red:'#E5484D',flash_green:'#30A46C',flash_blue:'#3E8BFF',steady_blue:'#1D3FB8',off:'#111'};
var LBL={flash_red:'RED',flash_green:'GREEN',flash_blue:'ENTRY',steady_blue:'EXIT',off:'BLACK'};
var ORDER=['flash_red','flash_green','flash_blue','steady_blue','off'];
var S=null, fit=true, view={cx:25,cy:0,s:10}, trail=[], tgen=-1;
var pick='flash_red', selIdx=-1, layout=[], rev=-1, drag=null;   // the field being edited: the palette's role, the selected buoy, the layout, its server revision
var ORIGIN={label:'start',arrow:true};   // what the map's (0, 0) is: the sim's boat start, or the lake's datum
var FEED_IDLE_TEXT='sim is down';
var TABLE_LATLON=false;   // the lake page adds lat/lon columns to the buoy table
var UAV_TEXT={commit:'LAUNCH',locked:'a run is going, or the sim is starting/stopping'};
function editable(){return false}
function emptyClick(w,e){}   // a click on empty water while the field is editable
function liveClick(w,e){}    // ... and while it is on the air (the lake page's approach tool)
function isTruthMap(){return true}
function feedIdle(){return false}
function feedWanted(){return true}
function fitExtra(){return[]}
function onUnreachable(){}
function $(i){return document.getElementById(i)}
function val(i){return $(i).value}
function esc(s){return String(s).replace(/[&<>]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;'}[c]})}
function post(url,body){
  return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})})
    .then(function(r){return r.json()}).then(function(d){if(!d.ok)alert(d.error||'failed');return d})
    .catch(function(e){alert('panel unreachable: '+e)});
}
var cv, g;
function useCanvas(c){cv=c;g=c.getContext('2d')}
function W2S(x,y){return[cv.width/2+(x-view.cx)*view.s, cv.height/2-(y-view.cy)*view.s]}
function S2W(u,v){return[view.cx+(u-cv.width/2)/view.s, view.cy-(v-cv.height/2)/view.s]}
function autofit(){
  var pts=[[0,0]].concat(buoys().map(function(b){return[b.x,b.y]}),buoys().map(function(b){return[b.ux,b.uy]}),trail);
  if(S&&S.boat)pts.push([S.boat.x,S.boat.y]);
  var lp=layData('path'),lt=layData('tracks'),lf=layData('fused');
  if(lp){pts=pts.concat(lp.path);if(lp.goal)pts.push(lp.goal)}
  if(lt)lt.items.forEach(function(t){pts.push([t.x,t.y])});
  if(lf)lf.buoys.forEach(function(b){pts.push([b.x,b.y])});
  pts=pts.concat(fitExtra());
  var x0=1e9,x1=-1e9,y0=1e9,y1=-1e9;
  pts.forEach(function(p){x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1])});
  x0-=5;x1+=5;y0-=5;y1+=5; if(x1-x0<30){x1=x0+30}
  view.cx=(x0+x1)/2; view.cy=(y0+y1)/2; view.s=Math.min(cv.width/(x1-x0),cv.height/(y1-y0));
}
function draw(){
  var dpr=window.devicePixelRatio||1;
  MAPS.forEach(function(m){var r=m.cv.getBoundingClientRect(),w=Math.round(r.width*dpr),h=Math.round(r.height*dpr);
    if(m.cv.width!==w||m.cv.height!==h){m.cv.width=w;m.cv.height=h}});
  useCanvas(MAPS[0].cv); if(fit)autofit();       // both canvases are the same size, so one fit serves both
  MAPS.forEach(function(m){useCanvas(m.cv);drawMap(m.boat,dpr)});
}
function drawGrid(dpr){
  var a=S2W(0,cv.height), b=S2W(cv.width,0);
  g.lineWidth=1; g.font=(11*dpr)+'px monospace'; g.fillStyle='#4a6070';
  for(var x=Math.ceil(a[0]/5)*5;x<=b[0];x+=5){var p=W2S(x,0);g.strokeStyle=x===0?'#35556b':'#16293a';
    g.beginPath();g.moveTo(p[0],0);g.lineTo(p[0],cv.height);g.stroke();g.fillText(x+'m',p[0]+2,cv.height-4)}
  for(var y=Math.ceil(a[1]/5)*5;y<=b[1];y+=5){var q=W2S(0,y);g.strokeStyle=y===0?'#35556b':'#16293a';
    g.beginPath();g.moveTo(0,q[1]);g.lineTo(cv.width,q[1]);g.stroke();g.fillText(y+'m',3,q[1]-3)}
  var o=W2S(0,0); g.strokeStyle='#8aa';g.lineWidth=2*dpr;g.beginPath();g.arc(o[0],o[1],3*view.s,0,7);g.stroke();
  if(ORIGIN.arrow)arrowAt(0,0,0,'#8aa'); g.fillStyle='#8aa';g.fillText(ORIGIN.label,o[0]+6,o[1]+16*dpr);
}
function drawTrail(dpr){
  if(trail.length>1){g.strokeStyle='#E0A245';g.lineWidth=1.5*dpr;g.beginPath();
    trail.forEach(function(p,i){var s=W2S(p[0],p[1]);i?g.lineTo(s[0],s[1]):g.moveTo(s[0],s[1])});g.stroke()}
}
// the boat's layers, bottom to top (= LAYS order): the path is drawn over the costmap, the hazards, the tracks and the fused diamonds
function drawBoatLayers(dpr){drawCostmap(dpr);drawHazards(dpr);drawTracks(dpr);drawFused(dpr);drawPath(dpr)}
// ---- the boat's layers. layData() is the ONLY way a layer's data reaches a draw call: it is null unless the
// layer is on AND the server called it fresh, so a stale feed draws nothing (blanks over guesses)
function layData(k){var f=S&&S.feed?S.feed[LAYSRC[k]]:null;return layOn[k]&&f&&f.status==='fresh'?f.data:null}
function pathColour(st){return st==='FOLLOWING'?'#45BCA0':(st==='BLOCKED'||st==='FAILED')?'#E0736A':
  (st==='PLANNING'||st==='DEGRADED')?'#F2D14B':'#7C8FA0'}
function drawPath(dpr){                       // the ground station's conventions (gcs_page.py drawNav)
  var d=layData('path');layDrawn.path=0;if(!d)return;
  g.strokeStyle=pathColour(d.state);g.lineWidth=2*dpr;
  if(d.path.length>1){g.setLineDash([6*dpr,4*dpr]);g.beginPath();
    d.path.forEach(function(q,i){var s=W2S(q[0],q[1]);i?g.lineTo(s[0],s[1]):g.moveTo(s[0],s[1])});g.stroke();g.setLineDash([])}
  if(d.target){var t=W2S(d.target[0],d.target[1]);g.beginPath();g.arc(t[0],t[1],5*dpr,0,7);g.stroke()}   // the carrot
  if(d.goal){var q=W2S(d.goal[0],d.goal[1]),k=5*dpr;g.beginPath();                                       // the goal
    g.moveTo(q[0]-k,q[1]-k);g.lineTo(q[0]+k,q[1]+k);g.moveTo(q[0]-k,q[1]+k);g.lineTo(q[0]+k,q[1]-k);g.stroke()}
  layDrawn.path=d.path.length;
}
// the boat's own colour call, as bt_runner's beaconFromLabel makes it: red/green/blue substrings, flash = ENTRY, steady = EXIT
function labelState(l){l=(l||'').toLowerCase();
  if(l.indexOf('red')>=0)return 'flash_red';if(l.indexOf('green')>=0)return 'flash_green';
  if(l.indexOf('blue')>=0)return l.indexOf('flash')>=0?'flash_blue':(l.indexOf('steady')>=0||l.indexOf('solid')>=0)?'steady_blue':'blue';
  if(l.indexOf('black')>=0||l.indexOf('off')>=0)return 'off';return null}
function stateCol(s){return s==='off'?'#9ab':s==='blue'?'#3E8BFF':s&&COL[s]?COL[s]:'#7C8FA0'}
// A fused buoy this close to a track IS that track's: the report carries no track id, and where a track matched
// the tree puts the buoy at the track's position (to the feed's 0.01 m rounding), so anything beyond a metre is not it
var FUSE_MATCH_M=0.75;
function fusedStateAt(t,fz){var best=null,bd=FUSE_MATCH_M;   // the UAV's colour (a COL key) for this track, or null
  if(fz)fz.buoys.forEach(function(b){var d=Math.hypot(b.x-t.x,b.y-t.y);if(d<=bd&&COL[b.state]){bd=d;best=b.state}});
  return best}
function drawTracks(dpr){   // squares: what the boat BELIEVES, not the truth (solid circles)
  var d=layData('tracks'),fz=layData('fused');layDrawn.tracks=0;if(!d)return;
  d.items.forEach(function(t){
    var p=W2S(t.x,t.y),r=Math.max(7*dpr,0.5*view.s)+5*dpr,un=t.unseen==null?0:t.unseen,al=Math.max(0.3,1-un/30),
        vote=labelState(t.label),uav=fusedStateAt(t,fz),ring=vote?stateCol(vote):'#7C8FA0';   // vote null = unknown_buoy / lidar only: grey
    if(uav){g.globalAlpha=al*0.85;g.fillStyle=stateCol(uav);g.fillRect(p[0]-r+2*dpr,p[1]-r+2*dpr,2*r-4*dpr,2*r-4*dpr)}   // the UAV's colour
    g.globalAlpha=al;g.strokeStyle=ring;g.lineWidth=2*dpr;                                                                // the boat's own vote
    g.setLineDash(t.confirmed?[]:[4*dpr,3*dpr]);g.strokeRect(p[0]-r,p[1]-r,2*r,2*r);g.setLineDash([]);
    g.strokeStyle=uav?'#fff':ring;g.lineWidth=1*dpr;g.beginPath();g.moveTo(p[0]-3*dpr,p[1]);g.lineTo(p[0]+3*dpr,p[1]);   // the estimate itself
    g.moveTo(p[0],p[1]-3*dpr);g.lineTo(p[0],p[1]+3*dpr);g.stroke();
    g.fillStyle='#DFE8EF';
    if(!vote&&!uav){g.font='bold '+(14*dpr)+'px monospace';g.textAlign='center';g.fillText('?',p[0],p[1]-r*0.45);   // above the cross, inside the square
      g.textAlign='left';g.font=(11*dpr)+'px monospace'}
    g.textAlign='right';
    g.fillText('#'+t.id+' '+(uav?LBL[uav]+' (UAV) · cam ':'')+(t.label||'lidar only'),p[0]-r-3*dpr,p[1]+r+8*dpr);   // the colour the boat uses first
    if(un>2)g.fillText('seen '+un.toFixed(0)+' s ago',p[0]-r-3*dpr,p[1]+r+8*dpr+12*dpr);
    g.textAlign='left';g.globalAlpha=1;layDrawn.tracks++});
}
function drawFused(dpr){                      // a diamond in the UAV's colour, at the tracker's position where a track matched (else the UAV's)
  var d=layData('fused');layDrawn.fused=0;if(!d)return;
  d.buoys.forEach(function(b){var p=W2S(b.x,b.y),k=5*dpr;
    g.fillStyle=stateCol(b.state);g.strokeStyle='#DFE8EF';g.lineWidth=1*dpr;g.beginPath();
    g.moveTo(p[0],p[1]-k);g.lineTo(p[0]+k,p[1]);g.lineTo(p[0],p[1]+k);g.lineTo(p[0]-k,p[1]);g.closePath();g.fill();g.stroke();
    layDrawn.fused++});
}
// ---- the hazards: the boat's known field (/crsd/nav/hazards), as the planner's costmap is built from it. A circle is drawn at
// its PHYSICAL radius (solid) with the keep-out ring the layer adds (dashed); a polygon (a dock) is a faint fill. Only when the
// layer is on, fresh and the page lists it (the sim page does not): layData() is null otherwise.
function drawHazards(dpr){
  var d=layData('hazards');layDrawn.hazards=0;if(!d)return;
  g.lineWidth=1.5*dpr;
  d.items.forEach(function(h){
    if(h.kind===0){var p=W2S(h.x,h.y),r=Math.max(h.r*view.s,3*dpr),k=(h.keepout||0)*view.s;
      g.strokeStyle='rgba(224,115,106,.85)';g.fillStyle='rgba(224,115,106,.18)';
      g.beginPath();g.arc(p[0],p[1],r,0,7);g.fill();g.stroke();
      if(k>1*dpr){g.setLineDash([3*dpr,3*dpr]);g.beginPath();g.arc(p[0],p[1],r+k,0,7);g.stroke();g.setLineDash([])}}
    else if(h.pts&&h.pts.length>2){g.strokeStyle='rgba(224,115,106,.85)';g.fillStyle='rgba(224,115,106,.18)';g.beginPath();
      h.pts.forEach(function(q,i){var s=W2S(q[0],q[1]);i?g.lineTo(s[0],s[1]):g.moveTo(s[0],s[1])});g.closePath();g.fill();g.stroke()}
    layDrawn.hazards++});
}
// ---- the costmap. panel_feed gives each position as one track's x/y are given: [x, y] course metres (a path point's
// form); {x, y} is read too. Anything else is skipped, never guessed. No "costmap" key in the feed (a panel_feed that
// predates the layer) = layData() is null = nothing is drawn and nothing throws.
var COSTMAP_COL={cells:'rgba(182,110,230,.42)',lidar:'rgba(78,200,230,.65)'};   // violet (not red: RED buoys, BLOCKED paths) and cyan
function isNum(v){return typeof v==='number'&&isFinite(v)}
function nItems(list){return Array.isArray(list)?list.length:0}
function eachPos(list,fn){
  for(var i=0;i<nItems(list);i++){var p=list[i],a=Array.isArray(p),x=a?p[0]:p&&p.x,y=a?p[1]:p&&p.y;if(isNum(x)&&isNum(y))fn(x,y)}
}
function fillCells(list,side,col){          // translucent squares, centred on each position; the ones off the canvas are skipped
  var h=side/2;g.fillStyle=col;
  eachPos(list,function(x,y){var p=W2S(x,y);
    if(p[0]>-side&&p[1]>-side&&p[0]<cv.width+side&&p[1]<cv.height+side)g.fillRect(p[0]-h,p[1]-h,side,side)});
}
function drawCostmap(dpr){
  var d=layData('costmap');layDrawn.costmap=0;if(!d)return;
  var side=Math.max((isNum(d.res_m)&&d.res_m>0?d.res_m:0)*view.s,1*dpr);   // res_m metres wide, but never under 1 px
  fillCells(d.cells,side,COSTMAP_COL.cells);fillCells(d.lidar,side,COSTMAP_COL.lidar);   // LiDAR voxels over the obstacle cells
  layDrawn.costmap=nItems(d.cells)+nItems(d.lidar);
}
// ---- the layer row under the boat's map: a toggle and one honest line of status per layer, and the NAV badge.
// The page lists its layers: [{k, src (the S.feed key), t (button text), tip}], bottom to top, and calls initLayers()
// once its #layers element exists. Each toggle is remembered in localStorage under `key`.
var LAYS=[], LAYSRC={}, LAY_KEY='', layOn={}, layDrawn={};
function initLayers(defs,key){
  LAYS=defs;LAY_KEY=key;
  defs.forEach(function(d){LAYSRC[d.k]=d.src;layOn[d.k]=true;layDrawn[d.k]=0});
  try{var _lo=JSON.parse(localStorage.getItem(LAY_KEY)||'{}');defs.forEach(function(d){if(_lo[d.k]===false)layOn[d.k]=false})}catch(e){}
  defs.forEach(function(d){
    var w=document.createElement('span');w.className='lay';
    w.innerHTML='<button class="tog" id="lay_'+d.k+'" title="'+esc(d.tip)+'"></button><span class="sub" id="laySt_'+d.k+'"></span>';
    $('layers').appendChild(w);
    $('lay_'+d.k).onclick=function(){layOn[d.k]=!layOn[d.k];laySave();layPaint();draw()};
  });
}
function laySave(){try{localStorage.setItem(LAY_KEY,JSON.stringify(layOn))}catch(e){}}
function layText(k){
  var F=S&&S.feed;if(!F)return 'this panel server has no feed: restart the panel';
  if(F.error)return F.error;
  var f=F[LAYSRC[k]],dn=feedIdle();
  if(!f)return k==='costmap'?"this panel_feed does not send a costmap (a rig that predates the layer: restart it)":'no such feed layer';
  if(f.status==='offline')return dn?FEED_IDLE_TEXT:F.packets?'panel_feed silent '+fmtAge(F.silent_s)+
    (f.age!=null?' (its last data is '+fmtAge(f.age)+' old, not drawn)':''):'panel_feed not heard (no rig up, or one that predates panel_feed)';
  if(f.status==='none')return k==='fused'?'not published yet: the tree sends it only while a run is going':
    k==='costmap'?'not published yet: is Nav2 up (NAV_MODE)?':'not published yet';
  if(f.status==='stale')return 'STALE '+fmtAge(f.age)+', not drawn';
  var d=f.data,age=' \u00b7 '+fmtAge(f.age);
  if(k==='costmap')return nItems(d.cells)+' cells \u00b7 '+nItems(d.lidar)+' LiDAR \u00b7 '+
    (isNum(d.res_m)?d.res_m+' m':'no res_m')+age;
  if(k==='path')return (d.state||'?')+(d.path.length?' \u00b7 '+d.path.length+' pts':' \u00b7 no path')+(d.why?' \u00b7 '+d.why:'')+age;
  if(k==='tracks')return d.items.length+(d.items.length===1?' track':' tracks')+' ('+d.items.filter(function(t){return t.confirmed}).length+' confirmed)'+
    (d.skipped?' \u00b7 '+d.skipped+' without a position':'')+age;
  if(k==='hazards')return d.items.length+(d.items.length===1?' hazard':' hazards')+(d.n>d.items.length?' of '+d.n:'')+age;
  return d.buoys.length+(d.buoys.length===1?' buoy':' buoys')+(d.skipped?' \u00b7 '+d.skipped+' without a position':'')+age;
}
function layPaint(){
  LAYS.forEach(function(d){var b=$('lay_'+d.k);b.textContent=d.t+': '+(layOn[d.k]?'on':'off');b.className='tog'+(layOn[d.k]?' on':'');
    var st=$('laySt_'+d.k),f=S&&S.feed?S.feed[LAYSRC[d.k]]:null;st.textContent=S?layText(d.k)+(layOn[d.k]?'':' (hidden)'):'';
    st.className='sub'+(f&&(f.status==='stale'||f.status==='offline')&&feedWanted()?' w':'')});
  var b=$('navbadge'),F=S&&S.feed,f=F&&F.leg;
  if(!layOn.path||!f||!feedWanted()){b.style.display='none';b.textContent='';return}
  b.style.display='';
  b.title='bt_runner publishes the leg at up to 2 Hz while a leg runs, and once when it goes IDLE: silence means no leg is running, or no bt_runner';
  if(f.status==='fresh'){var d=f.data;b.style.color=pathColour(d.state);
    b.textContent='NAV '+(d.state||'?')+(d.blocked_s>0?' '+d.blocked_s.toFixed(1)+'s':'')+(d.why?' '+d.why:'')+(d.hops>0&&d.hop!=null?' hop '+d.hop+'/'+d.hops:'')}
  else{b.style.color='#7C8FA0';
    b.textContent='NAV '+(f.status==='stale'?'stale '+fmtAge(f.age):f.status==='none'?'no data':'feed offline')}
}
function drawUav(dpr){   // what the UAV says: hollow, joined to the truth by a thin line
  buoys().forEach(function(bu){
    if(Math.hypot(bu.ux-bu.x,bu.uy-bu.y)<0.05)return;
    var p=W2S(bu.x,bu.y), q=W2S(bu.ux,bu.uy), rad=Math.max(7*dpr,0.5*view.s), jr=S&&S.uav?S.uav.jitter_m*view.s:0;
    g.strokeStyle='#7C8FA0';g.lineWidth=1*dpr;g.beginPath();g.moveTo(p[0],p[1]);g.lineTo(q[0],q[1]);g.stroke();
    g.strokeStyle=bu.staged==='off'?'#9ab':COL[bu.staged];g.lineWidth=2*dpr;g.beginPath();g.arc(q[0],q[1],rad,0,7);g.stroke();
    if(jr>2*dpr){g.setLineDash([3*dpr,3*dpr]);g.lineWidth=1*dpr;g.beginPath();g.arc(q[0],q[1],jr,0,7);g.stroke();g.setLineDash([])}});
}
function drawBuoys(dpr){   // the true buoys, solid, with the staged colour change and the ENTRY/EXIT direction
  buoys().forEach(function(bu,i){
    var p=W2S(bu.x,bu.y), rad=Math.max(7*dpr,0.5*view.s);
    g.fillStyle=COL[bu.staged]; g.beginPath(); g.arc(p[0],p[1],rad,0,7); g.fill();
    g.lineWidth=(i===selIdx?3:1.5)*dpr; g.strokeStyle=i===selIdx?'#fff':'#9ab'; g.stroke();
    if(bu.staged!==bu.sent){g.setLineDash([4*dpr,3*dpr]);g.strokeStyle='#E0A245';g.lineWidth=2*dpr;
      g.beginPath();g.arc(p[0],p[1],rad+5*dpr,0,7);g.stroke();g.setLineDash([])}
    if(bu.staged==='flash_blue'||bu.staged==='steady_blue'){ // cw for ENTRY, ccw for EXIT
      var cw=bu.staged==='flash_blue'; g.strokeStyle=COL[bu.staged]; g.lineWidth=2*dpr; g.beginPath();
      g.arc(p[0],p[1],rad+11*dpr,0,4.7,!cw); g.stroke()}
    g.fillStyle='#DFE8EF'; g.fillText('b'+i+(bu.staged==='flash_blue'?' ENTRY':bu.staged==='steady_blue'?' EXIT':''),p[0]+rad+4,p[1]-rad);
    if(bu.staged!==bu.sent){g.fillStyle='#E0A245';g.fillText('was '+LBL[bu.sent],p[0]+rad+4,p[1]+rad+8*dpr)}
  });
}
function hit(u,v){var bs=buoys(),dpr=window.devicePixelRatio||1;if(!isTruthMap())return -1;   // the boat's map has no buoys to grab
  for(var i=0;i<bs.length;i++){var p=W2S(bs[i].x,bs[i].y);if(Math.hypot(p[0]-u,p[1]-v)<Math.max(10*dpr,0.6*view.s))return i}return -1}
function bindMaps(){MAPS.forEach(function(m){var c=m.cv;
  c.onmousedown=onDown;c.onmousemove=onMove;c.onmouseup=onUp;c.onwheel=onWheel;
  c.onmouseleave=function(e){useCanvas(e.currentTarget);   // a drag never crosses to the other map
    if(drag&&drag.moved&&drag.i>=0&&editable())pushLayout();drag=null}})}
function onDown(e){var m=ev(e),i=hit(m[0],m[1]);$('pop').style.display='none';
  drag={i:i,u:m[0],v:m[1],moved:false,cx:view.cx,cy:view.cy}; if(i>=0&&editable()){selIdx=i;draw()}}
function onMove(e){var m=ev(e),w=S2W(m[0],m[1]);$('cursor').textContent='x '+w[0].toFixed(1)+'  y '+w[1].toFixed(1);
  if(!drag)return; if(Math.hypot(m[0]-drag.u,m[1]-drag.v)>4)drag.moved=true; if(!drag.moved)return;
  if(drag.i>=0&&editable()){layout[drag.i].x=Math.round(w[0]*10)/10;layout[drag.i].y=Math.round(w[1]*10)/10}
  else{fit=false;view.cx=drag.cx-(m[0]-drag.u)/view.s;view.cy=drag.cy+(m[1]-drag.v)/view.s} draw()}
function onUp(e){var d=drag;drag=null;if(!d)return;var m=ev(e),w=S2W(m[0],m[1]);
  if(!isTruthMap())return;
  if(editable()){
    if(d.i>=0&&d.moved)pushLayout();
    else if(d.i<0&&!d.moved)emptyClick(w,e);
  } else if(S&&S.run&&!d.moved){if(d.i>=0)showPop(d.i,e.clientX,e.clientY);else liveClick(w,e)}}
function arrowAt(x,y,yaw,c){var p=W2S(x,y),k=Math.max(10,1.0*view.s);g.save();g.translate(p[0],p[1]);g.rotate(-yaw);
  g.fillStyle=c;g.beginPath();g.moveTo(k,0);g.lineTo(-k*.6,k*.45);g.lineTo(-k*.6,-k*.45);g.closePath();g.fill();g.restore()}
function ev(e){useCanvas(e.currentTarget);var r=cv.getBoundingClientRect(),d=cv.width/r.width;return[(e.clientX-r.left)*d,(e.clientY-r.top)*d]}
function onWheel(e){e.preventDefault();var m=ev(e),w=S2W(m[0],m[1]),k=e.deltaY<0?1.15:1/1.15;fit=false;
  view.s*=k;view.cx=w[0]-(m[0]-cv.width/2)/view.s;view.cy=w[1]+(m[1]-cv.height/2)/view.s;draw()}
document.onkeydown=function(e){if(e.target.tagName==='INPUT'||e.target.tagName==='SELECT')return;
  if(e.key>='1'&&e.key<='5'){pick=ORDER[+e.key-1];paint()}
  if((e.key==='Delete'||e.key==='Backspace')&&selIdx>=0&&editable()){layout.splice(selIdx,1);selIdx=-1;pushLayout()}};
function showPop(i,x,y){var p=$('pop');p.innerHTML='<b>b'+i+'</b>'+ORDER.map(function(s){
  return '<button onclick="stage('+i+',\''+s+'\')"><span class="sw" style="background:'+COL[s]+'"></span>'+LBL[s]+
    (s===S.run.buoys[i].sent?' (sent)':'')+'</button>'}).join('');
  p.style.left=x+8+'px';p.style.top=y+8+'px';p.style.display='block'}
function stage(i,s){$('pop').style.display='none';post('/api/stage',{id:i,state:s}).then(poll1)}
function uavPost(k,el){var v=parseFloat(el.value);if(!isFinite(v)){alert(k+': not a number');return}
  var b={};b[k]=k==='seed'?Math.round(v):v;post('/api/uav_error',b).then(poll1)}
function bindUav(){
  $('uR').onchange=function(){uavPost('radius_m',this)};
  $('uJit').onchange=function(){uavPost('jitter_m',this)};
  $('uSeed').onchange=function(){uavPost('seed',this)};
}
function pushLayout(){post('/api/layout',{buoys:layout}).then(function(d){if(d&&d.ok)rev=d.rev;poll1()});renderTable();draw()}
// ---- setup table
function renderTable(){
  if(document.activeElement&&$('btable').contains(document.activeElement))return;
  $('btable').innerHTML=layout.map(function(b,i){
    return '<tr'+(i===selIdx?' style="background:#ffffff10"':'')+'><td>b'+i+'</td>'+
      '<td><input size="5" value="'+b.x+'" onchange="setXY('+i+',\'x\',this.value)"></td>'+
      '<td><input size="5" value="'+b.y+'" onchange="setXY('+i+',\'y\',this.value)"></td>'+latlonCells(i)+
      '<td><select onchange="layout['+i+'].state=this.value;pushLayout()">'+ORDER.map(function(s){
        return '<option value="'+s+'"'+(s===b.state?' selected':'')+'>'+LBL[s]+'</option>'}).join('')+'</select></td>'+
      '<td><button onclick="layout.splice('+i+',1);selIdx=-1;pushLayout()">✕</button></td></tr>'}).join('');
}
function latlonCells(i){   // the lake page's two read-only columns
  var b=TABLE_LATLON&&S&&S.layout.buoys[i];
  return b&&b.lat!=null?'<td class="sub">'+b.lat.toFixed(7)+'</td><td class="sub">'+b.lon.toFixed(7)+'</td>':TABLE_LATLON?'<td></td><td></td>':''}
function setXY(i,k,v){var n=parseFloat(v);if(!isFinite(n)){alert('not a number');return}layout[i][k]=n;pushLayout()}
function fillSelect(id,names,pref){var el=$(id),cur=el.value;if(el.dataset.k===names.join())return;el.dataset.k=names.join();
  el.innerHTML=names.map(function(n){return'<option>'+esc(n)+'</option>'}).join('');
  if(names.indexOf(cur)>=0)el.value=cur;else if(names.indexOf(pref)>=0)el.value=pref}
function renderUavFields(){   // the UAV position-error card's numbers (the page places the card itself)
  var U=S.uav;
  [['uR',U.radius_m],['uJit',U.jitter_m],['uSeed',U.seed]].forEach(function(f){
    var el=$(f[0]);el.disabled=U.locked;if(document.activeElement!==el)el.value=f[1]});
  $('uReroll').disabled=U.locked;
  $('uWarn').className='sub'+(U.jitter_m>0?' w':'');
  var m=U.offsets.map(function(o){return Math.hypot(o[0],o[1])});
  $('uInfo').textContent=(U.live?'in use: ':'will draw at '+UAV_TEXT.commit+': ')+(m.length?'offsets '+m.map(function(v,i){return'b'+i+' '+v.toFixed(1)}).join('  ')+' m  (max '+Math.max.apply(null,m).toFixed(2)+' m)':'no buoys');
  $('uLock').textContent=U.locked?'locked: '+UAV_TEXT.locked:'';
}
function paint(){ORDER.forEach(function(s){$('p_'+s).className=s===pick?'sel':''})}
function initPalette(){
ORDER.forEach(function(s,i){
  var b=document.createElement('button'); b.id='p_'+s;
  b.innerHTML='<span class="sw" style="background:'+COL[s]+'"></span>'+(i+1)+' '+LBL[s];
  b.onclick=function(){pick=s;paint()}; $('palette').appendChild(b);
});
paint();
}
// ---- logs: one tab per buffer the page names; the server sends lines after the cursor in `seq`
var logs={}, seq={}, tab='';
function initLogs(keys){
  keys.forEach(function(k){logs[k]=[];seq[k]=0;
    var b=document.createElement('button'); b.textContent=k; b.id='t_'+k;
    b.onclick=function(){tab=k;showLog()}; $('tabs').appendChild(b)});
  tab=keys[0];
}
function showLog(){Object.keys(logs).forEach(function(k){$('t_'+k).className=k===tab?'sel':''});
  var el=$('logview'),atEnd=el.scrollTop+el.clientHeight>=el.scrollHeight-20;el.textContent=logs[tab].join('\n');if(atEnd)el.scrollTop=el.scrollHeight}
// ---- poll
function fmtAge(t){return t==null?'—':t.toFixed(1)+' s'}
function poll1(){
  var q='log='+Object.keys(seq).map(function(k){return k+':'+seq[k]}).join(',')+'&trail='+tgen+':'+trail.length;
  return fetch('/api/state?'+q).then(function(r){return r.json()}).then(function(d){
    S=d; Object.keys(d.logs).forEach(function(k){if(!logs[k])return;var L=d.logs[k];if(L.seq<seq[k])logs[k]=[];
      logs[k]=logs[k].concat(L.lines).slice(-3000);seq[k]=L.seq});
    if(d.trail.gen!==tgen||d.trail.from===0){trail=[];tgen=d.trail.gen} trail=trail.concat(d.trail.pts);
    render(); showLog();
  }).catch(function(){onUnreachable()});
}
function loop(){poll1().then(function(){setTimeout(loop,250)},function(){setTimeout(loop,1000)})}
// ---- what both pages render the same way from S
function renderValid(){   // the layout's errors, warnings and note under the buoy table
  $('valid').innerHTML=S.layout.errors.map(function(e){return'<div class="e">✗ '+esc(e)+'</div>'}).join('')+
    S.layout.warnings.map(function(e){return'<div class="w">! '+esc(e)+'</div>'}).join('')+(S.layout.note?'<div class="sub">'+esc(S.layout.note)+'</div>':'');
}
function renderRun(){     // with a field on the air: the unsent count, auto-ACK, the radio line, the boat's ask, the checkpoint table
  var run=S.run;
  if(run){$('unsent').textContent=run.unsent+' unsent change'+(run.unsent===1?'':'s');$('send').disabled=!run.unsent||!!run.problem;$('problem').textContent=run.problem||''}
  $('auto').checked=S.radio.auto_ack;
  $('radio').textContent=S.radio.up?('fields sent '+S.radio.sent+', resends '+S.radio.resends+', last '+fmtAge(S.radio.last_tx_age)):'radio down';
  var c=S.checkpoint; $('ask').style.display=c?'':'none';
  if(c){$('askText').textContent='Boat asks: checkpoint '+c.seq+' — '+c.label+' (waiting '+c.waiting_s.toFixed(0)+' s, re-asked '+(c.asks-1)+(c.asks===2?' time)':' times)');
    $('askWhat').textContent=c.what||'';$('askSend').disabled=!run||!run.unsent||!!run.problem}
  $('cps').innerHTML=S.checkpoints.map(function(r){var t=new Date(r.asked*1000).toLocaleTimeString();
    return'<tr><td>'+r.seq+'</td><td>'+esc(r.label)+'</td><td>'+t+'</td><td>'+(r.reply?esc(r.reply)+' +'+(r.answered-r.asked).toFixed(0)+'s':'<b class="w">waiting</b>')+
      '</td><td>'+r.asks+'</td></tr>'}).join('');
}
function startPolling(){window.onresize=draw; loop()}
