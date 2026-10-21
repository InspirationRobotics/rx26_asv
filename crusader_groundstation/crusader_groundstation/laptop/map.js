// map.js: the ground station's one map. One state, drawn into every canvas that shows it
// (the Map tab and the Mission tab), in metres east/north of the first position heard.
// The MANUAL field editor works on the same state: what you place is what the boat is sent.
// Clicking the map and addManual/removeManual/setManual edit one list; onManualChange hears them all.
"use strict";
const CrsdMap = (() => {
  const M_PER_DEG = 111320;
  const FONT = "12px system-ui, sans-serif", FONT_BOLD = "bold 12px system-ui, sans-serif";
  const BEACON = {
    0: ["unknown", "#9e9e9e"], 1: ["off", "#424242"], 2: ["flashing red", "#e53935"],
    3: ["flashing green", "#43a047"], 4: ["flashing blue: ENTRY", "#1e88e5"],
    5: ["steady blue: EXIT", "#0d47a1"],
  };
  const FLASHING = new Set([2, 3, 4]);
  // theme key: [CSS variables tried in order, fallback]; read at every draw so a theme switch shows
  const THEME = {
    bg: [["--map-bg"], "#eef2f6"],
    grid: [["--map-grid", "--grid"], "rgba(120,135,155,0.25)"],
    major: [["--map-grid-major"], "rgba(120,135,155,0.55)"],
    text: [["--map-text", "--fg"], "#1d2330"],
    boat: [["--map-boat", "--accent"], "#1565c0"],
    path: [["--map-path"], "#00bcd4"],
    hazard: [["--map-hazard"], "#ff9800"],
    obstacle: [["--map-obstacle"], "#7b1fa2"],
  };
  const OBSTACLE_MIN_PX = 3;                      // radius floor: a small cluster survives zooming out
  const S = {
    origin: null, boat: null, trail: [], targets: [], plan: null, path: null, hazards: [],
    obstacles: [], manual: [], editing: false, beacon: 2,
  };
  const views = [], listeners = [];
  let T = null;                                   // the colours of the draw in progress

  function xy(lat, lon) {
    if (!S.origin) S.origin = [lat, lon];
    const k = Math.cos(S.origin[0] * Math.PI / 180);
    return [(lon - S.origin[1]) * M_PER_DEG * k, (lat - S.origin[0]) * M_PER_DEG];
  }
  function latlon(x, y) {
    const k = Math.cos(S.origin[0] * Math.PI / 180);
    return [S.origin[0] + y / M_PER_DEG, S.origin[1] + x / (M_PER_DEG * k)];
  }
  function theme() {
    const cs = getComputedStyle(document.documentElement), t = {};
    for (const [key, [names, fallback]] of Object.entries(THEME))
      t[key] = names.map(n => cs.getPropertyValue(n).trim()).find(Boolean) || fallback;
    return t;
  }
  function labelColour(name) {
    const l = (name || "").toLowerCase();
    for (const [w, c] of [["red", "#e53935"], ["green", "#43a047"], ["blue", "#1e88e5"],
                          ["yellow", "#fdd835"], ["white", "#fafafa"], ["black", "#212121"],
                          ["off", "#616161"]]) if (l.includes(w)) return c;
    return T.text;
  }

  // ---- state in -------------------------------------------------------------
  function update(kind, f) {
    if (kind === "state") {
      const b = f.boat || {};
      S.boat = b.lat == null ? (S.boat ? Object.assign({}, S.boat, {stale: true}) : null)
                             : {lat: b.lat, lon: b.lon, hdg: b.hdg, stale: false};
      if (b.lat != null) {
        const p = xy(b.lat, b.lon), last = S.trail[S.trail.length - 1];
        if (!last || Math.hypot(p[0] - last[0], p[1] - last[1]) > 0.5) {
          S.trail.push(p);
          if (S.trail.length > 3000) S.trail.shift();
        }
      }
    } else if (kind === "targets") S.targets = f.items || [];
    else if (kind === "plan") S.plan = f.ok ? f : null;
    else if (kind === "path") S.path = f.ok ? f : null;
    else if (kind === "hazards") S.hazards = f.items || [];
    else if (kind === "obstacles") S.obstacles = f.items || [];   // the LiDAR clusters; [] clears
    redraw();
  }

  // ---- views ----------------------------------------------------------------
  // Pointer maths is in CSS pixels (v.w by v.h); the backing store is scaled by devicePixelRatio.
  function attach(cv, opts) {
    const v = {cv, ctx: cv.getContext("2d"), w: 0, h: 0, scale: 4, centre: [0, 0], follow: true,
               drag: null, mouse: null, queued: false, editable: !!(opts && opts.editable)};
    views.push(v);
    cv.addEventListener("wheel", e => {
      e.preventDefault();
      const before = toWorld(v, e.offsetX, e.offsetY);
      v.scale = Math.max(0.2, Math.min(200, v.scale * (e.deltaY < 0 ? 1.25 : 0.8)));
      const after = toWorld(v, e.offsetX, e.offsetY);
      v.centre = [v.centre[0] + before[0] - after[0], v.centre[1] + before[1] - after[1]];
      v.follow = false;
      invalidate(v);
    }, {passive: false});
    cv.addEventListener("mousedown", e => { v.drag = {x: e.offsetX, y: e.offsetY, moved: false}; });
    cv.addEventListener("mousemove", e => {
      v.mouse = [e.offsetX, e.offsetY];
      if (v.drag) {
        const dx = e.offsetX - v.drag.x, dy = e.offsetY - v.drag.y;
        if (Math.abs(dx) + Math.abs(dy) > 3) v.drag.moved = true;
        if (v.drag.moved) {
          v.centre = [v.centre[0] - dx / v.scale, v.centre[1] + dy / v.scale];
          v.drag.x = e.offsetX; v.drag.y = e.offsetY; v.follow = false;
        }
      }
      invalidate(v);
    });
    cv.addEventListener("mouseup", e => {
      const click = v.drag && !v.drag.moved;
      v.drag = null;
      if (click && v.editable && S.editing && S.origin) edit(v, e.offsetX, e.offsetY);
    });
    cv.addEventListener("mouseleave", () => { v.drag = null; v.mouse = null; invalidate(v); });
    cv.addEventListener("dblclick", () => { v.follow = true; draw(v); });
    new ResizeObserver(() => draw(v)).observe(cv);
    return v;
  }
  function toWorld(v, px, py) {
    return [v.centre[0] + (px - v.w / 2) / v.scale, v.centre[1] - (py - v.h / 2) / v.scale];
  }
  function toScreen(v, p) {
    return [v.w / 2 + (p[0] - v.centre[0]) * v.scale, v.h / 2 - (p[1] - v.centre[1]) * v.scale];
  }
  function at(v, lat, lon) { return toScreen(v, xy(lat, lon)); }
  function trace(v, pts, geo) {                   // a polyline into the current path; geo: [lat, lon] points
    pts.forEach((q, i) => {
      const s = geo ? at(v, q[0], q[1]) : toScreen(v, q);
      i ? v.ctx.lineTo(s[0], s[1]) : v.ctx.moveTo(s[0], s[1]);
    });
  }
  function redraw() { views.forEach(draw); }
  function invalidate(v) {                        // pan, zoom and hover come faster than frames
    if (v.queued) return;
    v.queued = true;
    requestAnimationFrame(() => { v.queued = false; draw(v); });
  }
  function fit(v) {
    const cv = v.cv, dpr = window.devicePixelRatio || 1;
    v.w = cv.clientWidth; v.h = cv.clientHeight;
    const bw = Math.round(v.w * dpr), bh = Math.round(v.h * dpr);
    if (cv.width !== bw || cv.height !== bh) { cv.width = bw; cv.height = bh; }
    v.ctx.setTransform(bw / v.w, 0, 0, bh / v.h, 0, 0);
  }

  // ---- drawing --------------------------------------------------------------
  function draw(v) {
    const cv = v.cv, c = v.ctx;
    if (!cv.offsetParent) return;                 // a hidden tab draws nothing
    fit(v);
    T = theme();
    c.fillStyle = T.bg; c.fillRect(0, 0, v.w, v.h);
    c.font = FONT; c.lineJoin = "round";
    if (!S.origin) {
      label(c, "no position yet", v.w / 2, v.h / 2, "center", "middle");
      return;
    }
    if (v.follow && S.boat) v.centre = xy(S.boat.lat, S.boat.lon);
    grid(v);
    // hazards: what the planner keeps out of
    c.strokeStyle = T.hazard; c.lineWidth = 1;
    for (const h of S.hazards) {
      c.beginPath();
      if (h.c) {
        const p = at(v, h.c[0], h.c[1]);
        c.arc(p[0], p[1], Math.max(2, ((h.r || 0) + (h.keep || 0)) * v.scale), 0, 2 * Math.PI);
      } else if (h.poly) {
        trace(v, h.poly, true);
        c.closePath();
      }
      c.stroke();
    }
    // LiDAR clusters: translucent, under the trail and the targets they may turn out to be
    c.fillStyle = c.strokeStyle = T.obstacle; c.lineWidth = 1;
    for (const o of S.obstacles) {
      const p = at(v, o.lat, o.lon), r = Math.max(OBSTACLE_MIN_PX, (o.r || 0) * v.scale);
      c.beginPath(); c.arc(p[0], p[1], r, 0, 2 * Math.PI);
      c.globalAlpha = 0.35; c.fill();
      c.globalAlpha = 0.9; c.stroke();
    }
    c.globalAlpha = 1;
    // trail
    c.strokeStyle = T.boat; c.lineWidth = 1.5; c.globalAlpha = 0.6; c.beginPath();
    trace(v, S.trail);
    c.stroke(); c.globalAlpha = 1;
    // the planned leg
    if (S.path) {
      c.strokeStyle = T.path; c.lineWidth = 2; c.beginPath();
      trace(v, S.path.path || [], true);
      c.stroke();
      if (S.path.target) mark(v, S.path.target, "+", T.path);
      if (S.path.goal) mark(v, S.path.goal, "★", T.path);
    }
    // the passage plan, Ekko's or MANUAL
    if (S.plan) {
      for (const b of S.plan.buoys) buoy(v, b, S.plan.source === "manual" ? 0.6 : 1.0);
      if (S.plan.entry) mark(v, S.plan.entry, "ENTRY", BEACON[4][1]);
      if (S.plan.exit) mark(v, S.plan.exit, "EXIT", BEACON[5][1]);
    }
    // the field being edited, each buoy numbered by its list index
    S.manual.forEach((b, i) => buoy(v, b, 1.0, i));
    // tracked targets
    for (const t of S.targets) {
      const p = at(v, t.lat, t.lon), col = labelColour(t.label);
      disc(c, p[0], p[1], 5, t.confirmed ? col : null, col, 2);
      label(c, `${t.label} #${t.id}`, p[0] + 7, p[1] - 6);
    }
    boat(v);
    north(v);
    scaleBar(v);
    readout(v);
  }
  // 1-2-5 x 10^n nearest to m: [step, subdivisions of it into nice minor steps]
  function nice(m) {
    const p = Math.pow(10, Math.floor(Math.log10(m))), r = m / p;
    const lead = r < 1.414 ? 1 : r < 3.162 ? 2 : r < 7.071 ? 5 : 10;
    return [lead * p, lead === 2 ? 4 : 5];
  }
  function fmtM(m) { return `${+m.toFixed(3)} m`; }
  function label(c, s, x, y, align = "left", base = "alphabetic", col = T.text) {
    c.textAlign = align; c.textBaseline = base;
    c.lineWidth = 3; c.strokeStyle = T.bg; c.strokeText(s, x, y);   // halo: legible on any ground
    c.fillStyle = col; c.fillText(s, x, y);
  }
  function halo(c, lw) { c.lineWidth = lw + 3; c.strokeStyle = T.bg; c.stroke(); }
  function disc(c, x, y, r, fill, line, lw, dash) {
    c.beginPath(); c.arc(x, y, r, 0, 2 * Math.PI);
    halo(c, lw);
    if (fill) { c.fillStyle = fill; c.fill(); }
    c.setLineDash(dash || []); c.lineWidth = lw; c.strokeStyle = line; c.stroke(); c.setLineDash([]);
  }
  function grid(v) {
    const c = v.ctx, [major, div] = nice(90 / v.scale), minor = major / div;
    const lines = new Path2D(), heavy = new Path2D(), tags = [];
    for (const ax of [0, 1]) {                    // 0: vertical lines at east metres, 1: horizontal at north
      const mid = v.centre[ax], half = (ax ? v.h : v.w) / 2 / v.scale;
      for (let i = Math.ceil((mid - half) / minor); i <= Math.floor((mid + half) / minor); i++) {
        const g = i * minor, big = i % div === 0, p = big ? heavy : lines;
        const s = Math.round(ax ? v.h / 2 - (g - mid) * v.scale : v.w / 2 + (g - mid) * v.scale) + 0.5;
        if (ax) { p.moveTo(0, s); p.lineTo(v.w, s); } else { p.moveTo(s, 0); p.lineTo(s, v.h); }
        if (!big) continue;
        if (ax && s > 14 && s < v.h - 24) tags.push([fmtM(g), 4, s - 2, "left", "bottom"]);
        if (!ax && s > 40 && s < v.w - 40) tags.push([fmtM(g), s + 3, v.h - 3, "left", "bottom"]);
      }
    }
    c.lineWidth = 1;
    c.strokeStyle = T.grid; c.stroke(lines);
    c.strokeStyle = T.major; c.stroke(heavy);
    for (const t of tags) label(c, ...t);
  }
  function buoy(v, b, alpha, index) {
    const c = v.ctx, p = at(v, b.lat, b.lon), edited = index != null;
    c.globalAlpha = alpha;
    disc(c, p[0], p[1], 7, (BEACON[b.beacon] || BEACON[0])[1], T.text, edited ? 3 : 1.5,
         FLASHING.has(b.beacon) ? [3, 3] : null);
    if (edited) {
      c.font = FONT_BOLD; label(c, String(index), p[0] + 10, p[1] - 8); c.font = FONT;
    }
    c.globalAlpha = 1;
  }
  function mark(v, q, text, col) {
    const p = at(v, q[0], q[1]);
    v.ctx.font = FONT_BOLD; label(v.ctx, text, p[0] + 9, p[1] + 4, "left", "alphabetic", col); v.ctx.font = FONT;
  }
  function boat(v) {
    if (!S.boat) return;
    const c = v.ctx, p = at(v, S.boat.lat, S.boat.lon);
    const a = ((S.boat.hdg == null ? 0 : S.boat.hdg) - 90) * Math.PI / 180;
    c.save(); c.translate(p[0], p[1]); c.rotate(a);
    const col = S.boat.stale ? T.text : T.boat;
    if (S.boat.stale) c.globalAlpha = 0.5;
    c.beginPath(); c.moveTo(16, 0); c.lineTo(-8, 6); c.lineTo(-4, 0); c.lineTo(-8, -6); c.closePath();
    c.fillStyle = col; c.fill();
    if (S.boat.hdg != null) {                     // the heading line: direction at a glance
      c.beginPath(); c.moveTo(16, 0); c.lineTo(46, 0); c.strokeStyle = col; c.lineWidth = 1.5; c.stroke();
    }
    c.restore();
    if (S.boat.stale || S.boat.hdg == null)
      label(c, S.boat.stale ? "position stale" : "no heading", p[0] + 12, p[1] + 16, "left", "alphabetic", T.hazard);
  }
  function north(v) {
    const c = v.ctx, x = v.w - 24;
    c.beginPath(); c.moveTo(x, 24); c.lineTo(x + 6, 44); c.lineTo(x, 38); c.lineTo(x - 6, 44); c.closePath();
    halo(c, 0);
    c.fillStyle = T.text; c.fill();
    label(c, "N", x, 22, "center", "bottom");
  }
  function scaleBar(v) {
    const c = v.ctx, [step] = nice(100 / v.scale), w = step * v.scale, x = v.w - 12, y = v.h - 28;
    c.beginPath(); c.moveTo(x - w, y - 4); c.lineTo(x - w, y); c.lineTo(x, y); c.lineTo(x, y - 4);
    halo(c, 2);
    c.lineWidth = 2; c.strokeStyle = T.text; c.stroke();
    label(c, `${fmtM(step)}   (wheel: zoom, drag: pan, double-click: follow)`, x, y - 6, "right");
  }
  function readout(v) {
    if (!v.mouse) return;
    const w = toWorld(v, v.mouse[0], v.mouse[1]), ll = latlon(w[0], w[1]);
    label(v.ctx, `lat ${ll[0].toFixed(7)}  lon ${ll[1].toFixed(7)}`, v.w - 44, 8, "right", "top");
    label(v.ctx, `E ${w[0].toFixed(1)} m  N ${w[1].toFixed(1)} m`, v.w - 44, 24, "right", "top");
  }

  // ---- the MANUAL field -----------------------------------------------------
  function validPos(lat, lon) {
    return Number.isFinite(lat) && Number.isFinite(lon) && Math.abs(lat) <= 90 && Math.abs(lon) <= 180;
  }
  function validBuoy(lat, lon, beacon) {
    return validPos(lat, lon) && Number.isInteger(beacon) && beacon >= 0 && beacon <= 5;
  }
  function plain(b) { return {lat: b.lat, lon: b.lon, beacon: b.beacon}; }
  function manual() { return S.manual.map(plain); }
  function changed() {                            // after every add, remove or replace, clicks included
    if (!S.origin && S.manual.length) S.origin = [S.manual[0].lat, S.manual[0].lon];
    redraw();
    const list = manual();
    for (const fn of listeners) {
      try { fn(list); } catch (e) { console.error("CrsdMap listener:", e); }
    }
  }
  function addManual(lat, lon, beacon) {
    if (!validBuoy(lat, lon, beacon)) return -1;
    S.manual.push({lat, lon, beacon});
    changed();
    return S.manual.length - 1;
  }
  function removeManual(i) {
    if (!Number.isInteger(i) || i < 0 || i >= S.manual.length) return false;
    S.manual.splice(i, 1);
    changed();
    return true;
  }
  function setManual(list) {                      // bad entries are dropped; returns how many were kept
    if (!Array.isArray(list)) return -1;
    S.manual = list.filter(b => b && validBuoy(b.lat, b.lon, b.beacon)).map(plain);
    changed();
    return S.manual.length;
  }
  function onManualChange(fn) { if (typeof fn === "function") listeners.push(fn); }
  function centreOn(lat, lon) {
    if (!validPos(lat, lon)) return false;
    const p = xy(lat, lon);
    for (const v of views) { v.follow = false; v.centre = p.slice(); }
    redraw();
    return true;
  }
  function edit(v, px, py) {
    const near = S.manual.findIndex(b => {
      const p = at(v, b.lat, b.lon);
      return Math.hypot(p[0] - px, p[1] - py) < 10;
    });
    if (near >= 0) removeManual(near);
    else {
      const q = latlon(...toWorld(v, px, py));
      addManual(q[0], q[1], S.beacon);
    }
  }
  function manualPlan() {
    const buoys = S.manual.map((b, i) => ({id: i + 1, lat: b.lat, lon: b.lon, beacon: b.beacon}));
    const at = code => { const b = buoys.find(x => x.beacon === code); return b ? [b.lat, b.lon] : null; };
    return {buoys, entry: at(4), exit: at(5)};
  }
  function loadPlan() { setManual(S.plan ? S.plan.buoys : []); }

  matchMedia("(prefers-color-scheme: dark)").addEventListener("change", redraw);

  return {
    BEACON, update, attach, redraw, centreOn,
    setEditing(on) { S.editing = on; redraw(); },
    setBeacon(code) { S.beacon = code; },
    manual, addManual, removeManual, setManual, onManualChange, onEdit: onManualChange,
    manualPlan, loadPlan,
    clearManual() { setManual([]); },
    clearTrail() { S.trail = []; redraw(); },
  };
})();
