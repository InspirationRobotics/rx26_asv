// map.js: the ground station's one map. One state, drawn into every canvas that shows it
// (the Map tab and the Mission tab), in metres east/north of the first position heard.
// The MANUAL field editor works on the same state: what you place is what the boat is sent.
"use strict";
const CrsdMap = (() => {
  const M_PER_DEG = 111320;
  const BEACON = {
    0: ["unknown", "#9e9e9e"], 1: ["off", "#424242"], 2: ["flashing red", "#e53935"],
    3: ["flashing green", "#43a047"], 4: ["flashing blue: ENTRY", "#1e88e5"],
    5: ["steady blue: EXIT", "#0d47a1"],
  };
  const FLASHING = new Set([2, 3, 4]);
  const S = {
    origin: null, boat: null, trail: [], targets: [], plan: null, path: null, hazards: [],
    manual: [], editing: false, beacon: 2, onEdit: null,
  };
  const views = [];

  function xy(lat, lon) {
    if (!S.origin) S.origin = [lat, lon];
    const k = Math.cos(S.origin[0] * Math.PI / 180);
    return [(lon - S.origin[1]) * M_PER_DEG * k, (lat - S.origin[0]) * M_PER_DEG];
  }
  function latlon(x, y) {
    const k = Math.cos(S.origin[0] * Math.PI / 180);
    return [S.origin[0] + y / M_PER_DEG, S.origin[1] + x / (M_PER_DEG * k)];
  }
  function css(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || "#888";
  }
  function labelColour(label) {
    const l = (label || "").toLowerCase();
    for (const [w, c] of [["red", "#e53935"], ["green", "#43a047"], ["blue", "#1e88e5"],
                          ["yellow", "#fdd835"], ["white", "#fafafa"], ["black", "#212121"],
                          ["off", "#616161"]]) if (l.includes(w)) return c;
    return css("--muted");
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
    redraw();
  }

  // ---- views ----------------------------------------------------------------
  function attach(cv, opts) {
    const v = {cv, ctx: cv.getContext("2d"), scale: 4, centre: [0, 0], follow: true, drag: null,
               editable: !!(opts && opts.editable)};
    views.push(v);
    cv.addEventListener("wheel", e => {
      e.preventDefault();
      const before = toWorld(v, e.offsetX, e.offsetY);
      v.scale = Math.max(0.2, Math.min(200, v.scale * (e.deltaY < 0 ? 1.25 : 0.8)));
      const after = toWorld(v, e.offsetX, e.offsetY);
      v.centre = [v.centre[0] + before[0] - after[0], v.centre[1] + before[1] - after[1]];
      v.follow = false;
      draw(v);
    }, {passive: false});
    cv.addEventListener("mousedown", e => { v.drag = {x: e.offsetX, y: e.offsetY, moved: false}; });
    cv.addEventListener("mousemove", e => {
      if (!v.drag) return;
      const dx = e.offsetX - v.drag.x, dy = e.offsetY - v.drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 3) v.drag.moved = true;
      if (!v.drag.moved) return;
      v.centre = [v.centre[0] - dx / v.scale, v.centre[1] + dy / v.scale];
      v.drag.x = e.offsetX; v.drag.y = e.offsetY; v.follow = false;
      draw(v);
    });
    cv.addEventListener("mouseup", e => {
      const click = v.drag && !v.drag.moved;
      v.drag = null;
      if (click && v.editable && S.editing && S.origin) edit(v, e.offsetX, e.offsetY);
    });
    cv.addEventListener("dblclick", () => { v.follow = true; draw(v); });
    new ResizeObserver(() => draw(v)).observe(cv);
    return v;
  }
  function toWorld(v, px, py) {
    return [v.centre[0] + (px - v.cv.width / 2) / v.scale,
            v.centre[1] - (py - v.cv.height / 2) / v.scale];
  }
  function toScreen(v, p) {
    return [v.cv.width / 2 + (p[0] - v.centre[0]) * v.scale,
            v.cv.height / 2 - (p[1] - v.centre[1]) * v.scale];
  }
  function redraw() { views.forEach(draw); }

  // ---- drawing --------------------------------------------------------------
  function draw(v) {
    const cv = v.cv, c = v.ctx;
    if (!cv.offsetParent) return;                 // a hidden tab draws nothing
    cv.width = cv.clientWidth; cv.height = cv.clientHeight;
    c.fillStyle = css("--map-bg"); c.fillRect(0, 0, cv.width, cv.height);
    c.font = "12px system-ui, sans-serif";
    if (!S.origin) {
      c.fillStyle = css("--muted");
      c.fillText("no position yet: the map starts at the first fix", 12, 20);
      return;
    }
    if (v.follow && S.boat) v.centre = xy(S.boat.lat, S.boat.lon);
    rings(v);
    // hazards: what the planner keeps out of
    c.strokeStyle = "#ff9800"; c.lineWidth = 1;
    for (const h of S.hazards) {
      c.beginPath();
      if (h.c) {
        const p = toScreen(v, xy(h.c[0], h.c[1]));
        c.arc(p[0], p[1], Math.max(2, ((h.r || 0) + (h.keep || 0)) * v.scale), 0, 2 * Math.PI);
      } else if (h.poly) {
        h.poly.forEach((q, i) => { const p = toScreen(v, xy(q[0], q[1]));
                                   i ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1]); });
        c.closePath();
      }
      c.stroke();
    }
    // trail
    c.strokeStyle = css("--trail"); c.lineWidth = 1.5; c.beginPath();
    S.trail.forEach((p, i) => { const s = toScreen(v, p); i ? c.lineTo(s[0], s[1]) : c.moveTo(s[0], s[1]); });
    c.stroke();
    // the planned leg
    if (S.path) {
      c.strokeStyle = "#00bcd4"; c.lineWidth = 2; c.beginPath();
      (S.path.path || []).forEach((q, i) => { const s = toScreen(v, xy(q[0], q[1]));
                                             i ? c.lineTo(s[0], s[1]) : c.moveTo(s[0], s[1]); });
      c.stroke();
      if (S.path.target) mark(v, S.path.target, "+", "#00bcd4");
      if (S.path.goal) mark(v, S.path.goal, "★", "#00bcd4");
    }
    // the passage plan, Ekko's or MANUAL
    if (S.plan) {
      for (const b of S.plan.buoys) buoy(v, b, S.plan.source === "manual" ? 0.6 : 1.0);
      if (S.plan.entry) mark(v, S.plan.entry, "ENTRY", BEACON[4][1]);
      if (S.plan.exit) mark(v, S.plan.exit, "EXIT", BEACON[5][1]);
    }
    // the field being edited
    for (const b of S.manual) buoy(v, b, 1.0, true);
    // tracked targets
    for (const t of S.targets) {
      const p = toScreen(v, xy(t.lat, t.lon));
      c.beginPath(); c.arc(p[0], p[1], 5, 0, 2 * Math.PI);
      c.strokeStyle = labelColour(t.label); c.lineWidth = 2;
      if (t.confirmed) { c.fillStyle = labelColour(t.label); c.fill(); }
      c.stroke();
      c.fillStyle = css("--fg");
      c.fillText(`${t.label} #${t.id}`, p[0] + 7, p[1] - 6);
    }
    boat(v);
    scaleBar(v);
  }
  function buoy(v, b, alpha, edited) {
    const c = v.ctx, p = toScreen(v, xy(b.lat, b.lon)), col = (BEACON[b.beacon] || BEACON[0])[1];
    c.globalAlpha = alpha;
    c.beginPath(); c.arc(p[0], p[1], 7, 0, 2 * Math.PI);
    c.fillStyle = col; c.fill();
    c.setLineDash(FLASHING.has(b.beacon) ? [3, 3] : []);
    c.strokeStyle = edited ? css("--accent") : css("--fg"); c.lineWidth = edited ? 3 : 1.5;
    c.stroke(); c.setLineDash([]);
    c.globalAlpha = 1;
  }
  function mark(v, q, text, col) {
    const c = v.ctx, p = toScreen(v, xy(q[0], q[1]));
    c.fillStyle = col; c.font = "bold 12px system-ui, sans-serif";
    c.fillText(text, p[0] + 9, p[1] + 4); c.font = "12px system-ui, sans-serif";
  }
  function boat(v) {
    if (!S.boat) return;
    const c = v.ctx, p = toScreen(v, xy(S.boat.lat, S.boat.lon));
    const a = ((S.boat.hdg == null ? 0 : S.boat.hdg) - 90) * Math.PI / 180;
    c.save(); c.translate(p[0], p[1]); c.rotate(a);
    const col = S.boat.stale ? css("--muted") : css("--accent");
    c.beginPath(); c.moveTo(16, 0); c.lineTo(-8, 6); c.lineTo(-4, 0); c.lineTo(-8, -6); c.closePath();
    c.fillStyle = col; c.fill();
    if (S.boat.hdg != null) {                     // the heading line: direction at a glance
      c.beginPath(); c.moveTo(16, 0); c.lineTo(46, 0); c.strokeStyle = col; c.lineWidth = 1.5; c.stroke();
    }
    c.restore();
    if (S.boat.stale || S.boat.hdg == null) {
      c.fillStyle = css("--warn");
      c.fillText(S.boat.stale ? "position stale" : "no heading", p[0] + 12, p[1] + 16);
    }
  }
  function niceStep(m) {
    const p = Math.pow(10, Math.floor(Math.log10(m))), r = m / p;
    return (r < 2 ? 1 : r < 5 ? 2 : 5) * p;
  }
  function rings(v) {
    if (!S.boat) return;
    const c = v.ctx, p = toScreen(v, xy(S.boat.lat, S.boat.lon)), step = niceStep(80 / v.scale);
    c.strokeStyle = css("--grid"); c.lineWidth = 1; c.fillStyle = css("--muted");
    for (let r = step; r * v.scale < Math.hypot(v.cv.width, v.cv.height); r += step) {
      c.beginPath(); c.arc(p[0], p[1], r * v.scale, 0, 2 * Math.PI); c.stroke();
      c.fillText(`${r} m`, p[0] + 3, p[1] - r * v.scale - 2);
    }
  }
  function scaleBar(v) {
    const c = v.ctx, step = niceStep(100 / v.scale), w = step * v.scale, y = v.cv.height - 14;
    c.strokeStyle = css("--fg"); c.lineWidth = 2;
    c.beginPath(); c.moveTo(12, y); c.lineTo(12 + w, y); c.stroke();
    c.fillStyle = css("--fg"); c.fillText(`${step} m   (wheel: zoom, drag: pan, double-click: follow)`, 12, y - 5);
  }

  // ---- the MANUAL field -----------------------------------------------------
  function edit(v, px, py) {
    const near = S.manual.findIndex(b => {
      const p = toScreen(v, xy(b.lat, b.lon));
      return Math.hypot(p[0] - px, p[1] - py) < 10;
    });
    if (near >= 0) S.manual.splice(near, 1);
    else {
      const q = latlon(...toWorld(v, px, py));
      S.manual.push({lat: q[0], lon: q[1], beacon: S.beacon});
    }
    if (S.onEdit) S.onEdit();
    redraw();
  }
  function manualPlan() {
    const buoys = S.manual.map((b, i) => ({id: i + 1, lat: b.lat, lon: b.lon, beacon: b.beacon}));
    const at = code => { const b = buoys.find(x => x.beacon === code); return b ? [b.lat, b.lon] : null; };
    return {buoys, entry: at(4), exit: at(5)};
  }
  function loadPlan() {
    S.manual = S.plan ? S.plan.buoys.map(b => ({lat: b.lat, lon: b.lon, beacon: b.beacon})) : [];
    if (S.onEdit) S.onEdit();
    redraw();
  }

  return {
    BEACON, update, attach, redraw,
    setEditing(on) { S.editing = on; redraw(); },
    setBeacon(code) { S.beacon = code; },
    onEdit(fn) { S.onEdit = fn; },
    manual() { return S.manual; },
    manualPlan, loadPlan,
    clearManual() { S.manual = []; if (S.onEdit) S.onEdit(); redraw(); },
    clearTrail() { S.trail = []; redraw(); },
  };
})();
