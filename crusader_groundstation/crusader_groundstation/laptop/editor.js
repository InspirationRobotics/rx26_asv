// editor.js: the Mission tab's block editor. A mission is nested blocks, built by dragging them out of
// the palette (or clicking them); the boat's catalogue says which blocks and ports exist. Nothing
// here knows what makes a mission valid: gcs_app turns the blocks into the mission XML
// (mission_build) and the reasons it is refused come back from mission_xml, drawn on the block
// they are about. Save sends the checked XML to the boat (mission_upload).
// The GPS waypoints of the draft are shown on the map (CrsdMap.setDraft), numbered in run order.
// The DECLARATION row is the tier chosen for each task for this run: it is saved on the mission
// (mission_build's `declare`), sent to the OCS on request (op ocs_tiers) and compared with what the
// OCS says it will declare (status.ocs_run). Until the operator touches it, it follows the draft's
// task blocks; a mission that is opened or a template brings its own and keeps it.
"use strict";
const CrsdEditor = (() => {
  const $ = id => document.getElementById(id);
  const esc = s => deps.esc(s);
  const KEY = "crsd-mission-draft";                 // per-viewer convenience: the draft survives a reload
  const TASKS = {1: "Task 1 Safe Passage", 2: "Task 2 Survey and Repair", 3: "Task 3 Logistics",
                 4: "Task 4 Incident Response"};
  const TIERS = ["core", "advanced", "disruptive"];
  const DECL_TIERS = ["none"].concat(TIERS);        // what a task can be declared at
  const TASK_KEYS = ["task1", "task2", "task3", "task4"];
  const HIDDEN_BLOCKS = ["WaitForRunStart"];        // the runner always gates now; old missions still load and run
  const POSE_FRESH_S = 5, OCS_OLD_S = 120;
  const GROUPS = [["control", "Controls"], ["decorator", "Decorators"], ["action", "Actions"],
                  ["condition", "Conditions"]];
  const DRAG_PX = 4, VALIDATE_MS = 350, UNDO_MAX = 60, SCROLL_PX = 28;

  // blocks: {uid, id, name, subtree, ports: {name: text}, children}; uid is this page's own handle
  const noTiers = () => ({task1: "none", task2: "none", task3: "none", task4: "none"});
  const E = {blocks: [], sel: null, name: "", cat: null, why: "", problems: [], xml: "", dirty: false,
             undo: [], redo: [], rev: 0, gps: new Map(), declare: noTiers(), declTouched: false};
  const byUid = new Map();                          // uid -> block, rebuilt by every render
  let deps = null, nextUid = 1, timer = null, drag = null, pre = null;

  // ---- blocks ---------------------------------------------------------------
  const adopt = b => ({uid: nextUid++, id: String(b.id), name: b.name || "", subtree: !!b.subtree,
                       ports: Object.assign({}, b.ports), children: (b.children || []).map(adopt)});
  const plain = b => Object.assign({id: b.id, ports: b.ports, children: b.children.map(plain)},
                                   b.name ? {name: b.name} : {}, b.subtree ? {subtree: true} : {});
  function* walk(list, path = [], owner = null) {
    for (let i = 0; i < list.length; i++) {
      yield {block: list[i], path: path.concat(i), list, index: i, owner};
      yield* walk(list[i].children, path.concat(i), list[i]);
    }
  }
  const locate = u => { for (const w of walk(E.blocks)) if (w.block.uid === u) return w; return null; };
  function atPath(path) {
    let list = E.blocks, b = null;
    for (const i of path) { b = list[i]; if (!b) return null; list = b.children; }
    return b;
  }
  const contains = (outer, b) => outer.children.some(k => k === b || contains(k, b));
  const specOf = b => (E.cat && (b.subtree ? E.cat.tasks : E.cat.nodes).get(b.id)) || null;
  const isTask = s => !!s && s.task != null && !!s.tier;     // a shared body (Task1PlanDrive) has no task or tier
  const room = b => {                               // how many more blocks b can hold
    const spec = specOf(b), max = spec && !b.subtree ? spec.max_children : 0;
    return max == null ? Infinity : Math.max(0, max - b.children.length);
  };
  const blueprint = key => {                        // "gps", "hold", "node:Sequence" or "task:Task1Advanced"
    const [kind, id] = key.split(":");
    return kind === "gps" ? {id: "NavigateTo", ports: {target: "fix", lat: "", lon: ""}}
         : kind === "hold" ? {id: "HoldHeading", ports: {heading_deg: "0", duration_s: "30"}}
                          : {id, subtree: kind === "task", ports: {}};
  };

  // a GPS block is NavigateTo (or CircleBuoy) pointed at "fix": its lat and lon are a place
  const gpsPort = b => (E.cat && !b.subtree ? E.cat.fix[b.id] : null);
  const isGps = b => { const port = gpsPort(b); return !!port && b.ports[port] === "fix"; };
  const num = s => (String(s == null ? "" : s).trim() === "" ? NaN : Number(s));
  function gpsPoint(b) {                            // the place a GPS block names, null while it names none
    if (!isGps(b)) return null;
    const lat = num(b.ports.lat), lon = num(b.ports.lon);
    const ok = Number.isFinite(lat) && Number.isFinite(lon) && Math.abs(lat) <= 90 && Math.abs(lon) <= 180;
    return ok && (lat || lon) ? {lat, lon} : null;
  }
  // HoldHeading: the boat holds where the block before it left it, pointed at heading_deg
  const isHold = b => !b.subtree && b.id === "HoldHeading";
  const wrap360 = h => ((h % 360) + 360) % 360;
  function holdHeading(b) {                         // its heading_deg as a number, null while it is not one
    const h = num(b.ports.heading_deg);
    return isHold(b) && Number.isFinite(h) ? h : null;
  }
  function holdLabel(b) {                           // "Hold heading 090° for 30 s"
    const h = holdHeading(b), d = num(b.ports.duration_s);
    const deg = h == null ? (b.ports.heading_deg || "?") : String(Math.round(wrap360(h))).padStart(3, "0");
    return `Hold heading ${deg}° for ${Number.isFinite(d) ? d : 30} s`;
  }
  const kindOf = b => b.subtree ? "task" : isGps(b) ? "gps" : (specOf(b) || {}).kind || "unknown";
  function title(b) {
    const spec = specOf(b);
    if (isGps(b)) return E.gps.has(b.uid) ? `GPS waypoint ${E.gps.get(b.uid)}` : "GPS waypoint";
    if (isHold(b)) return holdLabel(b);
    if (b.subtree && isTask(spec)) return `${TASKS[spec.task] || "Task " + spec.task}, ${spec.tier}`;
    return b.name ? `${b.id} "${b.name}"` : b.id;
  }
  function summary(b) {
    if (isGps(b)) { const p = gpsPoint(b); return p ? `${p.lat.toFixed(6)}, ${p.lon.toFixed(6)}` : "no position yet"; }
    const shown = isHold(b) ? ["heading_deg", "duration_s"] : [];      // the title already says these
    return Object.entries(b.ports).filter(([k, v]) => v !== "" && !shown.includes(k)).map(([k, v]) => `${k}=${v}`).join("  ");
  }

  // ---- changes --------------------------------------------------------------
  const snapshot = () => JSON.stringify({name: E.name, blocks: E.blocks.map(plain), declare: E.declare,
                                         touched: E.declTouched});
  function remember(before) {
    E.undo.push(before || snapshot());
    if (E.undo.length > UNDO_MAX) E.undo.shift();
    E.redo = [];
  }
  function blockTiers() {                           // the tier of each task's block in the draft, else none
    const out = noTiers();
    for (const {block} of walk(E.blocks)) {
      const spec = block.subtree && specOf(block);
      if (isTask(spec) && out["task" + spec.task] !== undefined) out["task" + spec.task] = spec.tier;
    }
    return out;
  }
  function changed(structural) {                    // after any edit; structural ones redraw the blocks
    E.dirty = true; E.rev++;
    if (!E.declTouched) E.declare = blockTiers();
    syncDraft();
    if (structural) render(); else paintHeads();
    clearTimeout(timer);
    timer = setTimeout(validate, VALIDATE_MS);
    persist();
  }
  function edit(fn) { remember(); fn(); changed(true); }
  function restore(json) {
    const s = JSON.parse(json);
    E.blocks = s.blocks.map(adopt); E.sel = null;
    E.declare = s.declare || noTiers(); E.declTouched = !!s.touched;
    setName(s.name);
    changed(true);
  }
  function undo() { if (E.undo.length) { E.redo.push(snapshot()); restore(E.undo.pop()); } }
  function redo() { if (E.redo.length) { E.undo.push(snapshot()); restore(E.redo.pop()); } }
  function setName(name) { E.name = name; $("ed-name").value = name; }
  function load(blocks, name, declare) {            // the draft becomes these blocks: undoable, and nothing unsaved
    remember();
    E.blocks = blocks; E.sel = null;
    E.declare = declare ? Object.assign(noTiers(), declare) : noTiers(); E.declTouched = !!declare;
    setName(name);
    changed(true);
    E.dirty = false;
  }
  function select(u) {
    E.sel = E.sel === u ? null : u;
    syncDraft();
    render();
    const el = E.sel && $("ed-root").querySelector(`.blk[data-uid="${E.sel}"]`);
    if (el) el.scrollIntoView({block: "nearest"});
  }
  function target() {                               // where a block added "anywhere" goes: [list, index]
    // into the selected block if it is an empty container, else right after it: so an inserted
    // mission (selected) is followed by the next waypoint or mission, not swallowed by it
    const s = E.sel && locate(E.sel);
    if (s && !s.block.children.length && room(s.block) > 0) return [s.block.children, 0];
    if (s && (!s.owner || room(s.owner) > 0)) return [s.list, s.index + 1];
    return [E.blocks, E.blocks.length];
  }
  const place = (key, list, at) => edit(() => {
    const b = adopt(blueprint(key));
    list.splice(at, 0, b);
    E.sel = b.uid;
  });
  const move = (u, list, at) => edit(() => {
    const w = locate(u);
    w.list.splice(w.index, 1);
    list.splice(w.list === list && w.index < at ? at - 1 : at, 0, w.block);
    E.sel = u;
  });
  function remove(u) {
    const w = locate(u);
    if (w) edit(() => { w.list.splice(w.index, 1); if (E.sel === u) E.sel = null; });
  }
  function duplicate(u) {
    const w = locate(u);
    if (!w) return;
    if (w.owner && room(w.owner) < 1) return deps.toast(`${title(w.owner)} cannot hold another block`, false);
    edit(() => { const c = adopt(plain(w.block)); w.list.splice(w.index + 1, 0, c); E.sel = c.uid; });
  }
  function addWaypoint(lat, lon) {                  // a click on the map in "add waypoints" mode
    edit(() => {
      const [list, at] = target(), b = adopt(blueprint("gps"));
      b.ports.lat = lat.toFixed(7); b.ports.lon = lon.toFixed(7);
      list.splice(at, 0, b);
      E.sel = b.uid;
    });
  }
  function pick(b) {                                // "pick on map": the next click is this block's place
    if (!CrsdMap.hasOrigin()) return deps.toast("the map has no position yet: type the lat/lon instead", false);
    CrsdMap.beginPick(point => {
      if (!point || !byUid.has(b.uid)) return;
      remember();
      b.ports.lat = point[0].toFixed(7); b.ports.lon = point[1].toFixed(7);
      changed(true);
    });
  }

  // ---- the map --------------------------------------------------------------
  function syncDraft() {                            // the draft's valid GPS points, numbered in run order
    const points = [];
    let ring = -1;
    E.gps.clear();
    for (const {block} of walk(E.blocks)) {
      const hold = holdHeading(block);
      if (hold != null && points.length) points[points.length - 1].heading_deg = wrap360(hold);
      const p = gpsPoint(block);
      if (!p) continue;
      points.push(p);
      E.gps.set(block.uid, points.length);
      if (block.uid === E.sel) ring = points.length - 1;
    }
    CrsdMap.setDraft(points, ring);
  }

  // ---- drawing --------------------------------------------------------------
  function control(b, p) {
    const v = b.ports[p.name] || "", def = p.default == null ? "" : String(p.default), at = `data-port="${esc(p.name)}"`;
    const choices = p.choices || (p.kind === "bool" ? ["true", "false"] : null);
    if (choices) {
      const all = v && !choices.includes(v) ? choices.concat(v) : choices;      // a value the list lacks stays
      return `<select ${at}><option value="">${def ? "default: " + esc(def) : "-"}</option>` +
        all.map(c => `<option${c === v ? " selected" : ""}>${esc(c)}</option>`).join("") + "</select>";
    }
    const hint = def !== "" ? `default ${def}` : p.required ? "required" : "optional";
    const mode = p.kind === "int" || p.kind === "real" ? ' inputmode="decimal"' : "";
    return `<input ${at} value="${esc(v)}" placeholder="${esc(hint)}"${mode} spellcheck="false" autocomplete="off">`;
  }
  const row = (label, tip, inner) => `<label title="${esc(tip)}">${esc(label)}</label><div>${inner}</div>`;
  function fieldsHtml(b, spec) {
    const ports = spec ? spec.ports : [], known = new Set(ports.map(p => p.name));
    let h = row("label", "optional name shown in the tree", `<input data-label value="${esc(b.name)}" ` +
                'placeholder="optional" spellcheck="false" autocomplete="off">');
    for (const p of ports) {
      if (p.dir === "out" && !b.ports[p.name]) continue;          // an output is wired only if it was
      h += row(p.name + (p.required ? " *" : ""), p.doc || p.type || "", control(b, p));
      if (p.name === "lon" && isGps(b)) h += row("", "", '<button data-pick>pick on map</button>');
    }
    for (const k of Object.keys(b.ports).filter(k => !known.has(k)))
      h += row(k, "not a port of this block", control(b, {name: k}));
    return `<div class="bf">${h}</div>`;
  }
  function blockHtml(b) {
    const spec = specOf(b), open = b.uid === E.sel, holds = b.children.length || room(b) > 0;
    return `<div class="blk k-${kindOf(b)}${open ? " sel" : ""}" data-uid="${b.uid}">` +
      `<div class="bh"><span class="bt">${esc(title(b))}</span><span class="bsum muted">${esc(summary(b))}</span>` +
      '<button data-act="dup" title="duplicate">&#10697;</button><button data-act="del" title="remove">&#10005;</button></div>' +
      (open ? fieldsHtml(b, spec) : "") + '<div class="bp"></div>' +
      (holds ? `<div class="blist" data-uid="${b.uid}">${b.children.map(blockHtml).join("")}</div>` : "") + "</div>";
  }
  function paintHistory() { $("ed-undo").disabled = !E.undo.length; $("ed-redo").disabled = !E.redo.length; }
  function render() {
    byUid.clear();
    for (const w of walk(E.blocks)) byUid.set(w.block.uid, w.block);
    $("ed-root").innerHTML = E.blocks.map(blockHtml).join("");
    paintProblems(); paintTiers(); paintDecl();
    paintHistory();
  }
  function paintHeads() {                           // the titles and summaries, without touching the fields
    for (const el of $("ed-root").querySelectorAll(".blk")) {
      const b = byUid.get(+el.dataset.uid);
      el.querySelector(":scope > .bh .bt").textContent = title(b);
      el.querySelector(":scope > .bh .bsum").textContent = summary(b);
    }
    paintTiers(); paintDecl();
    paintHistory();
  }
  function paintProblems() {                        // each reason on its block; the rest above the blocks
    for (const el of $("ed-root").querySelectorAll(".blk")) {
      el.classList.remove("bad");
      el.querySelector(":scope > .bp").textContent = "";
    }
    const general = [];
    for (const p of E.problems) {
      const b = p.path && atPath(p.path), el = b && $("ed-root").querySelector(`.blk[data-uid="${b.uid}"]`);
      if (!el) { general.push(p); continue; }
      if (!isWarn(p)) el.classList.add("bad");
      const line = document.createElement("div");
      line.textContent = p.msg.replace(/^.*?: /, "");             // "mission > a[1] > b[2]: " is the block itself
      if (isWarn(p)) line.className = "warn";
      el.querySelector(":scope > .bp").appendChild(line);
    }
    $("ed-problems").innerHTML = general.map(p => `<div class="${isWarn(p) ? "warn" : "bad"}">${esc(p.msg)}</div>`).join("");
    const n = errors(E.problems).length, w = E.problems.length - n, st = $("ed-status");
    const said = [n ? `${n} problem${n > 1 ? "s" : ""}` : "", w ? `${w} warning${w > 1 ? "s" : ""}` : ""].filter(Boolean).join(", ");
    st.className = "small " + (n ? "bad" : w ? "warn" : "muted");
    st.textContent = E.why ? E.why : said || (E.blocks.length ? "no problems" : "");
  }
  function paintTiers() {                           // a task's other tiers go grey once one is used
    const used = new Map();
    for (const {block} of walk(E.blocks)) {
      const spec = block.subtree && specOf(block);
      if (isTask(spec)) used.set(String(spec.task), spec.tier);
    }
    for (const el of $("ed-pal").querySelectorAll("[data-task]")) {
      const taken = used.get(el.dataset.task), off = taken !== undefined && taken !== el.dataset.tier;
      el.classList.toggle("off", off);
      el.title = off ? `Task ${el.dataset.task} already uses ${taken}: a mission uses one tier of a task` : el.dataset.tip;
    }
  }

  // ---- the palette ----------------------------------------------------------
  function paletteHtml() {
    const d = E.cat.data, q = $("ed-find").value.trim().toLowerCase();
    const item = (kind, key, label, tip, extra = "", full = label) => (!q || `${full} ${tip}`.toLowerCase().includes(q))
      ? `<div class="pal k-${kind}" data-key="${esc(key)}" data-label="${esc(full)}" data-tip="${esc(tip)}" ` +
        `title="${esc(tip)}"${extra}>${esc(label)}</div>` : "";
    const group = (name, items) => items.some(Boolean) ? `<details open><summary>${esc(name)}</summary>${items.join("")}</details>` : "";
    const portNames = e => e.ports.length ? "ports: " + e.ports.map(p => p.name).join(", ") : "no ports";
    let h = E.cat.nodes.has("NavigateTo")
      ? group("Presets", [item("gps", "gps", "GPS waypoint", "dead reckoning: NavigateTo a lat/lon"),
          E.cat.nodes.has("HoldHeading")
            ? item("action", "hold", "Hold heading", "turn to a heading and stay there, where the last block left the boat (GUIDED)") : ""]) : "";
    const byTask = new Map();
    for (const s of (d.subtrees || []).filter(isTask)) byTask.set(s.task, (byTask.get(s.task) || []).concat(s));
    for (const [task, list] of [...byTask].sort((a, b) => a[0] - b[0]))
      h += group(TASKS[task] || `Task ${task}`, list.sort((a, b) => TIERS.indexOf(a.tier) - TIERS.indexOf(b.tier))
        .map(s => item("task", `task:${s.id}`, s.tier, `${s.id}: ${portNames(s)}`,
                       ` data-task="${esc(s.task)}" data-tier="${esc(s.tier)}"`, `${TASKS[task] || "Task " + task}, ${s.tier}`)));
    for (const [kind, name] of GROUPS)
      h += group(name, d.nodes.filter(n => n.kind === kind && !HIDDEN_BLOCKS.includes(n.id)).sort((a, b) => a.id.localeCompare(b.id))
        .map(n => item(kind, `node:${n.id}`, n.id, `${kind}: ${portNames(n)}`)));
    return h || '<span class="muted small">nothing matches</span>';
  }
  function missionOptions(keep) {
    const d = E.cat.data, group = (label, names, user) => names.length
      ? `<optgroup label="${label}">${names.map(n => `<option${user ? " data-user=1" : ""}>${esc(n)}</option>`).join("")}</optgroup>` : "";
    const yours = (d.user || []).map(u => u.name);
    const html = '<option value="">stored missions</option>' + group("Yours", yours, true) + group("Built in", d.builtins || [], false);
    $("ed-open").innerHTML = html;
    if (keep) $("ed-open").value = keep;
  }
  const templates = () => (E.cat && E.cat.data.templates) || [];
  function templateOptions() {
    const keep = $("ed-tpl").value;
    $("ed-tpl").innerHTML = templates().map(t => `<option value="${esc(t.name)}">${esc(t.name)}</option>`).join("");
    if (templates().some(t => t.name === keep)) $("ed-tpl").value = keep;
    $("ed-tpl-wrap").hidden = !templates().length;
    paintTemplateNote();
  }
  function paintTemplateNote() {
    const t = templates().find(x => x.name === $("ed-tpl").value);
    $("ed-tpl-desc").textContent = t ? t.description : "";
  }
  // Shifts every GPS waypoint by one offset so the pattern's centroid lands on the boat, keeping its shape.
  function centreOn(blocks, lat, lon) {
    const pts = [];
    for (const {block} of walk(blocks)) { const p = gpsPoint(block); if (p) pts.push([block, p]); }
    if (!pts.length) return 0;
    const c = pts.reduce((a, [, p]) => [a[0] + p.lat / pts.length, a[1] + p.lon / pts.length], [0, 0]);
    for (const [block, p] of pts) {
      block.ports.lat = (p.lat + lat - c[0]).toFixed(7);
      block.ports.lon = (p.lon + lon - c[1]).toFixed(7);
    }
    return pts.length;
  }
  async function fromTemplate() {                   // an unsaved draft: the operator drags the waypoints, then names and saves it
    const name = $("ed-tpl").value;
    if (!name) return deps.toast("no survey template to start from", false);
    if (E.dirty && E.blocks.length && !confirm("Replace the blocks being edited?")) return;
    const r = await deps.ask("mission_template", {name});
    if (!r.ok) return deps.toast(`${name}: ${r.msg}`, false);
    const blocks = r.data.blocks.map(adopt), boat = deps.boat() || {};
    const fresh = Number.isFinite(boat.lat) && Number.isFinite(boat.lon) && boat.age != null && boat.age <= POSE_FRESH_S;
    const moved = fresh ? centreOn(blocks, boat.lat, boat.lon) : 0;
    load(blocks, "", r.data.declare);
    E.dirty = true;                                 // a draft nobody has saved
    deps.toast(fresh ? `${name}: ${moved} waypoint(s) centred on the boat; drag them onto the course, then name and save it`
                     : `${name}: the boat's position is not fresh, so the waypoints stay where the template put them (near the venue); drag them onto the course, then name and save it`,
               fresh);
  }

  // ---- the declaration ------------------------------------------------------
  const tierWord = w => String(w == null ? "" : w).replace(/^TIER_/, "").toLowerCase();     // TIER_CORE -> core
  const byTask = f => Object.fromEntries(TASK_KEYS.map(k => [k, f(k)]));
  const wireTiers = () => byTask(k => "TIER_" + E.declare[k].toUpperCase());
  const words = wire => byTask(k => tierWord(wire[k]));            // the OCS's {task1: TIER_X} as {task1: x}
  function buildDecl() {
    $("ed-decl-sel").innerHTML = TASK_KEYS.map((k, i) =>
      ` <label>Task ${i + 1} <select data-decl="${k}">${DECL_TIERS.map(t => `<option>${t}</option>`).join("")}</select></label>`).join("");
  }
  function paintDecl() {
    for (const el of $("ed-decl-sel").querySelectorAll("select")) el.value = E.declare[el.dataset.decl];
    paintOcs();
  }
  function setDecl(task, tier) {
    remember();
    E.declare[task] = tier; E.declTouched = true;
    changed(false);
  }
  // The OCS's own word (status.ocs_run) beside the row. It speaks on changes, not on a timer, so its age is
  // shown and an old one is called old; with no ocs_run, or the OCS link down, nothing is claimed.
  function ocsTiers() {
    const r = deps.ocsRun && deps.ocsRun();
    return r && r.tiers ? words(r.tiers) : null;
  }
  const label = tiers => TASK_KEYS.map((k, i) => `T${i + 1} ${tiers[k] || "?"}`).join(", ");
  function tierDiffs(theirs) {
    return TASK_KEYS.map((k, i) => theirs[k] !== E.declare[k] ? `Task ${i + 1}: OCS ${theirs[k] || "?"}, row ${E.declare[k]}` : "").filter(Boolean);
  }
  function mismatch() {                             // for the Start dialog: null while the OCS's tiers are unknown
    const theirs = ocsTiers(), up = deps.ocsLinkUp ? deps.ocsLinkUp() : null;
    if (!theirs || up === false) return null;
    const d = tierDiffs(theirs);
    return d.length ? `the OCS will declare different tiers than the Declaration row (${d.join("; ")}). The OCS's are what RoboCommand gets: send the row first if it is the one you want.` : null;
  }
  function paintOcs() {
    const box = $("ed-ocs"), r = deps.ocsRun && deps.ocsRun(), up = deps.ocsLinkUp ? deps.ocsLinkUp() : null;
    const age = r && r.age != null ? Math.round(r.age) : null;
    if (!r || up === false) {
      box.className = "small warn";
      box.textContent = !r ? "OCS not heard: no run state received yet, so nothing is known about what it will declare."
                           : `OCS not heard: the boat's link to it is down (its last word was ${age} s ago: ${r.state}).`;
      return;
    }
    const theirs = ocsTiers(), diffs = theirs ? tierDiffs(theirs) : ["no tiers in its message"];
    const old = age != null && age > OCS_OLD_S;
    const reply = r.reply ? ` · reply to our last request${r.reply_age != null ? ` (${Math.round(r.reply_age)} s ago)` : ""}: ` +
      `<b class="${r.reply.ok ? "ok" : "bad"}">${r.reply.ok ? "accepted" : "refused"}</b> ${esc(r.reply.reason || "")}` : "";
    const cur = r.declared_tiers ? ` · last declaration: ${esc(label(words(r.declared_tiers)))}` : "";
    box.className = "small";
    box.innerHTML = `OCS <b>${esc(r.state)}</b> (heard ${age} s ago${old ? " — it only speaks on changes, so this may be old" : ""})` +
      (theirs ? ` · will declare ${esc(label(theirs))}${r.tiers_source ? ` (from ${esc(r.tiers_source)})` : ""}` : "") + cur + reply +
      ` · ${diffs.length ? `<b class="bad">&#10007; differs from the row: ${esc(diffs.join("; "))}</b>` : '<b class="ok">&#10003; matches the row</b>'}`;
  }
  async function sendTiers() {
    const r = await deps.ask("ocs_tiers", {tiers: wireTiers(), mission: E.name.trim()});
    deps.toast(`ocs_tiers: ${r.msg || (r.ok ? "sent" : "failed")}`, !!r.ok);
  }

  async function loadCatalogue() {
    const r = await deps.ask("mission_catalogue");
    const d = r.ok && r.data;
    E.cat = d ? {data: d, nodes: new Map(d.nodes.map(n => [n.id, n])),
                 tasks: new Map((d.subtrees || []).map(s => [s.id, s])), fix: d.fix_selector || {}} : null;
    E.why = d ? "" : r.msg || "the boat did not answer";
    $("ed-note-text").textContent = E.why;
    $("ed-note").hidden = !!d; $("ed-body").hidden = !d;
    if (!d) return;
    $("ed-pal-list").innerHTML = paletteHtml();
    missionOptions($("ed-open").value);
    templateOptions();
    syncDraft(); render(); validate();
  }

  // ---- checking and the boat ------------------------------------------------
  async function validate(force) {                  // {ok, data: {xml, problems}} of the latest edit, else null
    if (!E.cat) return null;
    const rev = E.rev;
    if (!force && !E.blocks.length) { E.problems = []; E.xml = ""; E.why = ""; $("ed-xml").textContent = ""; paintProblems(); return null; }
    const r = await deps.ask("mission_build", {blocks: E.blocks.map(plain), name: E.name.trim() || undefined, declare: E.declare});
    if (rev !== E.rev) return null;                 // edited meanwhile: its own check is on the way
    E.why = r.ok ? "" : r.msg || "no answer";
    E.problems = r.ok ? r.data.problems : [];
    E.xml = r.ok ? r.data.xml : "";
    $("ed-xml").textContent = E.xml;
    paintProblems();
    return r;
  }
  const isWarn = p => p.level === "warn";          // a warning never stops a save
  const errors = list => list.filter(p => !isWarn(p));
  const stored = () => ((E.cat && E.cat.data.user) || []).map(u => u.name);
  const said = (name, r, ok) => deps.toast(`${name}: ${r.msg || (r.ok ? ok : "failed")}`, !!r.ok);
  async function restocked(pick) { await loadCatalogue(); deps.stored(pick); }     // the stored missions changed
  async function save() {
    const name = E.name.trim();
    if (!name) return deps.toast("name the mission first", false);
    const r = await validate(true);
    if (!r) return deps.toast("the mission changed while it was being checked: save again", false);
    if (!r.ok) return deps.toast(r.msg, false);
    const bad = errors(r.data.problems).length;
    if (bad) return deps.toast(`not saved: ${bad} problem(s), shown on the blocks`, false);
    if (stored().includes(name) && !confirm(`Replace the saved mission ${name}?`)) return;
    const up = await deps.ask("mission_upload", {name, xml: r.data.xml});
    said(name, up, "saved");
    if (!up.ok) {
      if (up.data && up.data.problems) { E.problems = up.data.problems; paintProblems(); }
      return;
    }
    E.dirty = false;
    await restocked(name);
  }
  async function open(insert) {                     // replace the draft with a stored mission, or inline it
    const name = $("ed-open").value;
    if (!name) return deps.toast("choose a stored mission first", false);
    if (!insert && E.dirty && E.blocks.length && !confirm("Replace the blocks being edited?")) return;
    const r = await deps.ask("mission_open", insert ? {name, group: name} : {name});
    if (!r.ok) return deps.toast(`${name}: ${r.msg}`, false);
    const blocks = r.data.blocks.map(adopt);
    if (insert) edit(() => { const [list, at] = target(); list.splice(at, 0, ...blocks); E.sel = blocks[0] && blocks[0].uid; });
    else load(blocks, r.data.builtin ? `${name}_copy` : name, r.data.declare);      // a built-in cannot be overwritten
    if (r.data.notes.length) deps.toast(`${name}: ${r.data.notes.join("; ")}`);
  }
  async function erase() {
    const o = $("ed-open").selectedOptions[0];
    if (!o || !o.value) return deps.toast("choose a stored mission first", false);
    if (!o.dataset.user) return deps.toast(`${o.value} is built in: it cannot be deleted`, false);
    if (!confirm(`Delete the saved mission ${o.value} from the boat?`)) return;
    const r = await deps.ask("mission_delete", {name: o.value});
    said(o.value, r, "deleted");
    await restocked();
  }
  function clear() {
    if (E.blocks.length && E.dirty && !confirm("Discard the blocks being edited?")) return;
    load([], "");
  }

  // ---- the draft in this browser: a convenience, the editor works without it
  function persist() {
    try {
      if (E.blocks.length || E.name) localStorage.setItem(KEY, JSON.stringify({name: E.name, blocks: E.blocks.map(plain), declare: E.declare, touched: E.declTouched}));
      else localStorage.removeItem(KEY);
    } catch (e) { /* private window or blocked storage */ }
  }
  function restoreDraft() {
    try {
      const d = JSON.parse(localStorage.getItem(KEY) || "null");
      if (!d || !Array.isArray(d.blocks)) return;
      const blocks = d.blocks.map(adopt);           // before assigning: a bad draft changes nothing
      E.blocks = blocks; setName(String(d.name || "")); E.dirty = blocks.length > 0;
      if (d.declare) { E.declare = Object.assign(noTiers(), d.declare); E.declTouched = !!d.touched; }
    } catch (e) { /* unreadable draft: start empty */ }
  }

  // ---- dragging -------------------------------------------------------------
  // A press on a palette item or a block header becomes a drag after DRAG_PX, else it is a click.
  // A drop goes to the nearest slot: the gaps between the blocks of every list that may take it.
  function press(ev, src, label) {
    if (ev.button !== 0 || ev.target.closest("button")) return;
    if (document.activeElement && document.activeElement.blur) document.activeElement.blur();
    ev.preventDefault();                            // no text selection, no native drag of the label
    const from = [ev.clientX, ev.clientY];
    const onMove = e => {
      if (!drag && Math.hypot(e.clientX - from[0], e.clientY - from[1]) < DRAG_PX) return;
      if (!drag) startDrag(src, label);
      drag.last = e; track();
    };
    const onUp = e => {
      listeners.forEach(([type, fn]) => document.removeEventListener(type, fn));
      const released = e.type === "pointerup";      // not pointercancel
      if (!drag) return released ? (src.uid ? select(src.uid) : place(src.key, ...target())) : undefined;
      if (released) { drag.last = e; track(); }
      const slot = released ? drag.slot : null;
      endDrag();
      if (slot) src.uid ? move(src.uid, slot.list, slot.at) : place(src.key, slot.list, slot.at);
    };
    const listeners = [["pointermove", onMove], ["pointerup", onUp], ["pointercancel", onUp]];
    listeners.forEach(([type, fn]) => document.addEventListener(type, fn));
  }
  function startDrag(src, label) {
    const ghost = document.createElement("div"), line = document.createElement("div");
    ghost.className = "ed-ghost"; ghost.textContent = label;
    line.className = "ed-line"; line.hidden = true;
    document.body.append(ghost, line);
    document.body.classList.add("ed-dragging");
    drag = {src, ghost, line, slot: null, last: null, tick: setInterval(track, 60)};
  }
  function endDrag() {
    clearInterval(drag.tick);
    drag.ghost.remove(); drag.line.remove();
    document.body.classList.remove("ed-dragging");
    drag = null;
  }
  function track() {
    const e = drag && drag.last;
    if (!e) return;
    drag.ghost.style.transform = `translate(${e.clientX + 12}px, ${e.clientY + 10}px)`;
    const work = $("ed-work"), r = work.getBoundingClientRect();
    const over = e.clientX >= r.left && e.clientX <= r.right && e.clientY >= r.top && e.clientY <= r.bottom;
    if (over && e.clientY < r.top + SCROLL_PX) work.scrollTop -= 14;       // near an edge: scroll the blocks
    if (over && e.clientY > r.bottom - SCROLL_PX) work.scrollTop += 14;
    drag.slot = over ? nearest(e.clientX, e.clientY) : null;
    const s = drag.slot;
    drag.line.hidden = !s;
    if (s) Object.assign(drag.line.style, {left: s.x0 + "px", top: s.y - 1 + "px", width: s.x1 - s.x0 + "px"});
  }
  function mayTake(owner) {                         // may the dragged thing go into this block's list?
    if (!owner) return true;
    const moving = drag.src.uid && byUid.get(drag.src.uid);
    if (moving && (owner === moving || contains(moving, owner))) return false;     // not into itself
    return room(owner) + (moving && owner.children.includes(moving) ? 1 : 0) > 0;
  }
  function nearest(px, py) {
    let best = null, gap = Infinity;
    for (const el of $("ed-work").querySelectorAll(".blist")) {
      const owner = el.dataset.uid ? byUid.get(+el.dataset.uid) : null;
      if (!mayTake(owner)) continue;
      const r = el.getBoundingClientRect();
      if (px < r.left - 6 || px > r.right + 6) continue;
      const kids = [...el.children].filter(k => k.classList.contains("blk")).map(k => k.getBoundingClientRect());
      const ys = kids.length ? kids.map(k => k.top).concat(kids[kids.length - 1].bottom) : [r.top + r.height / 2];
      ys.forEach((y, at) => {
        const d = Math.abs(y - py);                 // a tie goes to the deeper list
        if (d < gap || (d === gap && best && r.left > best.x0))
          { gap = d; best = {list: owner ? owner.children : E.blocks, at, y, x0: r.left, x1: r.right}; }
      });
    }
    return best;
  }

  // ---- wiring ---------------------------------------------------------------
  const ownerOf = el => { const blk = el.closest(".blk"); return blk && byUid.get(+blk.dataset.uid); };
  function wire() {
    const work = $("ed-work");
    $("ed-pal").addEventListener("pointerdown", e => {
      const el = e.target.closest(".pal");
      if (el && !el.classList.contains("off")) press(e, {key: el.dataset.key}, el.dataset.label);
    });
    work.addEventListener("pointerdown", e => {
      const head = e.target.closest(".bh"), b = head && ownerOf(head);
      if (b) press(e, {uid: b.uid}, title(b));
    });
    work.addEventListener("click", e => {
      const b = ownerOf(e.target), act = e.target.closest("button");
      if (!b || !act) return;
      if (act.dataset.act === "del") remove(b.uid);
      else if (act.dataset.act === "dup") duplicate(b.uid);
      else if ("pick" in act.dataset) pick(b);
    });
    work.addEventListener("input", e => {
      const el = e.target, b = ownerOf(el);
      if (!b) return;
      if ("label" in el.dataset) b.name = el.value;
      else if (el.dataset.port) b.ports[el.dataset.port] = el.value;
      else return;
      changed(false);
    });
    work.addEventListener("focusin", e => { if (e.target.matches("[data-port],[data-label]")) pre = snapshot(); });
    work.addEventListener("focusout", () => { if (pre && snapshot() !== pre) remember(pre); pre = null; });
    $("ed-name").addEventListener("input", e => { E.name = e.target.value; changed(false); });
    $("ed-find").addEventListener("input", () => { $("ed-pal-list").innerHTML = paletteHtml(); paintTiers(); });
    $("ed-save").onclick = save;
    $("ed-open-go").onclick = () => open(false);
    $("ed-insert").onclick = () => open(true);
    $("ed-delete").onclick = erase;
    $("ed-undo").onclick = undo; $("ed-redo").onclick = redo; $("ed-clear").onclick = clear;
    $("ed-retry").onclick = loadCatalogue;
    $("ed-tpl-go").onclick = fromTemplate;
    $("ed-tpl").onchange = paintTemplateNote;
    $("ed-tiers-send").onclick = sendTiers;
    $("ed-decl-sel").addEventListener("change", e => { if (e.target.dataset.decl) setDecl(e.target.dataset.decl, e.target.value); });
    $("ed-wp").onclick = () => {
      if (CrsdMap.clickMode() === "waypoints") return CrsdMap.setClickMode("off");
      if (!CrsdMap.hasOrigin()) return deps.toast("the map has no position yet: type the first waypoint's lat/lon, then click", false);
      CrsdMap.setClickMode("waypoints");
    };
    CrsdMap.onModeChange(mode => {
      $("ed-wp").classList.toggle("primary", mode === "waypoints");
      $("ed-wp").textContent = mode === "waypoints" ? "Done adding waypoints" : "Add waypoints on map";
    });
    CrsdMap.onWaypoint(addWaypoint);
    CrsdMap.onDraftPick(i => { const hit = [...E.gps].find(([, n]) => n === i + 1); if (hit) { E.sel = null; select(hit[0]); } });
    document.addEventListener("keydown", e => {
      if (!$("t-mission").classList.contains("on") || e.target.closest("input, select, textarea")) return;
      if ((e.key === "Delete" || e.key === "Backspace") && E.sel) { e.preventDefault(); remove(E.sel); }
      else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") { e.preventDefault(); e.shiftKey ? redo() : undo(); }
      else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "y") { e.preventDefault(); redo(); }
    });
  }

  // deps: {ask(op, fields) -> result, toast(msg, ok), esc(text), stored(name?) once the stored missions changed}
  function init(d) {
    deps = d;
    buildDecl();
    wire();
    restoreDraft();
    render();
  }
  function enter() { loadCatalogue(); }
  function leave() {
    if (["pick", "waypoints"].includes(CrsdMap.clickMode())) CrsdMap.setClickMode("off");
  }
  return {init, enter, leave, status: paintOcs, tierMismatch: mismatch};
})();
