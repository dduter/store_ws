"use strict";

const byId = (id) => document.getElementById(id);
const neutral = {dx_px: 0, dy_px: 0, dz_px: 0};
const stopCommand = {...neutral, stop: true};
const statusColors = {OK: "ok", WARN: "warn", ERROR: "error", STALE: "", NORMAL: "ok", EXPIRED: "error", REAR_FACING: "warn"};
let toastTimer, jogTimer, gripperTimer, pointer, inFlight, gripperInFlight, pendingStop, graphKey, logKey, inventoryKey;
let gripperGen = 0;
let remoteEnabled = false;
let remoteToken = null;

function element(tag, value, className) {
  const node = document.createElement(tag);
  if (value != null) node.textContent = typeof value === "object" ? JSON.stringify(value) : value;
  if (className) node.className = className;
  return node;
}

function setBadge(id, text, tone = "") {
  const badge = byId(id);
  badge.textContent = text;
  badge.className = `badge ${tone}`;
}

function toast(text, error = false) {
  const output = byId("toast");
  output.textContent = text;
  output.className = error ? "error" : "ok";
  output.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { output.hidden = true; }, 3500);
}

async function post(path, body, keepalive = false, extraHeaders = {}) {
  let response;
  try {
    response = await fetch(path, {
      method: "POST", headers: {"Content-Type": "application/json", ...extraHeaders},
      body: JSON.stringify(body), keepalive,
    });
  } catch {
    throw new Error("서버에 연결할 수 없습니다.");
  }
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.detail || "요청 처리에 실패했습니다.");
  return data;
}

function renderFeed(id, rows) {
  byId(id).replaceChildren(...(rows.length ? rows.map((row) => element("li", row)) : [element("li", "기록 없음", "empty")]));
}

function renderTable(id, rows, keys) {
  const table = byId(id);
  table.replaceChildren();
  for (const row of rows) {
    const tr = table.insertRow();
    for (const key of keys) {
      const value = row?.[key] ?? "—";
      const cell = tr.insertCell();
      if (key === "status") cell.append(element("span", value, `badge ${statusColors[value] || ""}`));
      else cell.append(element("span", value));
    }
  }
  if (!rows.length) {
    const cell = table.insertRow().insertCell();
    cell.colSpan = keys.length;
    cell.textContent = "기록 없음";
    cell.className = "empty";
  }
}

function renderHealth(nodes) {
  const cards = Object.values(nodes).sort((a, b) => a.node_name.localeCompare(b.node_name)).map((node) => {
    const state = node.stale ? "STALE" : !node.alive ? "ERROR" : node.state;
    const card = element("article");
    const heading = element("div");
    heading.append(element("strong", node.node_name), element("span", state, `badge ${statusColors[state] || ""}`));
    card.append(heading, element("p", node.message || "—"), element("p", `수신 후 ${node.age_sec.toFixed(1)}초`));
    return card;
  });
  byId("health").replaceChildren(...(cards.length ? cards : [element("p", "하트비트 수신 대기", "empty")]));
}

function svgElement(tag, attributes, label) {
  const el = document.createElementNS("http://www.w3.org/2000/svg", tag);
  for (const [key, value] of Object.entries(attributes)) el.setAttribute(key, value);
  if (label != null) el.textContent = label;
  return el;
}

function renderGraph(graph) {
  const stats = byId("kg-stats");
  stats.replaceChildren(...Object.entries({
    catalog_count: "카탈로그", instance_count: "상품 인스턴스",
    container_count: "컨테이너", owlrl_triples: "OWL RL 트리플",
  }).map(([key, label]) => element("span", `${label} ${graph.stats[key] ?? 0}`, "badge info")));

  const groups = ["catalog", "instance", "container"].map((kind) => graph.nodes.filter((node) => node?.kind === kind));
  const height = 62 + Math.max(...groups.map((group) => group.length), 1) * 72;
  const svg = svgElement("svg", {viewBox: `0 0 960 ${height}`, role: "img", "aria-label": "상품 관계 그래프"});
  const defs = svgElement("defs", {});
  const arrow = svgElement("marker", {id: "arrow", viewBox: "0 0 10 10", refX: 9, refY: 5, markerWidth: 7, markerHeight: 7, orient: "auto"});
  arrow.append(svgElement("path", {d: "M0 0L10 5L0 10Z", fill: "#7394b9"}));
  defs.append(arrow);
  svg.append(defs);
  const positions = new Map();
  const nodes = svgElement("g", {});
  for (const [column, group] of groups.entries()) {
    const x = 24 + column * 320;
    svg.append(svgElement("text", {x, y: 27}, ["카탈로그", "상품 인스턴스", "컨테이너"][column]));
    if (!group.length) svg.append(svgElement("text", {x, y: 70}, "데이터 없음"));
    for (const [index, node] of group.entries()) {
      const y = 46 + index * 72;
      positions.set(node.id, {x, y});
      const label = String(node.label ?? node.id ?? "");
      const status = node.status || "";
      const color = {EXPIRED: "#ff818c", REAR_FACING: "#ffc16e", NORMAL: "#6aebac"}[status] || "#7394b9";
      const g = svgElement("g", {tabindex: 0});
      g.append(svgElement("title", {}, [label, node.id, ...(Array.isArray(node.entailed_types) ? node.entailed_types : [])].filter(Boolean).join("\n")));
      g.append(svgElement("rect", {x, y, width: 220, height: 54, rx: 8, fill: "#152436", stroke: color}));
      g.append(svgElement("text", {x: x + 10, y: y + (status ? 20 : 32), class: "node-label"}, label.length > 21 ? label.slice(0, 20) + "…" : label));
      if (status) {
        g.append(svgElement("rect", {x: x + 10, y: y + 27, width: 110, height: 19, rx: 9, fill: "#0c1521", stroke: color}));
        g.append(svgElement("text", {x: x + 18, y: y + 41, style: `fill:${color}`}, status));
      }
      nodes.append(g);
    }
  }
  for (const link of graph.links) {
    const source = positions.get(link?.source), target = positions.get(link?.target);
    if (!source || !target) continue;
    const right = target.x > source.x;
    const fromX = source.x + (right ? 220 : 0), toX = target.x + (right ? 0 : 220);
    const mid = (fromX + toX) / 2, fromY = source.y + 27, toY = target.y + 27;
    svg.append(svgElement("path", {
      d: `M${fromX} ${fromY} C${mid} ${fromY} ${mid} ${toY} ${toX} ${toY}`,
      stroke: "#7394b9", fill: "none", "marker-end": "url(#arrow)",
    }));
    svg.append(svgElement("text", {x: mid, y: (fromY + toY) / 2 - 4, "text-anchor": "middle", class: "edge-label"}, link.relation));
  }
  svg.append(nodes);
  byId("graph").replaceChildren(svg);
}

function render(state) {
  remoteEnabled = state.remote_override;
  if (!remoteEnabled) remoteToken = null;
  setBadge("mode", state.mode, "info");
  setBadge("estop-state", state.emergency_stop ? "비상 정지 중" : "비상 정지 해제됨", state.emergency_stop ? "error" : "ok");
  setBadge(
    "remote-state",
    !remoteEnabled ? "비활성" : remoteToken ? "활성" : "활성 (웹 인증 필요)",
    !remoteEnabled ? "" : remoteToken ? "ok" : "warn",
  );
  if (!remoteEnabled || !remoteToken || state.emergency_stop) stopJog(false, false);
  byId("controls").disabled = !remoteEnabled || !remoteToken || state.emergency_stop;
  for (const [name, camera] of Object.entries(state.cameras)) {
    setBadge(name, camera.live ? `${camera.fps.toFixed(1)} FPS` : "대기", camera.live ? "ok" : "");
  }
  renderHealth(state.node_health);
  const nextLog = JSON.stringify([state.agent_log, state.ui_events]);
  if (nextLog !== logKey) {
    logKey = nextLog;
    renderFeed("agent-log", [...state.agent_log].reverse());
    renderFeed("events", [...state.ui_events].reverse().map((entry) =>
      `${new Date(entry.time).toLocaleTimeString("ko-KR")} · ${entry.message}`));
  }
  const nextInventory = JSON.stringify([state.inventory, state.disposal_history]);
  if (nextInventory !== inventoryKey) {
    inventoryKey = nextInventory;
    renderTable("inventory-rows", state.inventory, ["track_id", "name", "status", "depth_layer"]);
    renderTable("disposal-rows", state.disposal_history, ["track_id", "reason"]);
  }
  const nextGraph = JSON.stringify(state.kg_graph);
  if (nextGraph !== graphKey) {
    graphKey = nextGraph;
    renderGraph(state.kg_graph);
  }
}

function connect() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  const socket = new WebSocket(`${protocol}//${location.host}/ws/state`);
  socket.onopen = () => setBadge("socket", "실시간 연결", "ok");
  socket.onmessage = (event) => render(JSON.parse(event.data));
  socket.onerror = () => socket.close();
  socket.onclose = () => {
    stopJog(false, Boolean(remoteToken), true);
    byId("controls").disabled = true;
    setBadge("socket", "연결 끊김 · 재연결 중", "warn");
    setTimeout(connect, 1200);
  };
}

byId("command-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const input = byId("command");
  const text = input.value;
  try {
    await post("/api/text_cmd", {text});
    if (input.value === text) input.value = "";
    toast("텍스트 명령을 보냈습니다.");
  } catch (error) { toast(error.message, true); }
});
byId("presets").addEventListener("click", (event) => {
  if (event.target.tagName !== "BUTTON") return;
  byId("command").value = event.target.textContent;
  byId("command").focus();
});

for (const [id, stop] of [["estop", true], ["resume", false]]) {
  byId(id).addEventListener("click", async () => {
    if (stop) stopJog();
    try {
      await post("/api/emergency_stop", {stop});
      toast(stop ? "비상 정지를 요청했습니다." : "비상 정지 해제를 요청했습니다.");
    } catch (error) { toast(error.message, true); }
  });
}

byId("remote-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const input = byId("admin-key");
  const admin_key = input.value;
  input.value = "";
  const enable_remote = event.submitter?.value !== "off";
  if (!enable_remote) stopJog();
  const buttons = [...event.currentTarget.querySelectorAll("button")];
  buttons.forEach((button) => { button.disabled = true; });
  try {
    const response = await post("/api/remote/verify", {admin_key, enable_remote});
    if (response.granted) remoteToken = enable_remote ? (response.remote_token || null) : null;
    byId("controls").disabled = !remoteEnabled || !remoteToken || byId("estop-state").classList.contains("error");
    setBadge(
      "remote-state",
      !remoteEnabled ? "비활성" : remoteToken ? "활성" : "활성 (웹 인증 필요)",
      !remoteEnabled ? "" : remoteToken ? "ok" : "warn",
    );
    toast(response.reason || (response.granted ? "원격 모드 요청이 승인되었습니다." : "원격 모드 요청이 거절되었습니다."), !response.granted);
  } catch (error) { toast(error.message, true); }
  finally { buttons.forEach((button) => { button.disabled = false; }); }
});

const hand = (command) => post(
  "/api/remote/hand_cmd", command, true,
  remoteToken ? {"X-Remote-Token": remoteToken} : {},
);

function stopGripperBurst() {
  gripperGen += 1;
  clearInterval(gripperTimer);
  gripperTimer = null;
}

function stopJog(force = false, send = true, silent = false) {
  stopGripperBurst();
  if (pointer == null && !force) return;
  pointer = null;
  clearInterval(jogTimer);
  if (!send || pendingStop) return;
  const movement = inFlight;
  const sendStop = () => hand(stopCommand).catch((error) => { if (!silent) toast(error.message, true); });
  // 정지를 즉시 전송하고 전송 중이던 이동이 뒤늦게 도착한 경우에도 다시 정지합니다.
  const immediate = sendStop();
  pendingStop = (movement ? Promise.allSettled([immediate, movement]).then(sendStop) : immediate)
    .finally(() => { pendingStop = null; });
}

for (const button of document.querySelectorAll("[data-vector]")) {
  const vector = button.dataset.vector.split(",").map(Number);
  button.addEventListener("pointerdown", (event) => {
    if (event.button !== 0 || pointer != null || pendingStop || byId("controls").disabled) return;
    event.preventDefault();
    stopGripperBurst();
    pointer = event.pointerId;
    button.setPointerCapture(pointer);
    const tick = () => {
      if (inFlight) return;
      const [dx_px, dy_px, dz_px] = vector.map((component) => component * Number(byId("step").value));
      inFlight = hand({dx_px, dy_px, dz_px, stop: false})
        .catch((error) => { toast(error.message, true); stopJog(); })
        .finally(() => { inFlight = null; });
    };
    tick();
    jogTimer = setInterval(tick, 100);
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture", "pointerleave", "mouseleave"]) {
    button.addEventListener(type, () => stopJog());
  }
  // 포인터를 캡처한 경우에도 버튼의 실제 경계 밖으로 나가면 정지합니다.
  button.addEventListener("pointermove", (event) => {
    const box = button.getBoundingClientRect();
    if (event.pointerId === pointer && (event.clientX < box.left || event.clientX > box.right ||
        event.clientY < box.top || event.clientY > box.bottom)) stopJog();
  });
}
window.addEventListener("pointerup", () => stopJog());
window.addEventListener("pointercancel", () => stopJog());
window.addEventListener("blur", () => stopJog());
window.addEventListener("pagehide", () => stopJog());
document.addEventListener("visibilitychange", () => { if (document.hidden) stopJog(); });
byId("step").addEventListener("input", (event) => { byId("step-value").value = `${event.target.value} px`; });
byId("stop").addEventListener("click", () => stopJog(true));
for (const [id, gripper_closed] of [["open", false], ["close", true]]) {
  byId(id).addEventListener("click", () => {
    if (pointer != null || pendingStop || byId("controls").disabled) return;
    stopGripperBurst();
    const gen = gripperGen;
    let remaining = 7;
    const pulse = () => {
      if (gen !== gripperGen || !remoteEnabled || !remoteToken || byId("controls").disabled || remaining <= 0) {
        if (gen === gripperGen) stopGripperBurst();
        return;
      }
      if (gripperInFlight) return;
      remaining -= 1;
      gripperInFlight = hand({...neutral, gripper_closed, stop: false})
        .catch((error) => {
          if (gen === gripperGen) {
            stopGripperBurst();
            toast(error.message, true);
          }
        })
        .finally(() => { gripperInFlight = null; });
    };
    // remote_node의 gripper_debounce_sec(0.5s)를 안정적으로 통과하도록 600ms 동안 연속 전송합니다.
    pulse();
    gripperTimer = setInterval(pulse, 100);
    toast(gripper_closed ? "그리퍼 닫기 명령을 보냈습니다." : "그리퍼 열기 명령을 보냈습니다.");
  });
}

const tabs = ["inventory", "disposal"];
function selectTab(name) {
  for (const tab of tabs) {
    byId(tab).hidden = tab !== name;
    byId(`${tab}-tab`).setAttribute("aria-selected", String(tab === name));
    byId(`${tab}-tab`).tabIndex = tab === name ? 0 : -1;
  }
}
for (const [index, name] of tabs.entries()) {
  const tab = byId(`${name}-tab`);
  tab.addEventListener("click", () => selectTab(name));
  tab.addEventListener("keydown", (event) => {
    if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
    event.preventDefault();
    const next = tabs[event.key === "Home" ? 0 : event.key === "End" ? tabs.length - 1 : 1 - index];
    selectTab(next);
    byId(`${next}-tab`).focus();
  });
}
connect();
