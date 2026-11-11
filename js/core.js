/* =====================================================================
   БАРЛАУШЫ — общее ядро сайта
   Данные: ровер --LoRa--> база --Wi-Fi--> MQTT-брокер --WSS--> этот сайт
   Формат строки (одинаковый для USB, MQTT и демо):
   TEL,seq,STATE,dist_cm,ch4_ppm,co_ppm,q135_dev,q2_dev,i1,i2,i3,i4,overall,front_cm,pt,ack,bat_dV,warm_s,rssi,snr
   ===================================================================== */
"use strict";

const B = (() => {
  const CFG = window.BARLAUSHY_CONFIG;

  const SENSORS = [
    { key: "ch4",  name: "Метан",                chip: "MQ-4",   color: "var(--s1)", unit: "ppm" },
    { key: "co",   name: "Угарный газ (CO)",     chip: "MQ-7",   color: "var(--s2)", unit: "ppm" },
    { key: "q135", name: "Канализационные газы", chip: "MQ-135", color: "var(--s3)", unit: "%"   },
    { key: "q2",   name: "Пропан / горючие",     chip: "MQ-2",   color: "var(--s4)", unit: "%"   },
  ];
  const STATE_RU = {
    WARM: "Прогрев датчиков", CAL: "Калибровка (чистый воздух)", READY: "Готов к миссии",
    DRIVE: "Движение", SAMPLE: "Замер", AVOID: "Объезд препятствия",
    ALARM: "ТРЕВОГА: опасный газ", RETREAT: "Отход из опасной зоны", HOLD: "Ждёт команды оператора",
    MANUAL: "Ручное управление", DONE: "Миссия завершена",
  };
  const STATUS = [
    { cls: "good",     color: "var(--good)",     icon: "✔", label: "норма" },
    { cls: "warning",  color: "var(--warning)",  icon: "▲", label: "осторожно" },
    { cls: "critical", color: "var(--critical)", icon: "⛔", label: "опасно" },
  ];
  const fmt = n => Math.round(n).toLocaleString("ru-RU");

  // ------------------------- состояние -------------------------
  const S = {
    mode: "live",          // live | demo
    mqtt: "idle",          // idle | connecting | connected | error | nolib
    base: "unknown",       // online | offline | norover | unknown
    last: null,            // последняя телеметрия
    lastFresh: 0,          // когда пришёл НЕ сохранённый (живой) пакет
    retainedOnly: false,   // видим только сохранённые данные
    points: [],            // точки замеров текущей миссии
    missionStart: null,
  };
  const listeners = [];
  const emit = () => listeners.forEach(f => f(S));

  function parseTel(line) {
    const f = line.trim().split(",");
    if (f[0] !== "TEL" || f.length < 20) return null;
    return {
      seq: +f[1], state: f[2], dist: +f[3] / 100,
      ch4: +f[4], co: +f[5], q135: +f[6], q2: +f[7],
      idx: [+f[8], +f[9], +f[10], +f[11]],
      overall: +f[12], front: +f[13], pt: +f[14] === 1, ack: +f[15],
      bat: +f[16] / 10, warm: +f[17], rssi: +f[f.length - 2], snr: +f[f.length - 1],
      time: new Date(),
    };
  }

  function onTel(t, retained) {
    const prev = S.last;
    if (t.state === "SAMPLE" && t.dist === 0 && prev && ["READY", "DONE", "HOLD"].includes(prev.state)) {
      S.points = []; S.missionStart = t.time;
    }
    if (t.pt && !S.points.some(p => p.seq === t.seq)) S.points.push(t);
    S.last = t;
    if (!retained) { S.lastFresh = Date.now(); S.retainedOnly = false; }
    else if (!S.lastFresh) S.retainedOnly = true;
    emit();
  }

  function loadMission(text) {
    const pts = text.split("\n").map(parseTel).filter(Boolean);
    S.points = pts;
    if (pts.length) S.missionStart = pts[0].time;
    emit();
  }

  // ------------------------- MQTT -------------------------
  let client = null;
  function connectLive() {
    stopDemo();
    S.mode = "live";
    if (!window.mqtt) { S.mqtt = "nolib"; emit(); return; }
    S.mqtt = "connecting"; emit();
    client = window.mqtt.connect(CFG.mqttUrl, {
      clientId: "barlaushy-web-" + Math.random().toString(16).slice(2, 10),
      clean: true, reconnectPeriod: 4000, connectTimeout: 10000,
    });
    client.on("connect", () => { S.mqtt = "connected"; client.subscribe(CFG.topicRoot + "/#"); emit(); });
    client.on("reconnect", () => { S.mqtt = "connecting"; emit(); });
    client.on("error", () => { S.mqtt = "error"; emit(); });
    client.on("offline", () => { S.mqtt = "error"; emit(); });
    client.on("message", (topic, payload, packet) => {
      if (S.mode !== "live") return;
      const msg = payload.toString();
      const retained = !!(packet && packet.retain);
      if (topic.endsWith("/tel")) { const t = parseTel(msg); if (t) onTel(t, retained); }
      else if (topic.endsWith("/mission")) loadMission(msg === "EMPTY" ? "" : msg);
      else if (topic.endsWith("/status")) { S.base = msg; emit(); }
    });
  }

  // ------------------------- ДЕМО -------------------------
  // Симулятор: тоннель 10 м, препятствие на 3,2 м, газовый карман на ~6,3 м
  class RoverSim {
    constructor(emitLine) {
      this.emitLine = emitLine; this.state = "WARM"; this.t0 = 0; this.now = 0;
      this.dist = 0; this.target = 0.5; this.seq = 0; this.idx = [0, 0, 0, 0];
      this.lastTel = 0; this.avoided = false;
      this.timer = setInterval(() => this.tick(100), 100);
    }
    stop() { clearInterval(this.timer); }
    enter(s) { this.state = s; this.t0 = this.now; this.send(false); }
    field(x) {
      const g = (c, w, a) => a * Math.exp(-(((x - c) / w) ** 2));
      return [1 + g(6.3, 0.9, 140), 4 + g(6.0, 1.2, 38), 12 + g(6.4, 1.6, 30), 5 + g(6.5, 1.0, 120)];
    }
    front() { return this.dist < 3.2 && !this.avoided ? Math.round((3.2 - this.dist) * 100 + 20) : 400; }
    get overall() { const m = Math.max(...this.idx); return m >= 100 ? 2 : m >= 50 ? 1 : 0; }
    send(pt) {
      const i = this.idx.map(v => Math.round(Math.min(150, Math.max(0, v))));
      const ppm = (v, w, d) => v < 50 ? v / 50 * w : w + (v - 50) / 50 * (d - w);
      const dev = v => v < 50 ? v / 50 * 35 : Math.min(100, 35 + (v - 50) / 50 * 25);
      const cal = !["WARM", "CAL"].includes(this.state);
      const vals = cal ? [Math.round(ppm(i[0], 5000, 10000)), Math.round(ppm(i[1], 25, 50)), Math.round(dev(i[2])), Math.round(dev(i[3]))] : [0, 0, 0, 0];
      const warm = this.state === "WARM" ? Math.max(0, Math.ceil((5000 - (this.now - this.t0)) / 1000)) : 0;
      this.seq++;
      this.emitLine(`TEL,${this.seq},${this.state},${Math.round(this.dist * 100)},${vals.join(",")},${(cal ? i : [0, 0, 0, 0]).join(",")},` +
        `${cal ? this.overall : 0},${this.front()},${pt ? 1 : 0},0,74,${warm},${Math.round(-52 - this.dist * 3 - Math.random() * 4)},${(8 + Math.random() * 2).toFixed(1)}`);
      this.lastTel = this.now;
    }
    tick(dt) {
      this.now += dt;
      const inS = this.now - this.t0, v = 0.3 * dt / 1000;
      const f = this.field(this.dist);
      this.idx = this.idx.map((x, k) => x + (f[k] + (Math.random() - 0.5) * 3 - x) * 0.12);
      const ov = this.overall;
      switch (this.state) {
        case "WARM": if (inS > 5000) this.enter("CAL"); break;
        case "CAL": if (inS > 3000) this.enter("READY"); break;
        case "READY": if (inS > 2500) { this.dist = 0; this.target = 0.5; this.avoided = false; this.enter("SAMPLE"); } break; // автозапуск
        case "DRIVE":
          if (ov === 2) { this.enter("ALARM"); this.send(true); break; }
          if (this.front() < 25) { this.enter("AVOID"); break; }
          this.dist += ov === 1 ? v * 0.66 : v;
          if (this.dist >= this.target) this.enter("SAMPLE");
          break;
        case "SAMPLE":
          if (ov === 2) { this.enter("ALARM"); this.send(true); break; }
          if (inS > 2500) {
            this.send(true);
            if (this.dist >= 10) { this.enter("DONE"); break; }
            this.target = this.dist + (ov === 1 ? 0.25 : 0.5);
            this.enter("DRIVE");
          }
          break;
        case "AVOID": if (inS > 2600) { this.avoided = true; this.enter("DRIVE"); } break;
        case "ALARM": if (inS > 4000) this.enter("RETREAT"); break;
        case "RETREAT": this.dist = Math.max(0, this.dist - v); if (inS > 1500) this.enter("HOLD"); break;
        case "HOLD": case "DONE": if (inS > 9000) this.enter("READY"); break;            // демо по кругу
      }
      if (this.now - this.lastTel >= 1000) this.send(false);
    }
  }
  let sim = null;
  function startDemo() {
    if (client) { try { client.end(true); } catch (e) {} client = null; }
    S.mode = "demo"; S.last = null; S.points = []; S.lastFresh = 0; S.retainedOnly = false;
    sim = new RoverSim(line => { const t = parseTel(line); if (t) onTel(t, false); });
    emit();
  }
  function stopDemo() { if (sim) { sim.stop(); sim = null; } }

  // ------------------------- связь -------------------------
  function link() {
    const age = S.lastFresh ? (Date.now() - S.lastFresh) / 1000 : null;
    if (S.mode === "demo") return { live: true, color: "var(--yellow)", badge: "ДЕМО", text: "демо-режим · симуляция тоннеля" };
    if (S.mqtt === "nolib") return { live: false, color: "var(--critical)", badge: "OFF", text: "нет интернета — не загрузилась библиотека MQTT" };
    if (S.mqtt !== "connected") return { live: false, color: "var(--warning)", badge: "OFF", text: "подключение к облаку…" };
    if (age !== null && age < 6) return { live: true, color: "var(--good)", badge: "LIVE", text: `связь с роботом · ${S.last.rssi} dBm` };
    if (S.base === "offline") return { live: false, color: "var(--critical)", badge: "OFF", text: S.last ? "база не в сети · последние сохранённые данные" : "база не в сети" };
    if (S.base === "norover") return { live: false, color: "var(--critical)", badge: "OFF", text: "база в сети, нет сигнала от робота" };
    if (S.last && age === null) return { live: false, color: "var(--warning)", badge: "OFF", text: "последние сохранённые данные" };
    if (age !== null) return { live: false, color: "var(--critical)", badge: "OFF", text: `нет пакетов ${Math.round(age)} с` };
    return { live: false, color: "var(--warning)", badge: "OFF", text: "облако на связи · ждём робота…" };
  }

  // ------------------------- вердикт -------------------------
  function verdict() {
    const t = S.last;
    if (!t) return { lv: -1, icon: "⏻", title: "Нет данных", sub: "Робот пока не передал ни одного пакета" };
    if (t.state === "WARM") return { lv: -1, icon: "♨", title: "Прогрев датчиков", sub: `Осталось ≈${t.warm} с` };
    if (t.state === "CAL") return { lv: -1, icon: "⟲", title: "Калибровка", sub: "Робот запоминает чистый воздух" };
    const worst = S.points.reduce((a, p) => (!a || p.overall > a.overall || (p.overall === a.overall && Math.max(...p.idx) > Math.max(...a.idx))) ? p : a, null);
    const lv = Math.max(t.overall, worst ? worst.overall : 0);
    const where = worst && worst.overall === lv ? worst : t;
    const gas = SENSORS[where.idx.indexOf(Math.max(...where.idx))].name.toLowerCase();
    if (lv === 2) return { lv, icon: "⛔", title: "ВХОД ЗАПРЕЩЁН", sub: `Опасная концентрация на ≈${where.dist.toFixed(1)} м: ${gas}` };
    if (lv === 1) return { lv, icon: "▲", title: "ОСТОРОЖНО", sub: `Повышенный уровень на ≈${where.dist.toFixed(1)} м: ${gas}` };
    if (t.state === "DONE") return { lv: 0, icon: "✅", title: "ВХОД РАЗРЕШЁН", sub: `Проверено ≈${t.dist.toFixed(1)} м — опасных газов нет` };
    if (t.state === "READY") return { lv: -1, icon: "●", title: "Готов к миссии", sub: "Робот стоит у входа" };
    return { lv: 0, icon: "🔍", title: "Идёт проверка", sub: `Пройдено ≈${t.dist.toFixed(1)} м — опасности пока нет` };
  }

  // ------------------------- отрисовка -------------------------
  const $ = id => document.getElementById(id);
  const NS = "http://www.w3.org/2000/svg";
  function el(tag, attrs, parent, text) {
    const e = document.createElementNS(NS, tag);
    for (const k in attrs) e.setAttribute(k, attrs[k]);
    if (text !== undefined) e.textContent = text;
    if (parent) parent.appendChild(e);
    return e;
  }

  function renderVerdict(id) {
    const box = $(id); if (!box) return;
    const v = verdict(), t = S.last;
    box.className = "verdict" + (v.lv >= 0 ? " " + STATUS[v.lv].cls : "") + (t && t.state === "ALARM" ? " flash" : "");
    box.innerHTML = `<div class="icon">${v.icon}</div><div><div class="title">${v.title}</div><div class="sub">${v.sub}</div></div>` +
      `<div class="meta">пройдено<b>${t ? "≈" + t.dist.toFixed(1) + " м" : "—"}</b></div>`;
  }

  function renderTiles(id) {
    const box = $(id); if (!box) return;
    if (!box.dataset.built) {
      box.innerHTML = SENSORS.map((s, i) => `
        <div class="tile"><div class="head"><span class="swatch" style="background:${s.color}"></span>${s.name}</div>
        <div class="val" data-v="${i}">—</div>
        <div class="meter"><i data-m="${i}"></i><u style="left:33.3%"></u><u style="left:66.6%"></u></div>
        <div class="status" data-s="${i}">${s.chip}</div></div>`).join("");
      box.dataset.built = 1;
    }
    const t = S.last, cal = t && !["WARM", "CAL"].includes(t.state);
    SENSORS.forEach((s, i) => {
      const idx = t ? t.idx[i] : 0, st = STATUS[idx >= 100 ? 2 : idx >= 50 ? 1 : 0];
      box.querySelector(`[data-v="${i}"]`).innerHTML = !cal ? "—" :
        s.unit === "ppm" ? `≈${fmt(t[s.key])}<small> ppm</small>` : `${fmt(t[s.key])}<small> % сигнал</small>`;
      const m = box.querySelector(`[data-m="${i}"]`);
      m.style.width = Math.min(idx, 150) / 150 * 100 + "%"; m.style.background = st.color;
      box.querySelector(`[data-s="${i}"]`).innerHTML = cal
        ? `<span style="color:${st.color}">${st.icon}</span> ${st.label} · ${Math.round(idx)}% порога · ${s.chip}` : s.chip;
    });
  }

  const tip = () => $("tip");
  function showTip(evt, html) {
    const t = tip(); if (!t) return;
    t.innerHTML = html; t.style.display = "block";
    t.style.left = Math.min(evt.clientX + 14, innerWidth - t.offsetWidth - 8) + "px";
    t.style.top = Math.min(evt.clientY + 14, innerHeight - t.offsetHeight - 8) + "px";
  }
  function hideTip() { const t = tip(); if (t) t.style.display = "none"; }
  function pointTip(p) {
    return `<div style="margin-bottom:4px"><b>≈${p.dist.toFixed(1)} м</b></div>` +
      SENSORS.map((s, i) => `<div class="row"><span class="swatch" style="background:${s.color}"></span>${s.name}<b>${Math.round(p.idx[i])}%</b></div>`).join("") +
      `<div class="row" style="margin-top:4px"><span style="color:${STATUS[p.overall].color}">${STATUS[p.overall].icon}</span>${STATUS[p.overall].label}</div>`;
  }
  const maxDist = () => Math.ceil(Math.max(10, S.last ? S.last.dist : 0, ...S.points.map(p => p.dist)));

  function renderTunnel(id) {
    const svg = $(id); if (!svg) return;
    svg.innerHTML = "";
    const L = 46, R = 960, top = 42, h = 46, D = maxDist(), X = d => L + (R - L) * d / D;
    el("rect", { x: L - 6, y: top - 6, width: R - L + 12, height: h + 12, rx: 14, fill: "#2a2a24", stroke: "var(--border)" }, svg);
    el("rect", { x: L, y: top, width: R - L, height: h, rx: 8, fill: "var(--neutral)" }, svg);
    const sorted = [...S.points].sort((a, b) => a.dist - b.dist);
    sorted.forEach((p, i) => {
      const x0 = X(p.dist), x1 = i + 1 < sorted.length ? X(sorted[i + 1].dist) : X(Math.min(D, p.dist + 0.5));
      const seg = el("rect", { x: x0, y: top, width: Math.max(4, x1 - x0 - 2), height: h, rx: 4, fill: STATUS[p.overall].color }, svg);
      seg.addEventListener("mousemove", e => showTip(e, pointTip(p)));
      seg.addEventListener("mouseleave", hideTip);
      if (p.overall === 2) el("text", { x: x0 + 4, y: top + h / 2 + 5, style: "fill:#fff;font-size:14px" }, svg, "⛔");
    });
    for (let m = 0; m <= D; m++) {
      el("line", { x1: X(m), x2: X(m), y1: top + h + 6, y2: top + h + 12, stroke: "var(--text-muted)" }, svg);
      if (D <= 20 || m % 2 === 0) el("text", { x: X(m), y: top + h + 26, "text-anchor": "middle" }, svg, m + " м");
    }
    el("text", { x: L - 40, y: top + h / 2 + 4 }, svg, "вход");
    if (S.last && !["WARM", "CAL"].includes(S.last.state)) {
      const x = X(Math.min(S.last.dist, D));
      el("path", { d: `M${x - 9} ${top - 22} L${x + 9} ${top - 22} L${x} ${top - 8} Z`, fill: "var(--yellow)" }, svg);
      el("text", { x, y: top - 27, "text-anchor": "middle", style: "fill:var(--text-primary);font-weight:600" }, svg, "робот");
    }
  }

  function renderLegend(id) {
    const box = $(id); if (!box || box.dataset.built) return;
    box.innerHTML = SENSORS.map(s => `<span><span class="swatch" style="background:${s.color}"></span>${s.name} (${s.chip})</span>`).join("");
    box.dataset.built = 1;
  }

  function renderChart(id) {
    const svg = $(id); if (!svg) return;
    svg.innerHTML = "";
    const L = 52, R = 930, T = 14, Bt = 290, D = maxDist(), YM = 150;
    const X = d => L + (R - L) * d / D, Y = v => Bt - (Bt - T) * Math.min(v, YM) / YM;
    [0, 50, 100, 150].forEach(v => {
      el("line", { x1: L, x2: R, y1: Y(v), y2: Y(v), stroke: "var(--grid)" }, svg);
      el("text", { x: L - 8, y: Y(v) + 4, "text-anchor": "end" }, svg, v + "%");
    });
    for (let m = 0; m <= D; m += (D > 20 ? 2 : 1)) el("text", { x: X(m), y: Bt + 18, "text-anchor": "middle" }, svg, m + " м");
    el("line", { x1: L, x2: R, y1: Y(50), y2: Y(50), stroke: "var(--warning)", "stroke-dasharray": "6 5", "stroke-width": 1.5 }, svg);
    el("text", { x: R, y: Y(50) - 6, "text-anchor": "end" }, svg, "▲ осторожно");
    el("line", { x1: L, x2: R, y1: Y(100), y2: Y(100), stroke: "var(--critical)", "stroke-dasharray": "6 5", "stroke-width": 1.5 }, svg);
    el("text", { x: R, y: Y(100) - 6, "text-anchor": "end" }, svg, "⛔ тревога");
    const pts = S.points;
    if (!pts.length) {
      el("text", { x: (L + R) / 2, y: (T + Bt) / 2, "text-anchor": "middle", style: "font-size:15px" }, svg, "Точки появятся после старта миссии");
    } else {
      SENSORS.forEach((s, i) => {
        el("polyline", { points: pts.map(p => `${X(p.dist)},${Y(p.idx[i])}`).join(" "), fill: "none", stroke: s.color, "stroke-width": 2, "stroke-linejoin": "round", "stroke-linecap": "round" }, svg);
        pts.forEach(p => el("circle", { cx: X(p.dist), cy: Y(p.idx[i]), r: 4, fill: s.color, stroke: "var(--surface-1)", "stroke-width": 2 }, svg));
      });
      const lp = pts[pts.length - 1];
      const labels = SENSORS.map((s, i) => ({ y: Y(lp.idx[i]), text: s.chip })).sort((a, b) => a.y - b.y);
      for (let k = 1; k < labels.length; k++) if (labels[k].y - labels[k - 1].y < 14) labels[k].y = labels[k - 1].y + 14;
      const over = labels[labels.length - 1].y - (Bt - 6);
      if (over > 0) labels.forEach(l => l.y -= over);
      labels.forEach(l => el("text", { x: Math.min(X(lp.dist) + 8, R + 6), y: l.y + 4, style: "fill:var(--text-primary)" }, svg, l.text));
    }
    const cross = el("line", { y1: T, y2: Bt, stroke: "var(--text-muted)", visibility: "hidden" }, svg);
    const hit = el("rect", { x: L, y: T, width: R - L, height: Bt - T, fill: "transparent" }, svg);
    hit.addEventListener("mousemove", e => {
      if (!pts.length) return;
      const p0 = svg.createSVGPoint(); p0.x = e.clientX; p0.y = e.clientY;
      const loc = p0.matrixTransform(svg.getScreenCTM().inverse());
      let best = pts[0];
      for (const p of pts) if (Math.abs(X(p.dist) - loc.x) < Math.abs(X(best.dist) - loc.x)) best = p;
      cross.setAttribute("x1", X(best.dist)); cross.setAttribute("x2", X(best.dist)); cross.setAttribute("visibility", "visible");
      showTip(e, pointTip(best));
    });
    hit.addEventListener("mouseleave", () => { cross.setAttribute("visibility", "hidden"); hideTip(); });
  }

  function renderTable(id) {
    const tb = $(id); if (!tb) return;
    const head = `<tr><th>№</th><th>Расстояние</th>${SENSORS.map(s => `<th>${s.chip}</th>`).join("")}<th>Статус</th></tr>`;
    const rows = S.points.slice().reverse().map((p, k) => `<tr><td>${S.points.length - k}</td><td>≈${p.dist.toFixed(1)} м</td>` +
      `<td>≈${fmt(p.ch4)} ppm</td><td>≈${fmt(p.co)} ppm</td><td>${fmt(p.q135)} %</td><td>${fmt(p.q2)} %</td>` +
      `<td><span style="color:${STATUS[p.overall].color}">${STATUS[p.overall].icon}</span> ${STATUS[p.overall].label}</td></tr>`).join("");
    tb.innerHTML = head + (rows || `<tr><td colspan="7" style="color:var(--text-muted)">Пока нет точек замера</td></tr>`);
  }

  function renderLink(dotId, textId, badgeId) {
    const l = link();
    if ($(dotId)) $(dotId).style.background = l.color;
    if ($(textId)) $(textId).textContent = l.text;
    if ($(badgeId)) { $(badgeId).textContent = l.badge; $(badgeId).className = "live-badge" + (l.live ? "" : " off"); }
    return l;
  }

  return { S, SENSORS, STATE_RU, STATUS, on: f => listeners.push(f), connectLive, startDemo, link, verdict,
           renderVerdict, renderTiles, renderTunnel, renderLegend, renderChart, renderTable, renderLink, fmt };
})();
