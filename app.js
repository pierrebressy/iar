// app.js — WebAR : recalage par QR code + suivi SLAM (8th Wall) + étiquettes d'objets connus
/* global XR8, ZXingWASM, Geom, Refine */
(function () {
  'use strict';
  const G = Geom;

  // ---------------- configuration ----------------
  const DEFAULTS = {
    pixelMaxDimension: 1280,   // résolution d'analyse du QR (plus = plus précis, plus lent)
    framesForLock: 20,         // nb d'observations du QR avant d'afficher les objets
    maxFrames: 80,             // fenêtre glissante d'observations conservées par ancre
    maxReprojPx: 1.5,          // rejet des poses dont l'erreur de reprojection dépasse ce seuil (px écran)
    requireFrame: true,        // n'utiliser que les vues où le cadre noir a été affiné
    focusAngleDeg: 8,          // un objet à moins de cet angle du centre de l'écran est "visé"
    maxLabelDistance: 15,      // m, au-delà on n'affiche plus l'étiquette
    pixelRotation: 'auto',     // 'auto' | 0 | 90 | 270 — orientation du tableau de pixels vs écran
    anchors: {},
    objects: [],
  };
  let CFG = DEFAULTS;

  // ---------------- état ----------------
  const state = {
    reality: null,
    trackingOK: false,
    obs: [],               // [{room:[x,y,z], world:[x,y,z], anchor}] — paires pour le recalage
    T_world_room: null,    // résultat du recalage
    fitRms: null,
    lastQR: null,          // {anchor, cornersScreen, reprojScreen, reproj, t}
    frame: 0,
    debug: false,
    selected: null,        // objet touché par l'utilisateur
    labelBoxes: [],
  };

  window.__arState = state; // accès console pour le diagnostic

  const $ = (id) => document.getElementById(id);
  const overlay = $('overlay'), ctx = overlay.getContext('2d');
  const glCanvas = $('camerafeed');

  // ---------------- tailles des canevas ----------------
  function resize() {
    const dpr = window.devicePixelRatio || 1;
    const W = window.innerWidth, H = window.innerHeight;
    for (const c of [glCanvas, overlay]) {
      c.width = Math.round(W * dpr); c.height = Math.round(H * dpr);
      c.style.width = W + 'px'; c.style.height = H + 'px';
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  window.addEventListener('resize', resize);

  // ---------------- conversions de coordonnées ----------------
  // pixel du tableau caméra (u,v) -> point écran (x,y) en px CSS.
  // Hypothèse : le tableau représente la même vue que le canevas ; s'il a un autre format,
  // on suppose un recadrage "cover" centré (comme l'affichage du flux).
  function pixelToScreen(u, v, cols, rows) {
    const W = window.innerWidth, H = window.innerHeight;
    let rot = CFG.pixelRotation;
    if (rot === 'auto') rot = (cols > rows) !== (W > H) ? 90 : 0;
    let x = u, y = v, w = cols, h = rows;
    if (rot === 90) { x = rows - v; y = u; w = rows; h = cols; }
    else if (rot === 270) { x = v; y = cols - u; w = rows; h = cols; }
    const s = Math.max(W / w, H / h);
    return [(x - w / 2) * s + W / 2, (y - h / 2) * s + H / 2];
  }

  // point écran -> coordonnées normalisées caméra (convention OpenCV : x droite, y bas, z devant)
  function screenToNormalizedCV(x, y, P) {
    const W = window.innerWidth, H = window.innerHeight;
    const ndcX = (2 * x) / W - 1, ndcY = 1 - (2 * y) / H;
    const a = (ndcX + P[8]) / P[0], b = (ndcY + P[9]) / P[5]; // X/(-Z), Y/(-Z) repère Three
    return [a, -b];
  }

  // point repère caméra Three (x droite, y haut, regarde -z) -> écran
  function camToScreen(pc, P) {
    const W = window.innerWidth, H = window.innerHeight;
    const cx = P[0] * pc[0] + P[4] * pc[1] + P[8] * pc[2] + P[12];
    const cy = P[1] * pc[0] + P[5] * pc[1] + P[9] * pc[2] + P[13];
    const cw = P[3] * pc[0] + P[7] * pc[1] + P[11] * pc[2] + P[15];
    return [((cx / cw + 1) / 2) * W, ((1 - cy / cw) / 2) * H];
  }

  function cameraPose(r) { // repère caméra Three -> monde SLAM
    return { R: G.quatToMat(r.rotation), t: [r.position.x, r.position.y, r.position.z] };
  }
  const CV_TO_THREE = [1, 0, 0, 0, -1, 0, 0, 0, -1];

  // pose d'une ancre dans le repère pièce
  function anchorPose(a) {
    return { R: G.eulerXYZDeg(a.rotation), t: a.position || [0, 0, 0] };
  }

  // ---------------- détection QR + affinage sur le cadre + recalage ----------------
  let busy = false;
  // Copie du tableau caméra (le tampon est réutilisé par le moteur) + pose de la même frame
  function snapshot(cpa, reality) {
    const { rows, cols, rowBytes, pixels } = cpa;
    const rgba = new Uint8ClampedArray(cols * rows * 4), L = new Uint8Array(cols * rows);
    for (let r = 0; r < rows; r++) {
      rgba.set(pixels.subarray(r * rowBytes, r * rowBytes + cols * 4), r * cols * 4);
    }
    for (let i = 0, j = 0; i < L.length; i++, j += 4) L[i] = (rgba[j] * 77 + rgba[j + 1] * 150 + rgba[j + 2] * 29) >> 8;
    return {
      cols, rows, rgba, L,
      reality: {
        position: { ...reality.position }, rotation: { ...reality.rotation },
        intrinsics: Array.from(reality.intrinsics || []), trackingStatus: reality.trackingStatus,
      },
    };
  }

  async function detectQR(snap) {
    const { cols, rows, rgba, L, reality } = snap;
    const res = await ZXingWASM.readBarcodes({ data: rgba, width: cols, height: rows, colorSpace: 'srgb' }, { formats: ['QRCode'], tryHarder: false, maxNumberOfSymbols: 4 });
    for (const code of res) {
      const anchorId = parseAnchorId(code.text);
      const anchor = anchorId && CFG.anchors[anchorId];
      if (!anchor) { setStatus(`QR inconnu : « ${code.text.slice(0, 40)} »`); continue; }
      processAnchor(anchorId, anchor, code.position, snap);
    }
  }

  function processAnchor(anchorId, anchor, pos, snap) {
    const { cols, rows, L, reality } = snap;
    state.pixSize = `${cols}×${rows}`;
    const qrPx = [pos.topLeft, pos.topRight, pos.bottomRight, pos.bottomLeft].map((p) => [p.x, p.y]);
    // 1) affinage sous-pixel sur le cadre noir imprimé
    let model, imgPx, method;
    const Hq = Refine.homographyFromQR(anchor.qrSide, qrPx);
    const ref = Hq && anchor.frameOuter ? Refine.refineFrame(L, cols, rows, Hq, { outer: anchor.frameOuter, inner: anchor.frameInner }) : null;
    if (ref && ref.edgeRms < 1.0) { model = ref.model; imgPx = ref.img; method = 'cadre'; }
    else { const h = anchor.qrSide / 2; model = [[-h, h], [h, h], [h, -h], [-h, -h]]; imgPx = qrPx; method = 'QR'; }

    // 2) pose par PnP planaire (repère cible : x droite, y haut, z sort de la page)
    const P = reality.intrinsics;
    const toScreen = (p) => pixelToScreen(p[0], p[1], cols, rows);
    const img = imgPx.map((p) => { const s = toScreen(p); return screenToNormalizedCV(s[0], s[1], P); });
    const pose = G.planarPnP(model, img);
    if (!pose) return;
    const reproj = (pose.rms * P[0] * window.innerWidth) / 2; // px écran
    const outline = (half) => [[-half, half], [half, half], [half, -half], [-half, -half]];
    const projModel = (pts) => pts.map(([X, Y]) => camToScreen(G.mulMV(CV_TO_THREE, G.add(G.mulMV(pose.R, [X, Y, 0]), pose.t)), P));
    const outer = anchor.frameOuter ? anchor.frameOuter / 2 : anchor.qrSide / 2;
    state.lastQR = {
      anchor: anchorId, method, reproj, t: performance.now(), dist: G.norm(pose.t),
      detected: imgPx.map(toScreen), reprojScreen: projModel(model), outline: projModel(outline(outer)),
      edgeRms: ref ? ref.edgeRms : null,
    };
    // garde-fous : toute valeur non finie (NaN) est rejetée et signalée dans le panneau Debug
    const fin = (a) => Array.from(a).every(Number.isFinite);
    const Tcam0 = cameraPose(reality);
    const checks = {
      intrinsics: fin(P), coins: imgPx.every(fin), norm: img.every(fin),
      poseR: fin(pose.R), poseT: fin(pose.t), camR: fin(Tcam0.R), camT: fin(Tcam0.t),
    };
    const bad = Object.keys(checks).filter((k) => !checks[k]);
    if (bad.length || !Number.isFinite(reproj)) {
      state.nanInfo = `NaN dans : ${bad.join(', ') || 'reproj'} | P=[${Array.from(P).slice(0, 16).map((v) => (+v).toFixed(3)).join(',')}] pos=${JSON.stringify(reality.position)} rot=${JSON.stringify(reality.rotation)}`;
      return;
    }
    if (!(reproj <= CFG.maxReprojPx) || reality.trackingStatus !== 'NORMAL') return;
    if (CFG.requireFrame && method !== 'cadre') return;

    // 3) paires (repère pièce ↔ monde SLAM) pour le recalage global
    const Tcam = cameraPose(reality);
    const Troom_anchor = anchorPose(anchor);
    const pts = outline(outer).concat([[0, 0]]); // 5 points par vue
    for (const [X, Y] of pts) {
      const pcv = G.add(G.mulMV(pose.R, [X, Y, 0]), pose.t);
      state.obs.push({ room: G.apply(Troom_anchor, [X, Y, 0]), world: G.apply(Tcam, G.mulMV(CV_TO_THREE, pcv)), anchor: anchorId });
    }
    const mine = state.obs.filter((o) => o.anchor === anchorId).length;
    if (mine > CFG.maxFrames * 5) state.obs.splice(state.obs.findIndex((o) => o.anchor === anchorId), 5);
    if (state.obs.length / 5 >= CFG.framesForLock || state.T_world_room) {
      const fit = G.rigidFit(state.obs.map((o) => o.room), state.obs.map((o) => o.world));
      if (!fit.R.every(Number.isFinite) || !fit.t.every(Number.isFinite)) { state.nanInfo = 'NaN dans le recalage global'; return; }
      state.T_world_room = { R: fit.R, t: fit.t };
      state.fitRms = fit.rms;
    }
  }

  function parseAnchorId(text) {
    const m = /^AR-ANCHOR:(.+)$/.exec(text.trim());
    return m ? m[1] : null;
  }

  // ---------------- rendu de l'overlay ----------------
  function draw() {
    const W = window.innerWidth, H = window.innerHeight;
    ctx.clearRect(0, 0, W, H);
    const r = state.reality;
    state.labelBoxes = [];
    if (!r) return;
    const P = r.intrinsics;

    // QR détecté : coins détectés (vert) et reprojetés (magenta) — vérification de cohérence
    if (state.lastQR && performance.now() - state.lastQR.t < 300) {
      poly(state.lastQR.outline, state.lastQR.method === 'cadre' ? '#3ddc84' : '#ffd23f', 3);
      if (state.debug) {
        for (const [x, y] of state.lastQR.detected) { ctx.beginPath(); ctx.arc(x, y, 4, 0, 2 * Math.PI); ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 1.5; ctx.stroke(); }
        for (const [x, y] of state.lastQR.reprojScreen) { ctx.beginPath(); ctx.arc(x, y, 2, 0, 2 * Math.PI); ctx.fillStyle = '#ff4fd8'; ctx.fill(); }
      }
    }
    if (!state.T_world_room) return;

    const Tcam = cameraPose(r), Tcam_inv = G.invert(Tcam);
    let focus = null, focusAng = CFG.focusAngleDeg;
    const items = [];
    for (const obj of CFG.objects) {
      const pw = G.apply(state.T_world_room, obj.position);
      const pc = G.apply(Tcam_inv, pw); // repère caméra Three
      const dist = G.norm(pc);
      const ang = (Math.acos(Math.max(-1, Math.min(1, -pc[2] / dist))) * 180) / Math.PI;
      items.push({ obj, pc, dist, ang });
      if (pc[2] < 0 && ang < focusAng && dist < CFG.maxLabelDistance) { focus = obj; focusAng = ang; }
    }
    items.sort((a, b) => b.dist - a.dist); // les plus lointains d'abord
    for (const it of items) {
      if (it.dist > CFG.maxLabelDistance) continue;
      const s = it.pc[2] < 0 ? camToScreen(it.pc, P) : null;
      const onScreen = s && s[0] > 0 && s[0] < W && s[1] > 0 && s[1] < H;
      if (onScreen) drawLabel(it, s, it.obj === (state.selected || focus));
      else drawEdgeArrow(it, P);
    }
    showInfo(state.selected || focus);
  }

  function poly(pts, color, w) {
    ctx.beginPath(); pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y))); ctx.closePath();
    ctx.strokeStyle = color; ctx.lineWidth = w; ctx.stroke();
  }

  function drawLabel(it, [x, y], active) {
    const color = it.obj.color || '#ffb020';
    const r = Math.max(6, Math.min(16, 40 / it.dist));
    ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI);
    ctx.fillStyle = color + (active ? 'ff' : 'cc'); ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = '#fff'; ctx.stroke();
    const text = `${it.obj.label}  ${it.dist.toFixed(1)} m`;
    ctx.font = `${active ? 600 : 500} 15px -apple-system, system-ui, sans-serif`;
    const tw = ctx.measureText(text).width, bx = x - tw / 2 - 8, by = y - r - 34;
    ctx.fillStyle = active ? 'rgba(20,20,24,.92)' : 'rgba(20,20,24,.7)';
    roundRect(bx, by, tw + 16, 26, 8); ctx.fill();
    if (active) { ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.stroke(); }
    ctx.fillStyle = '#fff'; ctx.textBaseline = 'middle'; ctx.fillText(text, bx + 8, by + 13);
    state.labelBoxes.push({ obj: it.obj, x0: bx, y0: by, x1: bx + tw + 16, y1: y + r });
  }

  function drawEdgeArrow(it) {
    const W = window.innerWidth, H = window.innerHeight;
    // direction dans le plan image (si derrière, on inverse)
    let dx = it.pc[0], dy = -it.pc[1];
    if (Math.hypot(dx, dy) < 1e-6) dy = 1;
    const a = Math.atan2(dy, dx), m = 28;
    const cx = W / 2, cy = H / 2;
    const k = Math.min((W / 2 - m) / Math.abs(Math.cos(a) || 1e-6), (H / 2 - m) / Math.abs(Math.sin(a) || 1e-6));
    const x = cx + Math.cos(a) * k, y = Math.max(80, Math.min(H - 76, cy + Math.sin(a) * k));
    ctx.save(); ctx.translate(x, y); ctx.rotate(a);
    ctx.beginPath(); ctx.moveTo(12, 0); ctx.lineTo(-8, -9); ctx.lineTo(-8, 9); ctx.closePath();
    ctx.fillStyle = (it.obj.color || '#ffb020') + 'cc'; ctx.fill(); ctx.restore();
    ctx.font = '500 12px -apple-system, system-ui, sans-serif';
    ctx.fillStyle = 'rgba(255,255,255,.9)'; ctx.textAlign = 'center';
    const tw = ctx.measureText(it.obj.label).width;
    const lx = Math.max(tw / 2 + 6, Math.min(W - tw / 2 - 6, x - Math.cos(a) * 30));
    const ly = Math.max(90, Math.min(H - 80, y - Math.sin(a) * 26));
    ctx.fillText(it.obj.label, lx, ly);
    ctx.textAlign = 'start';
  }

  function roundRect(x, y, w, h, r) {
    ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
  }

  let shownInfo = null;
  function showInfo(obj) {
    if (obj === shownInfo) return;
    shownInfo = obj;
    const card = $('info');
    if (!obj) { card.classList.remove('show'); return; }
    $('info-title').textContent = obj.label;
    $('info-body').textContent = obj.info || '';
    card.classList.add('show');
  }

  overlay.addEventListener('click', (e) => {
    const hit = state.labelBoxes.find((b) => e.clientX >= b.x0 && e.clientX <= b.x1 && e.clientY >= b.y0 && e.clientY <= b.y1);
    state.selected = hit ? (state.selected === hit.obj ? null : hit.obj) : null;
  });

  // ---------------- statut ----------------
  function setStatus(msg) { $('status').textContent = msg; }
  function updateStatus() {
    const r = state.reality;
    let msg;
    if (!r) msg = 'Démarrage…';
    else if (!state.trackingOK) msg = r.trackingReason === 'INITIALIZING' ? 'Initialisation du suivi : bougez doucement le téléphone' : 'Suivi limité : bougez doucement, évitez les surfaces unies';
    else if (!state.T_world_room) {
      const n = Math.floor(state.obs.length / 5);
      const q = state.lastQR && performance.now() - state.lastQR.t < 500 ? state.lastQR : null;
      if (!q) msg = 'Visez le QR code de référence (40–80 cm)';
      else if (q.reproj > CFG.maxReprojPx) msg = `QR vu mais imprécis (${q.reproj.toFixed(1)} px) : rapprochez-vous, stabilisez`;
      else msg = `Recalage ${n}/${CFG.framesForLock} — tournez lentement autour du QR`;
    } else {
      msg = `Recalé · ${Math.floor(state.obs.length / 5)} vues · résidu ${(state.fitRms * 1000).toFixed(1)} mm`;
    }
    setStatus(msg);
    if (state.debug && r) {
      const lines = [`tracking ${r.trackingStatus}/${r.trackingReason}`];
      if (state.lastQR) lines.push(`ancre ${state.lastQR.anchor} [${state.lastQR.method}] d=${state.lastQR.dist.toFixed(2)} m reproj=${state.lastQR.reproj.toFixed(2)} px bords=${state.lastQR.edgeRms != null ? state.lastQR.edgeRms.toFixed(2) : '-'} px`);
      const p = currentRoomPosition();
      if (p) lines.push(`position pièce x=${p[0].toFixed(2)} y=${p[1].toFixed(2)} z=${p[2].toFixed(2)} m`);
      const Pi = r.intrinsics || [];
      lines.push(`P0=${(+Pi[0]).toFixed(3)} P5=${(+Pi[5]).toFixed(3)} P8=${(+Pi[8]).toFixed(3)} P9=${(+Pi[9]).toFixed(3)} n=${Pi.length}`);
      lines.push(`écran ${window.innerWidth}×${window.innerHeight} · pixels ${state.pixSize || '?'}`);
      if (state.nanInfo) lines.push(state.nanInfo.replace(/(.{60})/g, '$1\n'));
      $('debug').textContent = lines.join('\n');
    }
  }

  function currentRoomPosition(offset = 0) {
    if (!state.T_world_room || !state.reality) return null;
    const Tcam = cameraPose(state.reality);
    const pw = G.apply(Tcam, [0, 0, -offset]); // point devant la caméra
    return G.apply(G.invert(state.T_world_room), pw);
  }

  // ---------------- boutons ----------------
  $('btn-reset').onclick = () => { state.obs = []; state.T_world_room = null; state.fitRms = null; state.selected = null; };
  $('btn-debug').onclick = () => { state.debug = !state.debug; $('debug').hidden = !state.debug; };
  $('btn-pos').onclick = async () => {
    const p = currentRoomPosition();
    if (!p) { setStatus('Pas encore recalé'); return; }
    const txt = `[${p.map((v) => v.toFixed(2)).join(', ')}]`;
    try { await navigator.clipboard.writeText(txt); toast(`Position copiée : ${txt}`); } catch (e) { toast(txt); }
  };
  function toast(t) { const el = $('toast'); el.textContent = t; el.classList.add('show'); setTimeout(() => el.classList.remove('show'), 2500); }

  // ---------------- module pipeline 8th Wall ----------------
  function appModule() {
    // Sans module Three.js, il faut donner nous-mêmes au SLAM la taille du canevas,
    // sinon la matrice de projection (reality.intrinsics) est calculée sur 0×0 → NaN.
    const setProjection = ({ canvasWidth, canvasHeight }) => {
      if (!canvasWidth || !canvasHeight) return;
      XR8.XrController.updateCameraProjectionMatrix({
        cam: { pixelRectWidth: canvasWidth, pixelRectHeight: canvasHeight, nearClipPlane: 0.01, farClipPlane: 1000 },
        origin: { x: 0, y: 0, z: 0 },
        facing: { w: 1, x: 0, y: 0, z: 0 },
      });
    };
    return {
      name: 'qr-anchor-app',
      onStart: setProjection,
      onCanvasSizeChange: setProjection,
      onUpdate: ({ processCpuResult, processGpuResult }) => {
        const reality = processCpuResult && processCpuResult.reality;
        if (!reality) return;
        state.reality = reality;
        state.trackingOK = reality.trackingStatus === 'NORMAL';
        state.frame++;
        const cpa = processGpuResult && processGpuResult.camerapixelarray;
        // cadence d'analyse : souvent tant que le recalage n'est pas fait, moins ensuite
        const every = state.T_world_room ? 6 : 1;
        if (cpa && cpa.pixels && !busy && state.frame % every === 0) {
          busy = true;
          detectQR(snapshot(cpa, reality)).catch((e) => console.error(e)).finally(() => { busy = false; });
        }
        updateStatus();
        draw();
      },
    };
  }

  async function loadConfig() {
    try {
      const res = await fetch('config.json', { cache: 'no-store' });
      if (res.ok) CFG = Object.assign({}, DEFAULTS, await res.json());
    } catch (e) { console.warn('config.json introuvable', e); }
    $('nobj').textContent = `${CFG.objects.length} objets · ${Object.keys(CFG.anchors).length} ancre(s)`;
  }

  function start() {
    $('start').hidden = true;
    $('hud').hidden = false;
    resize();
    XR8.XrController.configure({ scale: 'absolute' });
    XR8.addCameraPipelineModules([
      XR8.GlTextureRenderer.pipelineModule(),
      XR8.XrController.pipelineModule(),
      XR8.CameraPixelArray.pipelineModule({ maxDimension: CFG.pixelMaxDimension }),
      appModule(),
      {
        name: 'errors',
        onCameraStatusChange: ({ status }) => { if (status === 'failed') setStatus('Accès caméra refusé'); },
        onException: (e) => setStatus('Erreur : ' + (e && e.message ? e.message : e)),
      },
    ]);
    XR8.run({ canvas: glCanvas, allowedDevices: XR8.XrConfig.device().MOBILE });
  }

  loadConfig();
  const ready = () => { $('btn-start').disabled = false; $('btn-start').textContent = 'Démarrer'; };
  window.XR8 ? ready() : window.addEventListener('xrloaded', ready);
  $('btn-start').onclick = start;
})();
