// refine.js — affinage sous-pixel des coins du cadre noir imprimé autour du QR
// Principe : une homographie grossière (coins du QR donnés par le détecteur) prédit la position
// des 8 bords du cadre ; on cherche le maximum de gradient le long de la normale de chaque bord,
// on ajuste une droite (moindres carrés totaux + rejet), puis on intersecte les droites.
(function (root) {
  'use strict';
  const Geom = (typeof module !== 'undefined' && module.exports) ? require('./geom.js') : root.Geom;

  function applyH(H, x, y) {
    const w = H[6] * x + H[7] * y + H[8];
    return [(H[0] * x + H[1] * y + H[2]) / w, (H[3] * x + H[4] * y + H[5]) / w];
  }

  function bilinear(L, cols, rows, x, y) {
    if (x < 0 || y < 0 || x >= cols - 1 || y >= rows - 1) return NaN;
    const x0 = x | 0, y0 = y | 0, fx = x - x0, fy = y - y0, i = y0 * cols + x0;
    return (L[i] * (1 - fx) + L[i + 1] * fx) * (1 - fy) + (L[i + cols] * (1 - fx) + L[i + cols + 1] * fx) * fy;
  }

  // Ajustement de droite (moindres carrés totaux). Retour {n:[nx,ny], c} avec n·p = c
  function fitLine(pts) {
    let mx = 0, my = 0;
    for (const [x, y] of pts) { mx += x; my += y; }
    mx /= pts.length; my /= pts.length;
    let sxx = 0, sxy = 0, syy = 0;
    for (const [x, y] of pts) { const dx = x - mx, dy = y - my; sxx += dx * dx; sxy += dx * dy; syy += dy * dy; }
    const th = 0.5 * Math.atan2(2 * sxy, sxx - syy); // direction principale
    const n = [-Math.sin(th), Math.cos(th)];
    return { n, c: n[0] * mx + n[1] * my };
  }
  function robustLine(pts) {
    let line = fitLine(pts), kept = pts;
    for (let it = 0; it < 3; it++) {
      const d = kept.map(([x, y]) => Math.abs(line.n[0] * x + line.n[1] * y - line.c));
      const sorted = d.slice().sort((a, b) => a - b), med = sorted[sorted.length >> 1];
      const thr = Math.max(0.5, 3 * 1.4826 * med);
      const k2 = kept.filter((_, i) => d[i] <= thr);
      if (k2.length < 6 || k2.length === kept.length) break;
      kept = k2; line = fitLine(kept);
    }
    const res = kept.map(([x, y]) => line.n[0] * x + line.n[1] * y - line.c);
    line.rms = Math.sqrt(res.reduce((s, v) => s + v * v, 0) / res.length);
    line.count = kept.length;
    return line;
  }
  function intersect(a, b) {
    const det = a.n[0] * b.n[1] - a.n[1] * b.n[0];
    if (Math.abs(det) < 1e-9) return null;
    return [(a.c * b.n[1] - a.n[1] * b.c) / det, (a.n[0] * b.c - a.c * b.n[0]) / det];
  }

  // H : homographie modèle (m, repère cible : x droite, y haut) -> pixels
  // L : luminance Uint8 (cols×rows). frame : {outer, inner} côtés en m.
  // Retour {model:[[X,Y]...], img:[[u,v]...], rms} ou null
  function refineFrame(L, cols, rows, H, frame, opts = {}) {
    const samplesPerEdge = opts.samples || 24;
    const edges = [];
    for (const [half, outward] of [[frame.outer / 2, +1], [frame.inner / 2, -1]]) {
      // outward = +1 : bord extérieur (noir dedans → blanc dehors) ; -1 : bord intérieur
      const sides = [
        { p: (s) => [s, half], nrm: [0, 1] },   // haut
        { p: (s) => [half, -s], nrm: [1, 0] },  // droite
        { p: (s) => [-s, -half], nrm: [0, -1] },// bas
        { p: (s) => [-half, s], nrm: [-1, 0] }, // gauche
      ];
      const ring = [];
      for (const side of sides) {
        const pts = [];
        const thick = (frame.outer - frame.inner) / 2;
        for (let k = 0; k < samplesPerEdge; k++) {
          const s = (-0.85 + (1.7 * (k + 0.5)) / samplesPerEdge) * half;
          const M = side.p(s);
          const P = applyH(H, M[0], M[1]);
          // direction "noir → blanc" en modèle : dehors pour le bord extérieur, vers le centre pour l'intérieur
          const dirM = [side.nrm[0] * outward, side.nrm[1] * outward];
          const Q = applyH(H, M[0] + dirM[0] * thick * 0.5, M[1] + dirM[1] * thick * 0.5);
          let dx = Q[0] - P[0], dy = Q[1] - P[1];
          const len = Math.hypot(dx, dy);
          if (!(len > 1)) continue;
          dx /= len; dy /= len;
          const R = Math.max(2, Math.min(len * 0.9, 12)); // demi-fenêtre de recherche (px)
          const step = 0.5, n = Math.floor(R / step);
          const prof = [];
          for (let i = -n; i <= n; i++) prof.push(bilinear(L, cols, rows, P[0] + dx * i * step, P[1] + dy * i * step));
          if (prof.some(Number.isNaN)) continue;
          let best = -1, bestG = 0;
          for (let i = 1; i < prof.length - 1; i++) {
            const g = prof[i + 1] - prof[i - 1]; // positif = noir→blanc dans la direction de recherche
            if (g > bestG) { bestG = g; best = i; }
          }
          if (best < 2 || best > prof.length - 3 || bestG < (opts.minGrad || 20)) continue;
          const g0 = prof[best] - prof[best - 2], g1 = bestG, g2 = prof[best + 2] - prof[best];
          const den = g0 - 2 * g1 + g2;
          const off = Math.abs(den) > 1e-9 ? (0.5 * (g0 - g2)) / den : 0;
          const tpos = (best - n + Math.max(-0.5, Math.min(0.5, off))) * step;
          pts.push([P[0] + dx * tpos, P[1] + dy * tpos]);
        }
        if (pts.length < samplesPerEdge * 0.5) return null;
        ring.push(robustLine(pts));
      }
      edges.push(ring);
    }
    const model = [], img = [];
    let se = 0, ne = 0;
    edges.forEach((ring, ri) => {
      const half = (ri === 0 ? frame.outer : frame.inner) / 2;
      const cornersM = [[-half, half], [half, half], [half, -half], [-half, -half]]; // HG, HD, BD, BG
      // coin HG = gauche ∩ haut, HD = haut ∩ droite, BD = droite ∩ bas, BG = bas ∩ gauche
      const pairs = [[3, 0], [0, 1], [1, 2], [2, 3]];
      pairs.forEach(([a, b], k) => {
        const p = intersect(ring[a], ring[b]);
        if (p) { model.push(cornersM[k]); img.push(p); }
      });
      for (const l of ring) { se += l.rms * l.rms * l.count; ne += l.count; }
    });
    if (img.length < 8) return null;
    return { model, img, edgeRms: Math.sqrt(se / ne) };
  }

  // Homographie modèle → pixels à partir des coins du QR (ordre HG, HD, BD, BG)
  function homographyFromQR(qrSide, cornersPx) {
    const h = qrSide / 2;
    return Geom.homography([[-h, h], [h, h], [h, -h], [-h, -h]], cornersPx);
  }

  const api = { refineFrame, homographyFromQR, applyH };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.Refine = api;
})(typeof window !== 'undefined' ? window : globalThis);
