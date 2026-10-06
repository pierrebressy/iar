// geom.js — géométrie pour le recalage QR / SLAM
// Fonctionne dans le navigateur (window.Geom) et sous Node (module.exports) pour les tests.
(function (root) {
  'use strict';

  // ---------- vecteurs / matrices 3x3 (tableaux row-major de 9) ----------
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = (a) => Math.hypot(a[0], a[1], a[2]);
  const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];

  const mulMV = (M, v) => [
    M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
    M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
    M[6] * v[0] + M[7] * v[1] + M[8] * v[2],
  ];
  const mulMM = (A, B) => {
    const C = new Array(9);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
      C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
    return C;
  };
  const transpose = (M) => [M[0], M[3], M[6], M[1], M[4], M[7], M[2], M[5], M[8]];

  // Transformation rigide {R, t} : p' = R p + t
  const apply = (T, p) => add(mulMV(T.R, p), T.t);
  const compose = (A, B) => ({ R: mulMM(A.R, B.R), t: add(mulMV(A.R, B.t), A.t) }); // A∘B
  const invert = (T) => { const Rt = transpose(T.R); return { R: Rt, t: scale(mulMV(Rt, T.t), -1) }; };

  function quatToMat(q) { // q = {x,y,z,w}
    const { x, y, z, w } = q;
    return [
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
    ];
  }

  // Euler en degrés, ordre 'XYZ' (comme Three.js) : R = Rx * Ry * Rz
  function eulerXYZDeg(e) {
    const [ax, ay, az] = (e || [0, 0, 0]).map((d) => (d * Math.PI) / 180);
    const cx = Math.cos(ax), sx = Math.sin(ax), cy = Math.cos(ay), sy = Math.sin(ay), cz = Math.cos(az), sz = Math.sin(az);
    const Rx = [1, 0, 0, 0, cx, -sx, 0, sx, cx];
    const Ry = [cy, 0, sy, 0, 1, 0, -sy, 0, cy];
    const Rz = [cz, -sz, 0, sz, cz, 0, 0, 0, 1];
    return mulMM(mulMM(Rx, Ry), Rz);
  }

  function rodrigues(r) {
    const th = norm(r);
    if (th < 1e-12) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
    const k = scale(r, 1 / th), c = Math.cos(th), s = Math.sin(th), v = 1 - c;
    const [x, y, z] = k;
    return [
      c + x * x * v, x * y * v - z * s, x * z * v + y * s,
      y * x * v + z * s, c + y * y * v, y * z * v - x * s,
      z * x * v - y * s, z * y * v + x * s, c + z * z * v,
    ];
  }
  function rotToRodrigues(R) {
    const c = Math.max(-1, Math.min(1, (R[0] + R[4] + R[8] - 1) / 2));
    const th = Math.acos(c);
    if (th < 1e-9) return [0, 0, 0];
    const s = 2 * Math.sin(th);
    if (Math.abs(s) < 1e-6) { // th ≈ π
      const x = Math.sqrt(Math.max(0, (R[0] + 1) / 2)), y = Math.sqrt(Math.max(0, (R[4] + 1) / 2)) * Math.sign(R[1] || 1), z = Math.sqrt(Math.max(0, (R[8] + 1) / 2)) * Math.sign(R[2] || 1);
      return scale([x, y, z], th);
    }
    return scale([R[7] - R[5], R[2] - R[6], R[3] - R[1]], th / s);
  }

  // ---------- algèbre linéaire générique ----------
  function solve(A, b) { // élimination de Gauss avec pivot partiel, A n×n (tableau de lignes)
    const n = b.length, M = A.map((row, i) => [...row, b[i]]);
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (Math.abs(M[p][c]) < 1e-14) return null;
      [M[c], M[p]] = [M[p], M[c]];
      for (let r = c + 1; r < n; r++) {
        const f = M[r][c] / M[c][c];
        for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
      }
    }
    const x = new Array(n);
    for (let r = n - 1; r >= 0; r--) {
      let s = M[r][n];
      for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
      x[r] = s / M[r][r];
    }
    return x;
  }

  // Valeurs/vecteurs propres d'une matrice symétrique (Jacobi)
  function eigSym(Ain) {
    const n = Ain.length, A = Ain.map((r) => r.slice());
    const V = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
    for (let sweep = 0; sweep < 100; sweep++) {
      let off = 0;
      for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += A[i][j] * A[i][j];
      if (off < 1e-22) break;
      for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
        if (Math.abs(A[p][q]) < 1e-30) continue;
        const th = (A[q][q] - A[p][p]) / (2 * A[p][q]);
        const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
        const c = 1 / Math.sqrt(t * t + 1), s = t * c;
        for (let k = 0; k < n; k++) { const akp = A[k][p], akq = A[k][q]; A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq; }
        for (let k = 0; k < n; k++) { const apk = A[p][k], aqk = A[q][k]; A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk; }
        for (let k = 0; k < n; k++) { const vkp = V[k][p], vkq = V[k][q]; V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq; }
      }
    }
    return { values: A.map((r, i) => r[i]), vectors: V }; // vecteurs en colonnes
  }

  // ---------- PnP planaire ----------
  // model : [[X,Y],...] dans le plan Z=0 du QR ; img : [[x,y],...] coordonnées normalisées
  // caméra convention OpenCV (x droite, y bas, z devant). Retour {R,t,rms} avec p_cam = R p_model + t
  function homography(model, img) {
    const A = [], b = [];
    for (let i = 0; i < model.length; i++) {
      const [X, Y] = model[i], [x, y] = img[i];
      A.push([X, Y, 1, 0, 0, 0, -x * X, -x * Y]); b.push(x);
      A.push([0, 0, 0, X, Y, 1, -y * X, -y * Y]); b.push(y);
    }
    let h;
    if (A.length === 8) h = solve(A, b);
    else { // moindres carrés
      const AtA = Array.from({ length: 8 }, (_, i) => Array.from({ length: 8 }, (_, j) => A.reduce((s, r) => s + r[i] * r[j], 0)));
      const Atb = Array.from({ length: 8 }, (_, i) => A.reduce((s, r, k) => s + r[i] * b[k], 0));
      h = solve(AtA, Atb);
    }
    return h ? [...h, 1] : null;
  }

  function project(R, t, P) { const c = add(mulMV(R, P), t); return [c[0] / c[2], c[1] / c[2], c[2]]; }

  function residuals(params, model3, img) {
    const R = rodrigues(params.slice(0, 3)), t = params.slice(3);
    const r = [];
    for (let i = 0; i < model3.length; i++) { const p = project(R, t, model3[i]); r.push(p[0] - img[i][0], p[1] - img[i][1]); }
    return r;
  }

  function refineLM(params, model3, img, iters = 30) {
    let p = params.slice(), r = residuals(p, model3, img), cost = r.reduce((s, v) => s + v * v, 0), lambda = 1e-3;
    for (let it = 0; it < iters; it++) {
      const J = [];
      for (let k = 0; k < 6; k++) {
        const eps = k < 3 ? 1e-6 : 1e-6 * Math.max(1, Math.abs(p[k]));
        const q = p.slice(); q[k] += eps;
        const rq = residuals(q, model3, img);
        J.push(rq.map((v, i) => (v - r[i]) / eps)); // J[k][i]
      }
      const JtJ = Array.from({ length: 6 }, (_, a) => Array.from({ length: 6 }, (_, b) => J[a].reduce((s, v, i) => s + v * J[b][i], 0)));
      const Jtr = Array.from({ length: 6 }, (_, a) => J[a].reduce((s, v, i) => s + v * r[i], 0));
      let improved = false;
      for (let tries = 0; tries < 8; tries++) {
        const A = JtJ.map((row, i) => row.map((v, j) => (i === j ? v * (1 + lambda) + 1e-12 : v)));
        const d = solve(A, Jtr.map((v) => -v));
        if (!d) { lambda *= 10; continue; }
        const q = p.map((v, i) => v + d[i]);
        const rq = residuals(q, model3, img), cq = rq.reduce((s, v) => s + v * v, 0);
        if (cq < cost) { p = q; r = rq; const dc = cost - cq; cost = cq; lambda = Math.max(lambda / 10, 1e-9); improved = true; if (dc < 1e-18) it = iters; break; }
        lambda *= 10;
      }
      if (!improved) break;
    }
    return { params: p, rms: Math.sqrt(cost / model3.length) };
  }

  function planarPnP(model, img) {
    const H = homography(model, img);
    if (!H) return null;
    let h1 = [H[0], H[3], H[6]], h2 = [H[1], H[4], H[7]], h3 = [H[2], H[5], H[8]];
    let lam = 2 / (norm(h1) + norm(h2));
    if (h3[2] * lam < 0) lam = -lam; // le QR doit être devant la caméra
    let r1 = scale(h1, lam), r2 = scale(h2, lam);
    const t = scale(h3, lam);
    // orthonormalisation symétrique de (r1, r2)
    const a = scale(r1, 1 / norm(r1)), b = scale(r2, 1 / norm(r2));
    const bis = scale(add(a, b), 1 / norm(add(a, b))), dif = scale(sub(a, b), 1 / norm(sub(a, b)));
    r1 = scale(add(bis, dif), Math.SQRT1_2); r2 = scale(sub(bis, dif), Math.SQRT1_2);
    const r3 = cross(r1, r2);
    const R0 = [r1[0], r2[0], r3[0], r1[1], r2[1], r3[1], r1[2], r2[2], r3[2]];
    const model3 = model.map(([X, Y]) => [X, Y, 0]);
    const { params, rms } = refineLM([...rotToRodrigues(R0), ...t], model3, img);
    return { R: rodrigues(params.slice(0, 3)), t: params.slice(3), rms };
  }

  // ---------- Recalage rigide (Horn, quaternions) : trouve T minimisant Σ|T a_i - b_i|² ----------
  function rigidFit(A, B, weights) {
    const n = A.length, w = weights || new Array(n).fill(1), W = w.reduce((s, v) => s + v, 0);
    let ca = [0, 0, 0], cb = [0, 0, 0];
    for (let i = 0; i < n; i++) { ca = add(ca, scale(A[i], w[i] / W)); cb = add(cb, scale(B[i], w[i] / W)); }
    const S = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < n; i++) {
      const a = sub(A[i], ca), b = sub(B[i], cb);
      for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) S[r][c] += w[i] * a[r] * b[c];
    }
    const [[Sxx, Sxy, Sxz], [Syx, Syy, Syz], [Szx, Szy, Szz]] = S;
    const N = [
      [Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx],
      [Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz],
      [Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy],
      [Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz],
    ];
    const { values, vectors } = eigSym(N);
    let k = 0; for (let i = 1; i < 4; i++) if (values[i] > values[k]) k = i;
    const q = { w: vectors[0][k], x: vectors[1][k], y: vectors[2][k], z: vectors[3][k] };
    const R = quatToMat(q);
    const t = sub(cb, mulMV(R, ca));
    let se = 0;
    for (let i = 0; i < n; i++) { const d = sub(add(mulMV(R, A[i]), t), B[i]); se += w[i] * dot(d, d); }
    return { R, t, rms: Math.sqrt(se / W) };
  }

  const api = { sub, add, dot, cross, norm, scale, mulMV, mulMM, transpose, apply, compose, invert, quatToMat, eulerXYZDeg, rodrigues, rotToRodrigues, solve, eigSym, homography, planarPnP, rigidFit };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.Geom = api;
})(typeof window !== 'undefined' ? window : globalThis);
