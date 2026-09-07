// Minimal quaternion helpers. Quaternions are [w, x, y, z] (MuJoCo order).

export function mat2quat(m) {           // m: length-9 row-major 3x3
  const t = m[0] + m[4] + m[8]; let q;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2;
    q = [0.25 * s, (m[7]-m[5])/s, (m[2]-m[6])/s, (m[3]-m[1])/s]; }
  else if (m[0] > m[4] && m[0] > m[8]) { const s = Math.sqrt(1+m[0]-m[4]-m[8])*2;
    q = [(m[7]-m[5])/s, 0.25*s, (m[1]+m[3])/s, (m[2]+m[6])/s]; }
  else if (m[4] > m[8]) { const s = Math.sqrt(1+m[4]-m[0]-m[8])*2;
    q = [(m[2]-m[6])/s, (m[1]+m[3])/s, 0.25*s, (m[5]+m[7])/s]; }
  else { const s = Math.sqrt(1+m[8]-m[0]-m[4])*2;
    q = [(m[3]-m[1])/s, (m[2]+m[6])/s, (m[5]+m[7])/s, 0.25*s]; }
  const n = Math.hypot(...q) || 1; return q.map(x => x / n);
}

export const qmul = (a, b) => [
  a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
  a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
  a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
  a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]];

export const qconj = q => [q[0], -q[1], -q[2], -q[3]];

export function qlog(q) {                // unit quat -> rotation vector (3)
  const w = Math.min(1, Math.max(-1, q[0]));
  const v = [q[1], q[2], q[3]];
  const n = Math.hypot(...v);
  if (n < 1e-9) return [0, 0, 0];
  const a = 2 * Math.atan2(n, w);
  return v.map(x => x * a / n);
}

// rotation vector that rotates qc onto qt (both [w,x,y,z])
export const rotErr = (qt, qc) => qlog(qmul(qt, qconj(qc)));

// THREE.Quaternion (x,y,z,w) -> [w,x,y,z]
export const fromThree = q => [q.w, q.x, q.y, q.z];
