// Zuivere (three.js-vrije) coördinaat-hulpfuncties voor de controller -> TCP-mapping, zodat ze in Node te testen zijn
// (tools/test-teleop-mapping.mjs). Wereld = WebXR/three.js-referentieruimte (x rechts, y omhoog, -z vooruit);
// (ROOT_HOME in app.js: positie (0, 0.95, -0.32), rotatie +90° om y: MuJoCo +x = gebruiker-vooruit, MuJoCo +y = gebruiker-links.)
// root = de MuJoCo-scène-groep (mujocoRoot: positie + rotatie om y); mj = MuJoCo (x vooruit naar het bord, y links, z omhoog).

const rotY = (x, z, a) => [x * Math.cos(a) + z * Math.sin(a), -x * Math.sin(a) + z * Math.cos(a)];   // draai (x,z) om +y met hoek a (three-conventie)

/** wereldpunt -> lokaal in de root-groep (inverse van positie + rotatie om y). */
export function worldToRoot(p, rootPos, rootRotY) {
  const [x, z] = rotY(p[0] - rootPos[0], p[2] - rootPos[2], -rootRotY);
  return [x, p[1] - rootPos[1], z];
}
export function rootToWorld(p, rootPos, rootRotY) {
  const [x, z] = rotY(p[0], p[2], rootRotY);
  return [x + rootPos[0], p[1] + rootPos[1], z + rootPos[2]];
}
/** three (y omhoog) <-> MuJoCo (z omhoog), zelfde als de swizzle in scene-loader.js. */
export const threeToMj = v => [v[0], -v[2], v[1]];
export const mjToThree = v => [v[0], v[2], -v[1]];
export const mjToWorld = (p, rootPos, rootRotYv) => rootToWorld(mjToThree(p), rootPos, rootRotYv);

/** Kijkrichting van het hoofd (quaternion x,y,z,w) geprojecteerd op het vlak: hoek θ zodat vooruit = (-sinθ, -cosθ). null bij recht omhoog/omlaag kijken. */
export function headYaw(q) {
  const { x, y, z, w } = q;
  const fx = -2 * (x * z + w * y), fz = -(1 - 2 * (x * x + y * y));
  if (Math.hypot(fx, fz) < 0.2) return null;
  return Math.atan2(-fx, -fz);
}

// ---- quaternion-hulpjes (x,y,z,w <-> w,x,y,z) voor controller-oriëntatie in de root-frame
const qmulXYZW = (a, b) => [a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1], a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
                            a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3], a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];
/** controller-oriëntatie (WebXR x,y,z,w, wereld) -> [w,x,y,z] in de root-frame (inverse van de rotatie om y). */
export function worldQuatToRoot(o, rootRotY) {
  const h = -rootRotY / 2, inv = [0, Math.sin(h), 0, Math.cos(h)];       // rotatie om y over -rootRotY
  const q = qmulXYZW(inv, o);
  return [q[3], q[0], q[1], q[2]];
}

/** Volledige controller -> `raw` zoals HandTeleop.step het verwacht (pure versie van SandwichVR.readControllers). */
export function controllerToRaw(pos, orient, rootPos, rootRotY) {
  const lp = worldToRoot(pos, rootPos, rootRotY);
  return { pos: lp, quat: worldQuatToRoot(orient, rootRotY) };
}

export const HOME_POS = [0, 0.95, -0.32];     // scène-root t.o.v. de gebruiker (zit tussen de armen, kijkend naar het bord)
export const HOME_ROT_Y = Math.PI / 2;        // +90°: MuJoCo +x (naar het bord) = gebruiker-vooruit (-z)
/** Root-positie/-rotatie zo dat de gebruiker (hoofdpositie xz + kijkrichting `yaw`) tussen de armen zit, kijkend naar het bord. */
export function homeFromHead(headPos, yaw) {
  const dz = HOME_POS[2];                      // 0,32 m "voor" het hoofd, meegedraaid met de kijkrichting
  return { pos: [headPos[0] + dz * Math.sin(yaw), HOME_POS[1], headPos[2] + dz * Math.cos(yaw)], rotY: HOME_ROT_Y + yaw };
}
