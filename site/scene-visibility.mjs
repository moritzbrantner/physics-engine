export function createCamera(position, yaw, pitch, aspect, fovRadians, nearPlane, farPlane) {
  return {
    position,
    sinYaw: Math.sin(yaw),
    cosYaw: Math.cos(yaw),
    sinPitch: Math.sin(pitch),
    cosPitch: Math.cos(pitch),
    tanHalfY: Math.tan(fovRadians / 2),
    aspect,
    nearPlane,
    farPlane,
  };
}

export function cameraSpace(point, camera) {
  const dx = point[0] - camera.position[0];
  const dy = point[1] - camera.position[1];
  const dz = point[2] - camera.position[2];
  const cameraX = dx * camera.cosYaw + dz * camera.sinYaw;
  const forward = dx * camera.sinYaw - dz * camera.cosYaw;
  return [
    cameraX,
    dy * camera.cosPitch + forward * camera.sinPitch,
    -dy * camera.sinPitch + forward * camera.cosPitch,
  ];
}

// The enclosing sphere includes every orientation of an OBB. Testing its most
// permissive frustum extent may retain extra bodies, but never clips a visible one.
export function bodyMayBeVisible(body, camera) {
  const [x, y, z] = cameraSpace(body.position, camera);
  const radius = Math.hypot(...body.half);
  if (z + radius < camera.nearPlane || z - radius > camera.farPlane) return false;
  const tanHalfX = camera.tanHalfY * camera.aspect;
  return (
    Math.abs(x) <= z * tanHalfX + radius * Math.hypot(1, tanHalfX) &&
    Math.abs(y) <= z * camera.tanHalfY + radius * Math.hypot(1, camera.tanHalfY)
  );
}
