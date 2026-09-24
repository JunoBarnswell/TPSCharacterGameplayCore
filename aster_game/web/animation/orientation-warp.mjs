export function orientationWarpAngle(movementDirection, characterYaw, maxAngle = 90) {
  let difference = ((movementDirection - characterYaw + 540) % 360) - 180;
  difference = Math.max(-maxAngle, Math.min(maxAngle, difference));
  return difference;
}
