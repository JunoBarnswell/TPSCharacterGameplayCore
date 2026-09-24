export function evaluateAimOffset(aimYaw, aimPitch) {
  return {
    yaw: Math.max(-180, Math.min(180, aimYaw)),
    pitch: Math.max(-89, Math.min(89, aimPitch)),
  };
}
