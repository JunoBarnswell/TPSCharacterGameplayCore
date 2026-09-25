import { PlatformerWorld } from "./world.mjs";

const canvas = document.getElementById("game");
const ctx = canvas.getContext("2d");
const world = new PlatformerWorld();
const keys = new Set();
const touch = new Set();
const hud = Object.fromEntries(["level", "name", "coins", "score", "lives"]
  .map((name) => [name, document.getElementById(name)]));
const modal = document.getElementById("message");
let paused = false;
let cameraX = 0;
let lastFrame = 0;
let accumulator = 0;
let coinCount = 0;
let toast = "";
let toastUntil = 0;

const rounded = (x, y, w, h, radius, color) => {
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.roundRect(x, y, w, h, radius);
  ctx.fill();
};

function message(title, body, button, onClick) {
  document.getElementById("messageTitle").textContent = title;
  document.getElementById("messageBody").textContent = body;
  const action = document.getElementById("messageAction");
  action.textContent = button;
  action.onclick = onClick;
  modal.hidden = false;
}

function updateHud() {
  hud.level.textContent = `${world.levelIndex + 1} / ${world.levels.length}`;
  hud.name.textContent = world.level.name;
  hud.coins.textContent = `${coinCount} / ${world.level.coins.length}`;
  hud.score.textContent = String(world.score).padStart(4, "0");
  hud.lives.textContent = "♥ ".repeat(world.lives).trim() || "—";
}

function reset() {
  world.restartGame();
  paused = false;
  cameraX = 0;
  coinCount = 0;
  keys.clear();
  touch.clear();
  modal.hidden = true;
  document.getElementById("pause").textContent = "暂停";
  updateHud();
}

function advance() {
  if (world.nextLevel()) {
    coinCount = 0;
    cameraX = 0;
    keys.clear();
    touch.clear();
    modal.hidden = true;
    updateHud();
  } else message("星灯重新亮起", `旅程完成！最终得分 ${world.score}。`,
    "再玩一次", reset);
}

function togglePause() {
  if (!["playing"].includes(world.state)) return;
  paused = !paused;
  document.getElementById("pause").textContent = paused ? "继续" : "暂停";
  modal.hidden = !paused;
  if (paused) message("稍作休息", "回到探险时，从当前位置继续。", "继续", togglePause);
}

function input() {
  return {
    left: keys.has("ArrowLeft") || keys.has("KeyA") || touch.has("left"),
    right: keys.has("ArrowRight") || keys.has("KeyD") || touch.has("right"),
    jump: keys.has("Space") || keys.has("KeyW") || keys.has("ArrowUp") || touch.has("jump"),
  };
}

window.addEventListener("keydown", (event) => {
  if (["ArrowLeft", "ArrowRight", "ArrowUp", "Space"].includes(event.code)) event.preventDefault();
  if (event.code === "KeyP" && !event.repeat) togglePause();
  if (event.code === "Enter" && world.state === "complete") advance();
  keys.add(event.code);
});
window.addEventListener("keyup", (event) => keys.delete(event.code));
window.addEventListener("blur", () => { keys.clear(); touch.clear(); });
for (const button of document.querySelectorAll("[data-control]")) {
  const control = button.dataset.control;
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    touch.add(control);
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
    button.addEventListener(type, () => touch.delete(control));
  }
}
document.getElementById("pause").addEventListener("click", togglePause);
document.getElementById("restart").addEventListener("click", reset);

function backdrop(time) {
  const [top, bottom] = world.level.sky;
  const gradient = ctx.createLinearGradient(0, 0, 0, 540);
  gradient.addColorStop(0, top);
  gradient.addColorStop(1, bottom);
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, 960, 540);
  const night = world.level.id === "night";
  ctx.fillStyle = night ? "#ffe7a4" : "#fff5d1";
  ctx.beginPath();
  ctx.arc(765 - cameraX * 0.035, 98, night ? 27 : 44, 0, Math.PI * 2);
  ctx.fill();
  if (night) {
    ctx.fillStyle = "#fbecce";
    for (let i = 0; i < 40; i++) {
      const x = ((i * 173 + 63 - cameraX * 0.04) % 1000 + 1000) % 1000;
      const y = 22 + ((i * 71) % 240);
      ctx.globalAlpha = 0.48 + 0.3 * Math.sin(time * 2 + i);
      ctx.fillRect(x, y, 2, 2);
    }
    ctx.globalAlpha = 1;
  }
  for (let layer = 0; layer < 2; layer++) {
    const scroll = cameraX * (0.14 + layer * 0.15);
    const baseY = 393 + layer * 65;
    ctx.fillStyle = night
      ? ["#546a90", "#435981"][layer]
      : world.level.id === "canyon"
        ? ["#dc9a83", "#ba816f"][layer]
        : ["#84bfb7", "#5b9e9d"][layer];
    ctx.beginPath();
    ctx.moveTo(0, 540);
    for (let i = -2; i < 10; i++) {
      const x = i * 184 - (scroll % 184);
      ctx.lineTo(x, baseY);
      ctx.quadraticCurveTo(x + 92, baseY - (i % 2 ? 98 : 70), x + 184, baseY);
    }
    ctx.lineTo(960, 540);
    ctx.fill();
  }
}

function drawSolid(block) {
  const x = Math.round(block.x - cameraX);
  if (x + block.w < -30 || x > 990) return;
  const ground = block.kind === "ground";
  rounded(x, block.y + 4, block.w, block.h, 7, ground ? "#4a6a57" : "#665d62");
  rounded(x, block.y, block.w, ground ? 14 : 11, 5,
    world.level.id === "canyon" ? "#edb28a" : "#a8d5a1");
  if (ground) {
    ctx.fillStyle = "#60876b";
    for (let p = 20; p < block.w; p += 68) ctx.fillRect(x + p, block.y + 21, 24, 4);
  }
}

function drawCoin(item, time) {
  if (item.collected) return;
  const x = item.x - cameraX;
  if (x < -25 || x > 985) return;
  const y = item.y + Math.sin(time * 4 + item.x) * 3;
  ctx.fillStyle = "#ffe08b";
  ctx.beginPath(); ctx.ellipse(x, y, 10, 12, 0, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = "#b37a36"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.ellipse(x, y, 6, 8, 0, 0, Math.PI * 2); ctx.stroke();
  ctx.fillStyle = "#fff3c6"; ctx.fillRect(x - 2, y - 6, 3, 4);
}

function drawEnemy(enemy, time) {
  if (!enemy.alive) return;
  const x = enemy.x - cameraX;
  if (x < -40 || x > 1000) return;
  const y = enemy.y + Math.sin(time * 9 + enemy.x / 40) * 2;
  rounded(x + 2, y + 9, 26, 25, 10, "#9c5e86");
  rounded(x + 4, y + 3, 22, 22, 11, "#bd81a0");
  ctx.fillStyle = "#fce5c9";
  ctx.fillRect(x + 8, y + 13, 4, 5);
  ctx.fillRect(x + 19, y + 13, 4, 5);
  ctx.fillStyle = "#3b3a4c";
  ctx.fillRect(x + 9 + enemy.direction, y + 14, 2, 3);
  ctx.fillRect(x + 20 + enemy.direction, y + 14, 2, 3);
  rounded(x, y + 29, 11, 5, 3, "#543e65");
  rounded(x + 19, y + 29, 11, 5, 3, "#543e65");
}

function drawHero(time) {
  const p = world.player;
  if (p.invulnerable > 0 && Math.floor(time * 14) % 2) return;
  const x = Math.round(p.x - cameraX);
  const y = Math.round(p.y);
  const walking = p.grounded && Math.abs(p.vx) > 25;
  const stride = walking ? Math.sin(time * 17) * 4 : 0;
  ctx.save();
  ctx.translate(x + p.w / 2, y);
  ctx.scale(p.facing, 1);
  rounded(-11, 25 + stride, 10, 13, 3, "#243c61");
  rounded(2, 25 - stride, 10, 13, 3, "#243c61");
  rounded(-12, 14, 24, 20, 7, "#167e7e");
  rounded(7, 14, 7, 15, 3, "#eac169");
  rounded(-11, 1, 22, 20, 8, "#ffd6a8");
  // Amber goggles and a small sprout make the playable explorer recognizable.
  rounded(-12, 2, 24, 8, 4, "#294c5c");
  rounded(-8, 3, 8, 6, 3, "#f3b55f");
  rounded(2, 3, 8, 6, 3, "#f3b55f");
  ctx.fillStyle = "#43373c"; ctx.fillRect(4, 13, 2, 3);
  rounded(-12, -3, 24, 9, 5, "#207b70");
  ctx.strokeStyle = "#397c58"; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(0, -3); ctx.quadraticCurveTo(-8, -18, -13, -15); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, -3); ctx.quadraticCurveTo(7, -18, 13, -15); ctx.stroke();
  rounded(-15, 16, 8, 11, 3, "#ffcf83");
  ctx.restore();
}

function drawMarker(x, y, active, finish, time) {
  const screenX = x - cameraX;
  if (screenX < -50 || screenX > 1000) return;
  rounded(screenX + 12, y - 48, 5, finish ? 128 : 108, 2, "#efe8d8");
  ctx.fillStyle = finish ? "#ffe297" : active ? "#95f3d4" : "#d3e5e1";
  ctx.beginPath();
  ctx.moveTo(screenX + 16, y - 47);
  ctx.lineTo(screenX + 58 + Math.sin(time * 4) * 4, y - 35);
  ctx.lineTo(screenX + 16, y - 18);
  ctx.fill();
  if (finish) {
    ctx.fillStyle = "#fff7c9";
    ctx.beginPath(); ctx.arc(screenX + 14, y - 52, 8, 0, Math.PI * 2); ctx.fill();
  }
}

function render(time) {
  backdrop(time);
  for (const block of world.level.solids) drawSolid(block);
  for (const hazard of world.level.hazards) {
    const x = hazard.x - cameraX;
    ctx.fillStyle = "#9d3959";
    for (let j = 0; j < hazard.w; j += 13) {
      ctx.beginPath();
      ctx.moveTo(x + j, hazard.y + hazard.h);
      ctx.lineTo(x + j + 6, hazard.y);
      ctx.lineTo(x + j + 13, hazard.y + hazard.h);
      ctx.fill();
    }
  }
  for (const item of world.coins) drawCoin(item, time);
  for (const enemy of world.enemies) drawEnemy(enemy, time);
  if (world.level.checkpoint) drawMarker(world.level.checkpoint.x,
    world.level.checkpoint.y, world.checkpointActive, false, time);
  drawMarker(world.level.finish.x, world.level.finish.y, false, true, time);
  drawHero(time);
  if (toast && time < toastUntil) {
    rounded(355, 63, 250, 38, 8, "#163847dd");
    ctx.fillStyle = "#fcf1c9";
    ctx.font = "bold 18px system-ui";
    ctx.textAlign = "center";
    ctx.fillText(toast, 480, 89);
  }
}

function frame(now) {
  if (!lastFrame) lastFrame = now;
  const delta = Math.min(0.08, (now - lastFrame) / 1000);
  lastFrame = now;
  if (!paused && world.state === "playing") {
    accumulator = Math.min(0.1, accumulator + delta);
    while (accumulator >= 1 / 120 && world.state === "playing") {
      for (const event of world.step(input(), 1 / 120)) {
        if (event.type === "coin") coinCount++;
        if (event.type === "checkpoint") { toast = "检查点已点亮"; toastUntil = now / 1000 + 1.7; }
        if (event.type === "stomp") { toast = "漂亮的踩踏！"; toastUntil = now / 1000 + 0.9; }
      }
      accumulator -= 1 / 120;
    }
    updateHud();
    if (world.state === "complete") {
      const last = world.levelIndex === world.levels.length - 1;
      message(last ? "最后一盏星灯！" : "星灯点亮！",
        `关卡完成 · 当前得分 ${world.score}`, last ? "完成旅程" : "下一关", advance);
    } else if (world.state === "gameover") {
      message("旅程暂告一段落", `你获得了 ${world.score} 分。再来一次吧！`, "重新开始", reset);
    }
  }
  const desired = Math.max(0, Math.min(world.level.width - 960, world.player.x - 305));
  cameraX += (desired - cameraX) * Math.min(1, delta * 8);
  render(now / 1000);
  requestAnimationFrame(frame);
}

updateHud();
requestAnimationFrame(frame);
