import { LEVELS } from "./levels.mjs";

const TUNING = Object.freeze({
  acceleration: 2400,
  braking: 2800,
  speed: 300,
  gravity: 1720,
  fallSpeed: 890,
  jumpSpeed: 650,
  jumpCut: 0.48,
  coyoteTime: 0.105,
  jumpBuffer: 0.115,
  stompBounce: 395,
});

const overlap = (a, b) => a.x < b.x + b.w && a.x + a.w > b.x &&
  a.y < b.y + b.h && a.y + a.h > b.y;
const approach = (value, target, amount) => value < target
  ? Math.min(target, value + amount) : Math.max(target, value - amount);

function validateLevel(level) {
  if (!level || !Number.isFinite(level.width) || level.width < 960 ||
      !level.spawn || !level.finish || !Array.isArray(level.solids) ||
      !Array.isArray(level.enemies) || !Array.isArray(level.coins) ||
      !Array.isArray(level.hazards)) {
    throw new TypeError("a level requires bounds, spawn, finish, solids, enemies, coins and hazards");
  }
  const within = (item) => Number.isFinite(item.x) && Number.isFinite(item.y) &&
    item.x >= 0 && item.x < level.width && item.y >= 0 && item.y < 560;
  if (![level.spawn, level.finish, ...level.solids, ...level.enemies,
    ...level.coins, ...level.hazards, ...(level.checkpoint ? [level.checkpoint] : [])]
    .every(within)) throw new RangeError(`level '${level.id}' has out-of-bounds content`);
}

export class PlatformerWorld {
  constructor(levels = LEVELS) {
    if (!Array.isArray(levels) || !levels.length) throw new TypeError("levels must not be empty");
    levels.forEach(validateLevel);
    this.levels = levels;
    this.lives = 3;
    this.score = 0;
    this.levelIndex = 0;
    this.restartLevel();
  }

  restartLevel() {
    this.level = this.levels[this.levelIndex];
    this.player = { x: this.level.spawn.x, y: this.level.spawn.y, w: 26, h: 38,
      vx: 0, vy: 0, grounded: false, facing: 1, invulnerable: 0 };
    this.enemies = this.level.enemies.map((item) => ({ ...item, w: 29, h: 34, direction: 1,
      alive: true }));
    this.coins = this.level.coins.map((item) => ({ ...item, collected: false }));
    this.checkpointActive = false;
    this.coyote = 0;
    this.jumpBuffer = 0;
    this.previousJump = false;
    this.state = "playing";
    this.events = [];
    this.time = 0;
  }

  nextLevel() {
    if (this.state !== "complete") return false;
    if (this.levelIndex === this.levels.length - 1) {
      this.state = "victory";
      return false;
    }
    this.levelIndex++;
    this.restartLevel();
    return true;
  }

  restartGame() {
    this.lives = 3;
    this.score = 0;
    this.levelIndex = 0;
    this.restartLevel();
  }

  step(input = {}, dt = 1 / 60) {
    if (!Number.isFinite(dt) || dt < 0 || dt > 0.1) {
      throw new RangeError("simulation step must be between 0 and 0.1 seconds");
    }
    this.events = [];
    if (this.state !== "playing" || dt === 0) return this.events;
    const jump = Boolean(input.jump);
    if (jump && !this.previousJump) this.jumpBuffer = TUNING.jumpBuffer;
    this.previousJump = jump;
    const steps = Math.ceil(dt / (1 / 120));
    for (let index = 0; index < steps; index++) {
      this.substep(input, dt / steps);
      if (this.state !== "playing") break;
    }
    return this.events;
  }

  substep(input, dt) {
    const p = this.player;
    this.time += dt;
    p.invulnerable = Math.max(0, p.invulnerable - dt);
    this.coyote = Math.max(0, this.coyote - dt);
    this.jumpBuffer = Math.max(0, this.jumpBuffer - dt);
    const direction = Number(Boolean(input.right)) - Number(Boolean(input.left));
    if (direction) p.facing = direction;
    p.vx = approach(p.vx, direction * TUNING.speed,
      (direction ? TUNING.acceleration : TUNING.braking) * dt);
    if (this.jumpBuffer > 0 && (p.grounded || this.coyote > 0)) {
      p.vy = -TUNING.jumpSpeed;
      p.grounded = false;
      this.coyote = 0;
      this.jumpBuffer = 0;
      this.events.push({ type: "jump" });
    }
    if (!input.jump && p.vy < -TUNING.jumpSpeed * TUNING.jumpCut) {
      p.vy = -TUNING.jumpSpeed * TUNING.jumpCut;
    }
    p.vy = Math.min(TUNING.fallSpeed, p.vy + TUNING.gravity * dt);

    p.x += p.vx * dt;
    p.x = Math.max(0, Math.min(this.level.width - p.w, p.x));
    for (const solid of this.level.solids) {
      if (!overlap(p, solid)) continue;
      if (p.vx > 0) p.x = solid.x - p.w;
      else if (p.vx < 0) p.x = solid.x + solid.w;
      p.vx = 0;
    }

    const oldBottom = p.y + p.h;
    const oldTop = p.y;
    p.y += p.vy * dt;
    p.grounded = false;
    for (const solid of this.level.solids) {
      if (!overlap(p, solid)) continue;
      if (p.vy >= 0 && oldBottom <= solid.y + 1) {
        p.y = solid.y - p.h;
        p.vy = 0;
        p.grounded = true;
        this.coyote = TUNING.coyoteTime;
      } else if (p.vy < 0 && oldTop >= solid.y + solid.h - 1) {
        p.y = solid.y + solid.h;
        p.vy = 0;
      }
    }
    // Allow a buffered jump to fire on the frame that first touches the floor.
    if (p.grounded && this.jumpBuffer > 0) {
      p.vy = -TUNING.jumpSpeed;
      p.grounded = false;
      this.coyote = 0;
      this.jumpBuffer = 0;
      this.events.push({ type: "jump" });
    }
    if (p.y > 650) {
      this.damage("fall");
      return;
    }
    for (const item of this.coins) {
      if (!item.collected && overlap(p, { x: item.x - 10, y: item.y - 10, w: 20, h: 20 })) {
        item.collected = true;
        this.score += 10;
        this.events.push({ type: "coin", score: this.score });
      }
    }
    for (const enemy of this.enemies) {
      if (!enemy.alive) continue;
      enemy.x += enemy.direction * enemy.speed * dt;
      if (enemy.x <= enemy.minX || enemy.x >= enemy.maxX) {
        enemy.x = Math.max(enemy.minX, Math.min(enemy.maxX, enemy.x));
        enemy.direction *= -1;
      }
      if (!overlap(p, enemy)) continue;
      if (p.vy > 100 && oldBottom <= enemy.y + 12) {
        enemy.alive = false;
        p.y = enemy.y - p.h;
        p.vy = -TUNING.stompBounce;
        this.score += 25;
        this.events.push({ type: "stomp", score: this.score });
      } else this.damage("enemy");
    }
    for (const hazard of this.level.hazards) {
      if (overlap(p, hazard)) this.damage("hazard");
    }
    const checkpoint = this.level.checkpoint;
    if (checkpoint && !this.checkpointActive && overlap(p,
      { x: checkpoint.x, y: checkpoint.y, w: 28, h: 60 })) {
      this.checkpointActive = true;
      this.events.push({ type: "checkpoint" });
    }
    if (overlap(p, { x: this.level.finish.x, y: this.level.finish.y, w: 35, h: 80 })) {
      this.state = "complete";
      this.score += 100;
      this.events.push({ type: "finish", score: this.score });
    }
  }

  damage(source) {
    if (this.player.invulnerable > 0 || this.state !== "playing") return;
    this.lives--;
    this.events.push({ type: "damage", source, lives: this.lives });
    if (this.lives <= 0) {
      this.state = "gameover";
      return;
    }
    const point = this.checkpointActive ? this.level.checkpoint : this.level.spawn;
    Object.assign(this.player, { x: point.x, y: point.y, vx: 0, vy: 0,
      grounded: false, invulnerable: 1.4 });
    this.coyote = 0;
    this.jumpBuffer = 0;
  }
}
