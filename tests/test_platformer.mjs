import test from "node:test";
import assert from "node:assert/strict";

import { LEVELS } from "../aster_game/web/platformer/levels.mjs";
import { PlatformerWorld } from "../aster_game/web/platformer/world.mjs";

const dt = 1 / 120;
const advance = (world, frames, input = {}) => {
  let events = [];
  for (let i = 0; i < frames; i++) events = events.concat(world.step(input, dt));
  return events;
};

test("all authored levels have a ground-supported spawn, checkpoint and finish", () => {
  assert.equal(LEVELS.length, 3);
  for (const level of LEVELS) {
    assert.ok(level.solids.some((solid) => level.spawn.x >= solid.x &&
      level.spawn.x + 26 <= solid.x + solid.w && level.spawn.y + 38 === solid.y));
    assert.ok(level.solids.some((solid) => level.finish.x >= solid.x &&
      level.finish.x + 35 <= solid.x + solid.w && level.finish.y + 80 === solid.y));
    assert.ok(level.checkpoint && level.checkpoint.x > level.spawn.x &&
      level.checkpoint.x < level.finish.x);
  }
});

test("held jump rises farther than a released jump and horizontal speed is bounded", () => {
  const held = new PlatformerWorld();
  const released = new PlatformerWorld();
  held.enemies = [];
  released.enemies = [];
  advance(held, 4);
  advance(released, 4);
  advance(held, 24, { jump: true });
  advance(released, 1, { jump: true });
  advance(released, 23);
  assert.ok(held.player.y < released.player.y - 35);
  advance(held, 180, { right: true });
  assert.ok(held.player.vx <= 300 && held.player.vx >= 0);
});

test("a buffered jump fires on landing and coyote time permits a late jump", () => {
  const world = new PlatformerWorld();
  world.enemies = [];
  world.player.x = 180;
  world.player.y = 440;
  world.player.vy = 140;
  world.player.grounded = false;
  let events = world.step({ jump: true }, dt);
  assert.equal(events.length, 0);
  events = events.concat(advance(world, 15, { jump: true }));
  assert.ok(events.some((event) => event.type === "jump"));
  assert.ok(world.player.vy < 0);

  const late = new PlatformerWorld();
  late.enemies = [];
  advance(late, 2);
  late.player.x = 654; // just beyond the first ground edge
  late.player.grounded = false;
  late.player.y = 462;
  assert.ok(late.step({ jump: true }, dt).some((event) => event.type === "jump"));
});

test("jump can cross the opening but walking into it eventually causes a fall", () => {
  const jumper = new PlatformerWorld();
  jumper.enemies = [];
  jumper.player.x = 580;
  advance(jumper, 2);
  advance(jumper, 80, { right: true, jump: true });
  assert.ok(jumper.player.x > 760 && jumper.lives === 3);

  const walker = new PlatformerWorld();
  walker.enemies = [];
  walker.player.x = 580;
  const events = advance(walker, 150, { right: true });
  assert.ok(events.some((event) => event.type === "damage" && event.source === "fall"));
  assert.equal(walker.lives, 2);
});

test("every authored gap has a feasible running-jump takeoff", () => {
  for (const [index, level] of LEVELS.entries()) {
    const grounds = level.solids.filter((solid) => solid.kind === "ground");
    for (let gap = 0; gap < grounds.length - 1; gap++) {
      const edge = grounds[gap].x + grounds[gap].w;
      const farSide = grounds[gap + 1].x;
      let reachable = false;
      for (let start = edge - 170; start <= edge - 20 && !reachable; start += 10) {
        const world = new PlatformerWorld();
        world.levelIndex = index;
        world.restartLevel();
        world.enemies = [];
        world.level = { ...world.level, hazards: [] };
        world.player.x = start;
        world.player.y = 462;
        advance(world, 125, { right: true, jump: true });
        reachable = world.lives === 3 && world.player.x >= farSide + 25;
      }
      assert.ok(reachable, `${level.id}: gap ${edge}–${farSide} cannot be crossed`);
    }
  }
});

test("stomping defeats an enemy while lateral contact costs a life", () => {
  const stomp = new PlatformerWorld();
  stomp.enemies = [{ x: 200, y: 466, w: 29, h: 34, speed: 0, minX: 200, maxX: 200,
    direction: 1, alive: true }];
  stomp.player.x = 200;
  stomp.player.y = 400;
  stomp.player.vy = 420;
  const events = advance(stomp, 16);
  assert.ok(events.some((event) => event.type === "stomp"));
  assert.equal(stomp.enemies[0].alive, false);
  assert.equal(stomp.lives, 3);

  const hit = new PlatformerWorld();
  hit.enemies = [{ x: 100, y: 466, w: 29, h: 34, speed: 0, minX: 100, maxX: 100,
    direction: 1, alive: true }];
  advance(hit, 30, { right: true });
  assert.equal(hit.lives, 2);
  advance(hit, 30, { right: true });
  assert.equal(hit.lives, 2); // respawn invulnerability prevents immediate repeated damage
});

test("coins, checkpoint respawn and progression retain score and lives", () => {
  const world = new PlatformerWorld();
  world.enemies = [];
  world.player.x = 315;
  world.player.y = 345;
  advance(world, 1);
  assert.equal(world.score, 10);
  advance(world, 1);
  assert.equal(world.score, 10);
  world.player.x = 1080;
  world.player.y = 462;
  assert.ok(world.step({}, dt).some((event) => event.type === "checkpoint"));
  world.damage("fall");
  assert.equal(world.player.x, 1080);
  assert.equal(world.lives, 2);
  world.player.x = world.level.finish.x;
  world.player.y = 462;
  advance(world, 1);
  assert.equal(world.state, "complete");
  assert.equal(world.score, 110);
  assert.equal(world.nextLevel(), true);
  assert.equal(world.levelIndex, 1);
  assert.equal(world.lives, 2);
  assert.equal(world.score, 110);
});

test("game over requires an explicit restart", () => {
  const world = new PlatformerWorld();
  for (let i = 0; i < 3; i++) {
    world.player.invulnerable = 0;
    world.damage("hazard");
  }
  assert.equal(world.state, "gameover");
  advance(world, 30, { right: true });
  assert.equal(world.state, "gameover");
  world.restartGame();
  assert.equal(world.lives, 3);
  assert.equal(world.state, "playing");
});
