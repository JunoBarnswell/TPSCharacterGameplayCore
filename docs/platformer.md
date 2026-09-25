# Original platform adventure: first playable slice

## Scope

`/platformer` is an independent game mode built beside the TPS motion lab. Its hero A-Ya, star
lanterns, patrol creatures and three levels are original canvas art and level data. No TPS protocol
messages are used: the prototype is local and single-player.

## Gameplay contract

- Run left/right, jump with a short input buffer and ground grace, release jump early for a lower arc.
- Patrol creatures damage the hero on contact; landing from above defeats them and rebounds the hero.
- Collect coins once for points. Spike beds and falling off the map cost one life.
- Touch a checkpoint banner to update the respawn point. Damage gives a short invulnerability window.
- Touch the star lantern to complete a level; continue through three levels and restart after victory.
- A fixed 120 Hz simulation with a capped browser catch-up loop keeps physics independent of display FPS.

## Extension points

`levels.mjs` declares geometry and encounters. `world.mjs` owns simulation, collision, state and
events without DOM dependencies. `game.mjs` owns controls, camera, rendering and HUD. Add new levels
as data and validate playable routes in the automated movement tests. Future online play would need a
separate authoritative platformer simulation; the current TPS room and capsule solver are designed
for third-person combat and should not be presented as a multiplayer platformer backend.

## Verify

```sh
node --test tests/test_platformer.mjs
node --test tests/test_web_motion.mjs
uv run --locked pytest -q
uv run --locked ruff check .
```
