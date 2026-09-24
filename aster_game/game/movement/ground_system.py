from __future__ import annotations

from math import acos, degrees
from typing import TYPE_CHECKING

from aster_game.game.movement.state import LifeState, MovementMode

if TYPE_CHECKING:
    from aster_game.game.world import GameWorld


class GroundSystem:
    """Resolve stable floor samples around the authoritative capsule."""

    def update(self, world: GameWorld, dt: float) -> None:
        settings = world.settings
        for character in world.characters.values():
            movement = character.movement
            if character.life_state is not LifeState.ALIVE:
                continue

            previous_grounded = movement.grounded
            previous_ground_sample = (
                movement.floor_normal,
                movement.floor_distance,
                movement.walkable_floor,
                movement.ground_contact_point,
                movement.ground_entity,
                movement.slope_angle,
                movement.ground_sample_count,
            )
            samples = world.physics.probe_ground(character.transform.position)
            center = next((sample for sample in samples if sample.probe == "center"), None)
            selected = center or min(samples, key=lambda sample: sample.distance, default=None)
            if center is not None:
                raised_support = [
                    sample
                    for sample in samples
                    if sample.probe != "center"
                    and 0.02
                    < sample.position[1] - center.position[1]
                    <= settings.character_step_height + 0.02
                    and -settings.ground_probe_radius <= sample.distance
                    and sample.distance <= settings.ground_snap_distance
                ]
                if raised_support:
                    selected = max(raised_support, key=lambda sample: sample.position[1])
            controller_grounded = character.physics.controller.isOnGround()
            snapped = False

            if selected is None:
                movement.floor_distance = float("inf")
                movement.floor_normal = (0.0, 1.0, 0.0)
                movement.ground_contact_point = None
                movement.ground_entity = None
                movement.slope_angle = 0.0
                movement.walkable_floor = False
                movement.ground_sample_count = 0
            else:
                normal = selected.normal
                slope_angle = degrees(acos(max(-1.0, min(1.0, normal[1]))))
                movement.floor_distance = selected.distance
                movement.floor_normal = normal
                movement.ground_contact_point = selected.position
                movement.ground_entity = selected.node_name
                movement.slope_angle = slope_angle
                movement.walkable_floor = slope_angle <= settings.max_walkable_slope
                movement.ground_sample_count = sum(
                    1
                    for sample in samples
                    if degrees(acos(max(-1.0, min(1.0, sample.normal[1]))))
                    <= settings.max_walkable_slope
                    and sample.distance >= -settings.ground_probe_radius
                    and sample.distance <= settings.ground_snap_distance
                )
                if (
                    movement.walkable_floor
                    and selected.distance >= -settings.ground_probe_radius
                    and selected.distance <= settings.ground_snap_distance
                    and movement.vertical_speed <= 0.0
                    and (selected.distance > 1e-4 or not controller_grounded)
                ):
                    fall = character.fall
                    if fall.airborne:
                        fall.last_vertical_velocity = movement.vertical_speed
                    position = character.physics.node_path.getPos()
                    character.physics.node_path.setY(position.y - selected.distance)
                    snapped_position = character.physics.node_path.getPos()
                    character.transform.position = (
                        float(snapped_position.x),
                        float(snapped_position.y),
                        float(snapped_position.z),
                    )
                    movement.floor_distance = 0.0
                    movement.velocity = (movement.velocity[0], 0.0, movement.velocity[2])
                    movement.vertical_speed = 0.0
                    snapped = True

            center_supported = (
                center is not None
                and degrees(acos(max(-1.0, min(1.0, center.normal[1]))))
                <= settings.max_walkable_slope
                and center.distance >= -settings.ground_probe_radius
                and center.distance <= settings.ground_snap_distance
            )
            peripheral_support_count = sum(
                1
                for sample in samples
                if sample.probe != "center"
                and degrees(acos(max(-1.0, min(1.0, sample.normal[1]))))
                <= settings.max_walkable_slope
                and sample.distance >= -settings.ground_probe_radius
                and sample.distance <= settings.ground_snap_distance
            )
            contact_confirmed = movement.walkable_floor and (
                snapped
                or controller_grounded and (center_supported or peripheral_support_count >= 3)
            )
            if contact_confirmed:
                movement.last_grounded_tick = world.tick_id

            grace_active = (
                not contact_confirmed
                and selected is None
                and previous_grounded
                and movement.last_grounded_tick >= 0
                and world.tick_id - movement.last_grounded_tick <= settings.ground_grace_ticks
                and previous_ground_sample[1] <= settings.ground_grace_distance
            )
            if grace_active:
                (
                    movement.floor_normal,
                    movement.floor_distance,
                    movement.walkable_floor,
                    movement.ground_contact_point,
                    movement.ground_entity,
                    movement.slope_angle,
                    movement.ground_sample_count,
                ) = previous_ground_sample

            movement.ground_contact_confirmed = contact_confirmed
            movement.grounded = contact_confirmed or grace_active
            movement.movement_mode = (
                MovementMode.GROUNDED if movement.grounded else MovementMode.AIRBORNE
            )

            fall = character.fall
            if previous_grounded and not movement.grounded and not fall.airborne:
                fall.airborne = True
                fall.apex_reached = False
                fall.jump_started = False
                fall.start_y = movement.previous_position[1]
                fall.start_tick = world.tick_id
                fall.last_vertical_velocity = movement.velocity[1]
                fall.impact_velocity = 0.0
            elif (
                not movement.grounded
                and not fall.airborne
                and movement.vertical_speed < -settings.apex_velocity_threshold
            ):
                fall.airborne = True
                fall.apex_reached = False
                fall.jump_started = False
                fall.start_y = movement.previous_position[1]
                fall.start_tick = world.tick_id
                fall.last_vertical_velocity = movement.vertical_speed
                fall.impact_velocity = 0.0
            elif not movement.grounded and fall.airborne:
                fall.last_vertical_velocity = movement.velocity[1]
