from math import ceil

from aster_game.app.config import Settings
from aster_game.game.components import (
    Character,
    FallComponent,
    HealthComponent,
    InputCommand,
    PendingCommands,
    PhysicsComponent,
    Projectile,
    TransformComponent,
)
from aster_game.game.events import DamageRequest, EventBus, GameplayEvent, ResolvedDamage
from aster_game.game.movement.solver import angle_delta
from aster_game.game.movement.state import (
    CharacterMovementState,
    LifeState,
    LocomotionPhase,
)
from aster_game.game.physics import PhysicsWorld
from aster_game.infrastructure.metrics import RuntimeMetrics


class GameWorld:
    """In-memory authoritative state and systems for exactly one room."""

    def __init__(
        self,
        room_id: str,
        settings: Settings,
        metrics: RuntimeMetrics,
    ) -> None:
        from aster_game.game.systems import (
            AirLifecycleSystem,
            AttackSystem,
            CommandSystem,
            DamageSystem,
            DeathSystem,
            GroundSystem,
            HealthSystem,
            LocomotionPhaseSystem,
            MovementSystem,
            PhysicsStepSystem,
            ProjectileSystem,
            RespawnSystem,
            StateSystem,
        )

        self.room_id = room_id
        self.settings = settings
        self.metrics = metrics
        self.tick_id = 0
        self.characters: dict[int, Character] = {}
        self.player_entities: dict[str, int] = {}
        self.projectiles: dict[int, Projectile] = {}
        self.pending = PendingCommands()
        self.attack_requests: list[int] = []
        self.respawn_requests: set[int] = set()
        self.damage_requests: list[DamageRequest] = []
        self.resolved_damage: list[ResolvedDamage] = []
        self.death_requests: list[DamageRequest] = []
        self.events = EventBus()
        self.next_entity_id = 1
        self.next_projectile_id = 1
        self._spawn_index = 0
        self.physics = PhysicsWorld(settings)
        self.systems = (
            CommandSystem(),
            MovementSystem(),
            PhysicsStepSystem(),
            GroundSystem(),
            AirLifecycleSystem(),
            AttackSystem(),
            ProjectileSystem(),
            DamageSystem(),
            HealthSystem(),
            DeathSystem(),
            RespawnSystem(),
            LocomotionPhaseSystem(),
            StateSystem(),
        )

    def _spawn_position(self, exclude_entity_id: int | None = None) -> tuple[float, float, float]:
        coordinates = (-12.0, -4.0, 4.0, 12.0)
        spawn_points = [
            (x, self.settings.spawn_height, z) for z in coordinates for x in coordinates
        ]
        for offset in range(len(spawn_points)):
            index = (self._spawn_index + offset) % len(spawn_points)
            candidate = spawn_points[index]
            occupied = any(
                abs(character.transform.position[0] - candidate[0])
                < 2.0 * self.settings.character_radius + 0.1
                and abs(character.transform.position[2] - candidate[2])
                < 2.0 * self.settings.character_radius + 0.1
                and character.entity_id != exclude_entity_id
                for character in self.characters.values()
            )
            if not occupied:
                self._spawn_index = (index + 1) % len(spawn_points)
                return candidate
        raise RuntimeError("no unoccupied spawn point is available")

    def add_player(self, player_id: str, player_name: str) -> Character:
        if player_id in self.player_entities:
            raise ValueError("player is already in this room")
        if len(self.characters) >= self.settings.max_players_per_room:
            raise OverflowError("room is full")
        entity_id = self.next_entity_id
        self.next_entity_id += 1
        position = self._spawn_position()
        controller, node_path = self.physics.create_character(entity_id, position)
        character = Character(
            entity_id=entity_id,
            player_id=player_id,
            player_name=player_name,
            transform=TransformComponent(position=position),
            movement=CharacterMovementState(previous_position=position),
            health=HealthComponent(
                max_health=self.settings.max_health,
                current_health=self.settings.max_health,
            ),
            fall=FallComponent(),
            physics=PhysicsComponent(controller=controller, node_path=node_path),
        )
        self.characters[entity_id] = character
        self.player_entities[player_id] = entity_id
        self.publish(
            "character_spawn",
            entity_id=entity_id,
            player_id=player_id,
            position=position,
            tick=self.tick_id,
        )
        return character

    def remove_player(self, player_id: str) -> int | None:
        entity_id = self.player_entities.pop(player_id, None)
        if entity_id is None:
            return None
        character = self.characters.pop(entity_id, None)
        if character is not None:
            self.physics.remove_character(
                character.physics.controller,
                character.physics.node_path,
                character.physics.attached,
            )
        self.pending.inputs.pop(entity_id, None)
        self.pending.jump_requests.discard(entity_id)
        self.pending.attacks = [
            pending_id for pending_id in self.pending.attacks if pending_id != entity_id
        ]
        self.pending.respawns.discard(entity_id)
        self.attack_requests = [
            pending_id for pending_id in self.attack_requests if pending_id != entity_id
        ]
        self.respawn_requests.discard(entity_id)
        self.projectiles = {
            projectile_id: projectile
            for projectile_id, projectile in self.projectiles.items()
            if projectile.owner_entity_id != entity_id
        }
        self.publish("character_left", entity_id=entity_id, player_id=player_id)
        return entity_id

    def queue_input(self, entity_id: int, command: InputCommand) -> bool:
        character = self.characters.get(entity_id)
        if character is None or character.life_state is not LifeState.ALIVE:
            return False
        pending = self.pending.inputs.get(entity_id)
        last_sequence = max(
            character.movement.last_processed_input,
            pending.sequence if pending is not None else -1,
        )
        if command.sequence <= last_sequence:
            return False
        held_before_command = pending.jump if pending is not None else character.movement.jump_held
        if command.jump and not held_before_command:
            self.pending.jump_requests.add(entity_id)
        self.pending.inputs[entity_id] = command
        return True

    def queue_attack(self, entity_id: int) -> bool:
        if entity_id not in self.characters:
            return False
        pending_count = self.pending.attacks.count(entity_id) + self.attack_requests.count(
            entity_id
        )
        if pending_count >= self.settings.max_pending_attacks_per_player:
            return False
        self.pending.attacks.append(entity_id)
        return True

    def queue_respawn(self, entity_id: int) -> bool:
        if entity_id not in self.characters:
            return False
        self.pending.respawns.add(entity_id)
        return True

    def tick(self, dt: float) -> list[GameplayEvent]:
        self.tick_id += 1
        for system in self.systems:
            system.update(self, dt)
        self.metrics.set_room_command_queue_size(
            self.room_id,
            len(self.pending.inputs)
            + len(self.pending.jump_requests)
            + len(self.pending.attacks)
            + len(self.pending.respawns)
            + len(self.attack_requests)
            + len(self.respawn_requests),
        )
        return self.events.drain()

    def respawn_character(self, character: Character) -> None:
        self.physics.remove_character(
            character.physics.controller,
            character.physics.node_path,
            character.physics.attached,
        )
        position = self._spawn_position(exclude_entity_id=character.entity_id)
        controller, node_path = self.physics.create_character(character.entity_id, position)
        character.physics = PhysicsComponent(controller=controller, node_path=node_path)
        character.transform.position = position
        last_processed_input = character.movement.last_processed_input
        character.movement = CharacterMovementState(
            previous_position=position,
            last_processed_input=last_processed_input,
        )
        character.health.current_health = character.health.max_health
        character.health.invulnerable_until_tick = self.tick_id
        character.fall = FallComponent()
        character.life_state = LifeState.ALIVE
        character.action_channels.clear(self.tick_id)
        character.hit_direction = None
        character.hit_region = None
        character.hit_strength = 0.0
        character.hit_source_position = None
        character.previous_channels = None
        character.attack_ready_tick = self.tick_id
        self.publish(
            "respawn",
            entity_id=character.entity_id,
            player_id=character.player_id,
            position=position,
            health=character.health.current_health,
            tick=self.tick_id,
        )

    def entity_id_from_physics_name(self, name: str) -> int | None:
        prefix = "character:"
        if not name.startswith(prefix):
            return None
        try:
            entity_id = int(name[len(prefix) :])
        except ValueError:
            return None
        return entity_id if entity_id in self.characters else None

    def publish(self, event_type: str, **data: object) -> None:
        self.events.publish(GameplayEvent(type=event_type, tick=self.tick_id, data=data))

    def snapshot(self) -> dict[str, object]:
        players = [
            {
                "entity_id": character.entity_id,
                "player_id": character.player_id,
                "player_name": character.player_name,
                "position": character.transform.position,
                "velocity": character.movement.velocity,
                "acceleration": character.movement.acceleration,
                "desired_velocity": character.movement.desired_velocity,
                "desired_move_direction": character.movement.desired_move_direction,
                "current_speed": character.movement.current_speed,
                "horizontal_speed": character.movement.horizontal_speed,
                "vertical_speed": character.movement.vertical_speed,
                "grounded": character.movement.grounded,
                "last_grounded_tick": character.movement.last_grounded_tick,
                "floor_normal": character.movement.floor_normal,
                "floor_distance": (
                    character.movement.floor_distance
                    if character.movement.floor_distance != float("inf")
                    else None
                ),
                "walkable_floor": character.movement.walkable_floor,
                "ground_contact_confirmed": character.movement.ground_contact_confirmed,
                "ground_sample_count": character.movement.ground_sample_count,
                "blocked_move_ticks": character.movement.blocked_move_ticks,
                "slope_angle": character.movement.slope_angle,
                "ground_contact_point": character.movement.ground_contact_point,
                "ground_entity": character.movement.ground_entity,
                "movement_mode": character.movement.movement_mode.value,
                "actual_gait": character.movement.actual_gait.value,
                "gait_phase": character.movement.gait_phase,
                "requested_gait": character.movement.requested_gait.value,
                "character_yaw": character.movement.character_yaw,
                "desired_facing_yaw": character.movement.desired_facing_yaw,
                "angular_velocity": character.movement.angular_velocity,
                "yaw_rate": character.movement.yaw_rate,
                "view_yaw": character.movement.view_yaw,
                "view_pitch": character.movement.view_pitch,
                "aim_yaw": character.movement.aim_yaw,
                "aim_pitch": character.movement.aim_pitch,
                "rotation_mode": character.movement.rotation_mode.value,
                "locomotion_phase": character.movement.locomotion_phase.value,
                "phase_start_tick": character.movement.phase_start_tick,
                "phase_duration_ticks": character.movement.phase_duration_ticks,
                "phase_progress": (
                    max(
                        0.0,
                        min(
                            1.0,
                            (self.tick_id - character.movement.phase_start_tick)
                            / character.movement.phase_duration_ticks,
                        ),
                    )
                    if character.movement.phase_duration_ticks > 0
                    else 0.0
                ),
                "phase_until_tick": character.movement.phase_until_tick,
                "landing_recovery_until_tick": character.movement.landing_recovery_until_tick,
                "turn_angle": character.movement.turn_angle,
                "turn_direction": (
                    character.movement.turn_direction
                    if character.movement.locomotion_phase is LocomotionPhase.TURN_IN_PLACE
                    else "none"
                ),
                "remaining_turn_angle": (
                    angle_delta(
                        character.movement.desired_facing_yaw,
                        character.movement.character_yaw,
                    )
                    if character.movement.locomotion_phase is LocomotionPhase.TURN_IN_PLACE
                    else 0.0
                ),
                "turn_progress": (
                    max(
                        0.0,
                        min(
                            1.0,
                            1.0
                            - abs(
                                angle_delta(
                                    character.movement.desired_facing_yaw,
                                    character.movement.character_yaw,
                                )
                            )
                            / max(1.0, character.movement.turn_angle),
                        ),
                    )
                    if character.movement.locomotion_phase is LocomotionPhase.TURN_IN_PLACE
                    else 0.0
                ),
                "landing_impact_velocity": character.fall.impact_velocity,
                "jump_held": character.movement.jump_held,
                "action_channels": character.action_channels.snapshot(),
                "life_state": character.life_state.value,
                "hit_direction": character.hit_direction,
                "hit_region": character.hit_region,
                "hit_strength": character.hit_strength,
                "hit_source_position": character.hit_source_position,
                "health": character.health.current_health,
                "max_health": character.health.max_health,
                "last_processed_input": character.movement.last_processed_input,
                "last_client_tick": character.movement.last_client_tick,
                "jump_available_tick": character.movement.last_jump_tick
                + ceil(self.settings.jump_cooldown_seconds * self.settings.tick_rate),
            }
            for character in self.characters.values()
        ]
        projectiles = [
            {
                "projectile_id": projectile.projectile_id,
                "owner_entity_id": projectile.owner_entity_id,
                "position": projectile.position,
            }
            for projectile in self.projectiles.values()
        ]
        return {
            "type": "snapshot",
            "tick": self.tick_id,
            "players": players,
            "projectiles": projectiles,
        }

    def close(self) -> None:
        self.physics.close()
        self.metrics.remove_room(self.room_id)
        self.characters.clear()
        self.player_entities.clear()
        self.projectiles.clear()
