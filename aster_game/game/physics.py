from dataclasses import dataclass
from math import cos, hypot, radians

from panda3d.bullet import (
    BulletBoxShape,
    BulletCapsuleShape,
    BulletCharacterControllerNode,
    BulletPlaneShape,
    BulletRigidBodyNode,
    BulletSphereShape,
    BulletWorld,
    YUp,
    getDefaultUpAxis,
)
from panda3d.core import BitMask32, NodePath, Point3, TransformState, Vec3, loadPrcFileData

from aster_game.app.config import Settings


@dataclass(frozen=True, slots=True)
class SweepHit:
    node_name: str
    position: tuple[float, float, float]
    fraction: float


@dataclass(frozen=True, slots=True)
class GroundProbeHit:
    probe: str
    node_name: str
    distance: float
    normal: tuple[float, float, float]
    position: tuple[float, float, float]


class PhysicsWorld:
    """Headless Bullet world owned exclusively by one game room."""

    def __init__(self, settings: Settings) -> None:
        if getDefaultUpAxis() != YUp:
            loadPrcFileData("aster-game", "coordinate-system y-up-right")
        if getDefaultUpAxis() != YUp:
            raise RuntimeError("Panda3D Bullet must use the Y-up coordinate system")

        self.settings = settings
        self.world = BulletWorld()
        self.world.setGravity(Vec3(0.0, -settings.gravity, 0.0))
        self.root = NodePath("world-root")
        self._projectile_shape = BulletSphereShape(settings.projectile_radius)
        self._ground_probe_shape = BulletSphereShape(settings.ground_probe_radius)
        self._build_arena()

    def _add_static_box(
        self,
        name: str,
        center: tuple[float, float, float],
        half_extents: tuple[float, float, float],
    ) -> None:
        body = BulletRigidBodyNode(name)
        body.addShape(BulletBoxShape(Vec3(*half_extents)))
        path = self.root.attachNewNode(body)
        path.setPos(*center)
        self.world.attach(body)

    def _build_arena(self) -> None:
        floor = BulletRigidBodyNode("arena-floor")
        floor.addShape(BulletPlaneShape(Vec3(0.0, 1.0, 0.0), 0.0))
        floor_path = self.root.attachNewNode(floor)
        self.world.attach(floor)

        extent = self.settings.arena_half_extent
        wall_height = 3.0
        wall_thickness = 0.75
        self._add_static_box(
            "arena-wall-north",
            (0.0, wall_height / 2, extent),
            (extent, wall_height / 2, wall_thickness),
        )
        self._add_static_box(
            "arena-wall-south",
            (0.0, wall_height / 2, -extent),
            (extent, wall_height / 2, wall_thickness),
        )
        self._add_static_box(
            "arena-wall-east",
            (extent, wall_height / 2, 0.0),
            (wall_thickness, wall_height / 2, extent),
        )
        self._add_static_box(
            "arena-wall-west",
            (-extent, wall_height / 2, 0.0),
            (wall_thickness, wall_height / 2, extent),
        )
        self._add_static_box("cover-center", (0.0, 0.9, 0.0), (1.1, 0.9, 1.1))
        ramp = BulletRigidBodyNode("walkable-ramp")
        ramp.addShape(BulletBoxShape(Vec3(2.0, 0.15, 2.0)))
        ramp_path = self.root.attachNewNode(ramp)
        ramp_path.setPos(8.0, 0.95, -4.0)
        ramp_path.setP(-20.0)
        self.world.attach(ramp)
        step_rise = 0.3
        step_spacing = 0.5
        step_start_z = 5.0
        step_count = 24
        for index in range(step_count):
            top = step_rise * (index + 1)
            self._add_static_box(
                f"upper-platform-step-{index + 1}",
                (0.0, top - step_rise / 2.0, step_start_z + index * step_spacing),
                (1.75, step_rise / 2.0, 0.4),
            )
        platform_top = step_rise * step_count
        self._add_static_box(
            "upper-platform",
            (0.0, platform_top - 0.2, 18.0),
            (4.0, 0.2, 2.5),
        )
        # Keep strong references to Panda nodes while the Bullet world is alive.
        self._floor_node = floor
        self._floor_path = floor_path
        self._ramp_node = ramp
        self._ramp_path = ramp_path

    def create_character(
        self, entity_id: int, position: tuple[float, float, float]
    ) -> tuple[BulletCharacterControllerNode, NodePath]:
        shape = BulletCapsuleShape(
            self.settings.character_radius,
            self.settings.character_cylinder_height,
            YUp,
        )
        controller = BulletCharacterControllerNode(
            shape, self.settings.character_step_height, f"character:{entity_id}"
        )
        controller.setGravity(self.settings.gravity)
        controller.setFallSpeed(self.settings.max_fall_speed)
        controller.setJumpSpeed(self.settings.jump_speed)
        controller.setMaxSlope(self.settings.max_walkable_slope)
        controller.setUseGhostSweepTest(True)
        path = self.root.attachNewNode(controller)
        path.setPos(*position)
        self.world.attach(controller)
        return controller, path

    def detach_character(self, controller: BulletCharacterControllerNode) -> None:
        if controller in self.world.getCharacters():
            self.world.remove(controller)

    def remove_character(
        self,
        controller: BulletCharacterControllerNode,
        path: NodePath,
        attached: bool = True,
    ) -> None:
        if attached:
            self.detach_character(controller)
        path.removeNode()

    def step(self, dt: float) -> None:
        self.world.doPhysics(dt, 1, dt)

    def sweep_projectile(
        self,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
    ) -> SweepHit | None:
        start_transform = TransformState.makePos(Point3(*start))
        end_transform = TransformState.makePos(Point3(*end))
        result = self.world.sweepTestClosest(
            self._projectile_shape,
            start_transform,
            end_transform,
            BitMask32.allOn(),
            0.0,
        )
        if not result.hasHit():
            return None
        hit_pos = result.getHitPos()
        return SweepHit(
            node_name=result.getNode().getName(),
            position=(float(hit_pos.x), float(hit_pos.y), float(hit_pos.z)),
            fraction=float(result.getHitFraction()),
        )

    def probe_ground(
        self, position: tuple[float, float, float]
    ) -> tuple[GroundProbeHit, ...]:
        half_height = self.settings.character_radius + self.settings.character_cylinder_height / 2.0
        feet_y = position[1] - half_height
        radius = self.settings.character_radius * 0.9
        probes = (
            ("center", 0.0, 0.0),
            ("front", 0.0, radius),
            ("back", 0.0, -radius),
            ("right", radius, 0.0),
            ("left", -radius, 0.0),
        )
        hits: list[GroundProbeHit] = []
        for name, offset_x, offset_z in probes:
            start = Point3(
                position[0] + offset_x,
                feet_y + self.settings.ground_probe_start_offset,
                position[2] + offset_z,
            )
            end = Point3(
                position[0] + offset_x,
                feet_y - self.settings.ground_probe_depth,
                position[2] + offset_z,
            )
            result = self.world.sweepTestClosest(
                self._ground_probe_shape,
                TransformState.makePos(start),
                TransformState.makePos(end),
                BitMask32.allOn(),
                0.0,
            )
            if not result.hasHit():
                continue
            node = result.getNode()
            node_name = node.getName()
            if node_name.startswith("character:"):
                continue
            normal = result.getHitNormal()
            contact = result.getHitPos()
            if normal.y <= 0.0:
                continue
            hits.append(
                GroundProbeHit(
                    probe=name,
                    node_name=node_name,
                    distance=float(feet_y - contact.y),
                    normal=(float(normal.x), float(normal.y), float(normal.z)),
                    position=(float(contact.x), float(contact.y), float(contact.z)),
                )
            )
        return tuple(hits)

    def find_step_up_target(
        self,
        current_position: tuple[float, float, float],
        proposed_position: tuple[float, float, float],
        current_floor_y: float,
    ) -> tuple[tuple[float, float, float], GroundProbeHit] | None:
        if self.settings.character_step_height <= 0.0:
            return None
        delta_x = proposed_position[0] - current_position[0]
        delta_z = proposed_position[2] - current_position[2]
        horizontal_distance = hypot(delta_x, delta_z)
        if horizontal_distance <= 1e-6:
            return None
        step_forward = self.settings.character_radius + self.settings.ground_probe_radius
        target_x = proposed_position[0] + delta_x / horizontal_distance * step_forward
        target_z = proposed_position[2] + delta_z / horizontal_distance * step_forward
        half_height = self.settings.character_radius + self.settings.character_cylinder_height / 2.0
        target_position = (
            target_x,
            proposed_position[1] + self.settings.character_step_height,
            target_z,
        )
        minimum_normal_y = cos(radians(self.settings.max_walkable_slope))
        candidates = [
            sample
            for sample in self.probe_ground(target_position)
            if sample.distance >= -self.settings.ground_probe_radius
            and sample.distance <= self.settings.ground_snap_distance
            and sample.normal[1] >= minimum_normal_y
        ]
        if not candidates:
            return None
        support = max(candidates, key=lambda sample: sample.position[1])
        rise = support.position[1] - current_floor_y
        if rise <= 0.02 or rise > self.settings.character_step_height + 0.02:
            return None
        return (
            (
                target_x,
                support.position[1] + half_height,
                target_z,
            ),
            support,
        )

    def close(self) -> None:
        for controller in list(self.world.getCharacters()):
            self.world.remove(controller)
        for body in list(self.world.getRigidBodies()):
            self.world.remove(body)
        self.root.removeNode()
