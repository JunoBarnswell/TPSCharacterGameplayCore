from dataclasses import dataclass
from importlib.resources import files
from json import loads

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
        self.collision_profile = self.build_collision_profile(settings)
        self._build_arena()

    @staticmethod
    def build_collision_profile(settings: Settings) -> dict[str, object]:
        resource = files("aster_game").joinpath("web/motion/arena-collision.json")
        profile = loads(resource.read_text(encoding="utf-8"))
        if profile["version"] != 1:
            raise RuntimeError(f"unsupported arena collision profile: {profile['version']}")
        extent = settings.arena_half_extent
        wall_height = float(profile["wall_height"])
        wall_thickness = float(profile["wall_thickness"])
        boxes = [
            {
                "name": "arena-wall-north",
                "center": [0.0, wall_height / 2.0, extent],
                "half_extents": [extent, wall_height / 2.0, wall_thickness],
            },
            {
                "name": "arena-wall-south",
                "center": [0.0, wall_height / 2.0, -extent],
                "half_extents": [extent, wall_height / 2.0, wall_thickness],
            },
            {
                "name": "arena-wall-east",
                "center": [extent, wall_height / 2.0, 0.0],
                "half_extents": [wall_thickness, wall_height / 2.0, extent],
            },
            {
                "name": "arena-wall-west",
                "center": [-extent, wall_height / 2.0, 0.0],
                "half_extents": [wall_thickness, wall_height / 2.0, extent],
            },
            *profile["boxes"],
        ]
        staircase = profile["staircase"]
        rise = float(staircase["step_rise"])
        for index in range(int(staircase["step_count"])):
            top = rise * (index + 1)
            boxes.append(
                {
                    "name": f"upper-platform-step-{index + 1}",
                    "center": [
                        0.0,
                        top - rise / 2.0,
                        float(staircase["step_start_z"])
                        + index * float(staircase["step_spacing"]),
                    ],
                    "half_extents": [
                        float(staircase["step_half_width"]),
                        rise / 2.0,
                        float(staircase["step_half_depth"]),
                    ],
                }
            )
        boxes.append(
            {
                "name": "upper-platform",
                "center": staircase["platform_center"],
                "half_extents": staircase["platform_half_extents"],
            }
        )
        return {
            "version": 1,
            "planes": [{"name": "arena-floor", "normal": [0.0, 1.0, 0.0], "constant": 0.0}],
            "boxes": boxes,
            "ramps": profile["ramps"],
        }

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
        self._static_bodies.append(body)
        self._static_paths.append(path)

    def _build_arena(self) -> None:
        self._static_bodies: list[BulletRigidBodyNode] = []
        self._static_paths: list[NodePath] = []
        for plane in self.collision_profile["planes"]:
            floor = BulletRigidBodyNode(plane["name"])
            floor.addShape(BulletPlaneShape(Vec3(*plane["normal"]), plane["constant"]))
            floor_path = self.root.attachNewNode(floor)
            self.world.attach(floor)
            self._static_bodies.append(floor)
            self._static_paths.append(floor_path)
        for box in self.collision_profile["boxes"]:
            center = tuple(box["center"])
            half_extents = tuple(box["half_extents"])
            self._add_static_box(box["name"], center, half_extents)
        for ramp in self.collision_profile["ramps"]:
            body = BulletRigidBodyNode(ramp["name"])
            body.addShape(BulletBoxShape(Vec3(*ramp["half_extents"])))
            path = self.root.attachNewNode(body)
            path.setPos(*ramp["center"])
            path.setP(ramp["pitch_degrees"])
            self.world.attach(body)
            self._static_bodies.append(body)
            self._static_paths.append(path)
        # Keep strong references to Panda nodes while the Bullet world is alive.

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
        *,
        ignore_node_name: str | None = None,
    ) -> SweepHit | None:
        start_transform = TransformState.makePos(Point3(*start))
        end_transform = TransformState.makePos(Point3(*end))
        ignored_controller = next(
            (
                controller
                for controller in self.world.getCharacters()
                if controller.getName() == ignore_node_name
            ),
            None,
        )
        previous_mask = (
            ignored_controller.getIntoCollideMask() if ignored_controller is not None else None
        )
        try:
            # Bullet exposes only the closest-hit sweep query. Temporarily removing the
            # owner's into mask lets the same sweep continue to the next real obstacle.
            if ignored_controller is not None:
                ignored_controller.setIntoCollideMask(BitMask32.allOff())
            result = self.world.sweepTestClosest(
                self._projectile_shape,
                start_transform,
                end_transform,
                BitMask32.allOn(),
                0.0,
            )
        finally:
            if ignored_controller is not None and previous_mask is not None:
                ignored_controller.setIntoCollideMask(previous_mask)
        if not result.hasHit():
            return None
        hit_pos = result.getHitPos()
        return SweepHit(
            node_name=result.getNode().getName(),
            position=(float(hit_pos.x), float(hit_pos.y), float(hit_pos.z)),
            fraction=float(result.getHitFraction()),
        )

    def raycast(
        self,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        *,
        ignore_node_name: str | None = None,
    ) -> SweepHit | None:
        result = self.world.rayTestAll(Point3(*start), Point3(*end), BitMask32.allOn())
        hits = sorted(result.getHits(), key=lambda hit: hit.getHitFraction())
        for hit in hits:
            node_name = hit.getNode().getName()
            if node_name == ignore_node_name:
                continue
            hit_pos = hit.getHitPos()
            return SweepHit(
                node_name=node_name,
                position=(float(hit_pos.x), float(hit_pos.y), float(hit_pos.z)),
                fraction=float(hit.getHitFraction()),
            )
        return None

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

    def close(self) -> None:
        for controller in list(self.world.getCharacters()):
            self.world.remove(controller)
        for body in list(self.world.getRigidBodies()):
            self.world.remove(body)
        self.root.removeNode()
