from dataclasses import dataclass

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
        controller.setMaxSlope(50.0)
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

    def close(self) -> None:
        for controller in list(self.world.getCharacters()):
            self.world.remove(controller)
        for body in list(self.world.getRigidBodies()):
            self.world.remove(body)
        self.root.removeNode()
