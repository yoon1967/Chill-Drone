"""맵 생성기와 드론 모델 사양 검사. 실행: python -m unittest discover -s tests"""
import json
import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from worldgen import STAGES, LayoutParams, generate, known_map, write_world  # noqa: E402
from worldgen import furnish  # noqa: E402
from worldgen.grid import OccupancyGrid  # noqa: E402

SEEDS = range(25)


def _full_grid(world, with_humans=True) -> OccupancyGrid:
    """정답 기준 점유 격자: 벽·모든 장애물(·사람)을 드론 반지름만큼 부풀린다."""
    p = world.params
    grid = OccupancyGrid(p.width, p.depth, p.grid_res)
    for b in world.walls:
        grid.mark_rect(*b.bounds2d(), inflate=p.drone_radius)
    for o in world.obstacles:
        grid.mark_rect(*o.box.bounds2d(), inflate=p.drone_radius)
    if with_humans:
        for h in world.humans:
            grid.mark_rect(h.x - h.radius, h.y - h.radius, h.x + h.radius, h.y + h.radius, inflate=p.drone_radius)
    return grid


class LayoutTest(unittest.TestCase):
    def test_same_seed_builds_same_building_in_every_stage(self):
        worlds = [generate(3, stage) for stage in STAGES]
        for w in worlds[1:]:
            self.assertEqual(w.walls, worlds[0].walls)
            self.assertEqual([o.box for o in w.obstacles], [o.box for o in worlds[0].obstacles])
            self.assertEqual(w.spawn, worlds[0].spawn)
        self.assertEqual(worlds[2].humans, worlds[3].humans)   # 3·4단계 사람 위치도 같다

    def test_different_seeds_differ(self):
        self.assertNotEqual(generate(1, 1).walls, generate(2, 1).walls)

    def test_stage_rules(self):
        for seed in SEEDS:
            with self.subTest(seed=seed):
                w1, w2, w3, w4 = (generate(seed, s) for s in (1, 2, 3, 4))
                self.assertTrue(w1.obstacles, "장애물이 하나도 없음")
                # 1단계: 모든 장애물이 지도에, 목적지 있음, 사람 없음
                self.assertTrue(all(w1.is_known(o) for o in w1.obstacles))
                self.assertIsNotNone(w1.goal)
                self.assertEqual(w1.humans, [])
                # 2단계: 지도에 없는 장애물이 1개 이상, 목적지 있음
                self.assertTrue(any(not w2.is_known(o) for o in w2.obstacles))
                self.assertIsNotNone(w2.goal)
                self.assertEqual(w2.humans, [])
                # 3단계: 목적지 없음, 사람 있음
                self.assertIsNone(w3.goal)
                self.assertEqual(len(w3.humans), w3.params.humans)
                # 4단계: 지도에 벽·장애물이 없음
                m4 = known_map(w4)
                self.assertEqual(m4["walls"], [])
                self.assertEqual(m4["obstacles"], [])
                self.assertIsNone(m4["goal"])

    def test_goal_reachable_with_all_obstacles(self):
        for seed in SEEDS:
            with self.subTest(seed=seed):
                w = generate(seed, 2)
                grid = _full_grid(w, with_humans=False)
                dist = grid.distances_from(*grid.cell_of(w.spawn["x"], w.spawn["y"]))
                gi, gj = grid.cell_of(w.goal["x"], w.goal["y"])
                self.assertGreaterEqual(dist[gj][gi], 0, "숨긴 장애물까지 포함하면 목적지에 갈 수 없음")

    def test_every_room_and_human_is_reachable(self):
        for seed in SEEDS:
            with self.subTest(seed=seed):
                w = generate(seed, 3)
                grid = _full_grid(w)
                dist = grid.distances_from(*grid.cell_of(w.spawn["x"], w.spawn["y"]))
                for room in w.rooms:
                    cells = grid.cells_in(room.x0, room.y0, room.x1, room.y1)
                    self.assertTrue(any(dist[j][i] >= 0 for i, j in cells), f"갈 수 없는 방 {room}")
                for h in w.humans:
                    near = [(i, j) for i, j in grid.cells_in(h.x - 1.2, h.y - 1.2, h.x + 1.2, h.y + 1.2)
                            if dist[j][i] >= 0]
                    self.assertTrue(near, f"{h.name} 근처로 갈 수 없음")

    def test_blockers_sit_between_spawn_and_goal(self):
        for seed in SEEDS:
            with self.subTest(seed=seed):
                base = generate(seed, 2)
                w = generate(seed, 2, LayoutParams(blockers=2))
                blockers = [o for o in w.obstacles if o.blocker]
                self.assertGreaterEqual(len(blockers), 1, "경로를 막는 물건을 못 놓음")
                # 나머지 건물·가구는 그대로
                self.assertEqual([o.box for o in w.obstacles if not o.blocker], [o.box for o in base.obstacles])
                self.assertEqual((w.spawn, w.goal), (base.spawn, base.goal))
                # 2단계에선 지도에 없고, 1단계에선 지도에 있다
                self.assertTrue(all(o.hidden and not w.is_known(o) for o in blockers))
                w1 = generate(seed, 1, LayoutParams(blockers=2))
                self.assertTrue(all(w1.is_known(o) for o in w1.obstacles))
                for o in blockers:
                    self.assertGreaterEqual(o.box.sz, 1.0, "비행 높이까지 닿아야 한다")
                    self.assertGreater(math.dist((o.box.x, o.box.y), (w.spawn["x"], w.spawn["y"])), 1.0)
                    self.assertGreater(math.dist((o.box.x, o.box.y), (w.goal["x"], w.goal["y"])), 1.0)
                grid = _full_grid(w, with_humans=False)
                dist = grid.distances_from(*grid.cell_of(w.spawn["x"], w.spawn["y"]))
                gi, gj = grid.cell_of(w.goal["x"], w.goal["y"])
                self.assertGreaterEqual(dist[gj][gi], 0, "물건을 놓았더니 목적지에 갈 수 없음")

    def test_obstacles_do_not_overlap(self):
        for seed in SEEDS:
            with self.subTest(seed=seed):
                w = generate(seed, 1, LayoutParams(blockers=2))
                t = w.params.wall_thickness
                for o in w.obstacles:
                    x0, y0, x1, y1 = o.box.bounds2d()
                    self.assertGreater(x0, t / 2)
                    self.assertGreater(y0, t / 2)
                    self.assertLess(x1, w.params.width - t / 2)
                    self.assertLess(y1, w.params.depth - t / 2)
                    for wall in w.walls:
                        a = wall.bounds2d()
                        overlap = min(x1, a[2]) - max(x0, a[0]) > 0 and min(y1, a[3]) - max(y0, a[1]) > 0
                        self.assertFalse(overlap, f"{o.box.name}가 {wall.name}와 겹침")

    def test_furniture_fits_rooms_and_footprints(self):
        for seed in SEEDS:
            with self.subTest(seed=seed):
                w = generate(seed, 2, LayoutParams(blockers=2))
                self.assertGreaterEqual(sum(not o.hidden for o in w.obstacles), len(w.rooms))
                for o in w.obstacles:
                    x0, y0, x1, y1 = o.box.bounds2d()
                    room = next(r for r in w.rooms if r.x0 <= o.box.x <= r.x1 and r.y0 <= o.box.y <= r.y1)
                    if o.hidden:
                        self.assertIn(o.kind, furnish.BLOCKERS if o.blocker else furnish.CLUTTER)
                    else:   # 방 종류에 맞는 가구만 놓인다
                        plan = furnish.ROOM_KINDS[room.kind]
                        self.assertIn(o.kind, [k for k, _, _ in plan["wall"] + plan["center"]])
                    for other in w.obstacles:
                        if other is not o:
                            a = other.box.bounds2d()
                            overlap = min(x1, a[2]) > max(x0, a[0]) and min(y1, a[3]) > max(y0, a[1])
                            self.assertFalse(overlap, f"{o.box.name}와 {other.box.name}가 겹침")
                    for part in o.parts():   # 보이는 모양이 충돌 상자 밖으로 나가지 않는다
                        if part.shape == "box":
                            c, s = abs(math.cos(part.yaw)), abs(math.sin(part.yaw))
                            hx = c * part.size[0] / 2 + s * part.size[1] / 2
                            hy = s * part.size[0] / 2 + c * part.size[1] / 2
                            hz = part.size[2] / 2
                        else:
                            hx = hy = part.size[0]
                            hz = part.size[1] / 2 if part.shape == "cylinder" else part.size[0]
                        tol = 0.02
                        self.assertGreaterEqual(part.x - hx, x0 - tol, f"{o.box.name} {part}")
                        self.assertLessEqual(part.x + hx, x1 + tol, f"{o.box.name} {part}")
                        self.assertGreaterEqual(part.y - hy, y0 - tol, f"{o.box.name} {part}")
                        self.assertLessEqual(part.y + hy, y1 + tol, f"{o.box.name} {part}")
                        self.assertLessEqual(part.z + hz, o.box.sz + tol, f"{o.box.name} {part}")

    def test_lintels_sit_above_doors(self):
        w = generate(4, 1)
        self.assertEqual(len(w.lintels), len(w.doors))
        for lintel in w.lintels:
            self.assertAlmostEqual(lintel.z - lintel.sz / 2, w.params.door_height)
            self.assertAlmostEqual(lintel.z + lintel.sz / 2, w.params.wall_height)

    def test_spawn_is_inside_map(self):
        for seed in SEEDS:
            w = generate(seed, 1)
            self.assertTrue(0 < w.spawn["x"] < w.params.width and 0 < w.spawn["y"] < w.params.depth)

    def test_rejects_bad_stage_and_params(self):
        with self.assertRaises(ValueError):
            generate(0, 5)
        with self.assertRaises(ValueError):
            generate(0, 1, LayoutParams(min_room=1.5))


class ExportTest(unittest.TestCase):
    def test_files_and_contents(self):
        world = generate(5, 3)
        with tempfile.TemporaryDirectory() as tmp:
            paths = write_world(world, Path(tmp))
            for path in paths.values():
                self.assertTrue(path.exists(), path)
            self.assertEqual(paths["preview"].read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            for i in range(len(world.rooms)):   # 방마다 바닥 텍스처
                self.assertTrue((Path(tmp) / "textures" / f"floor_{i}.png").exists())

            root = ET.parse(paths["world"]).getroot()
            w = root.find("world")
            self.assertEqual(w.findtext("gravity"), "0 0 0")
            self.assertEqual(len(w.findall("actor")), len(world.humans))
            for actor, human in zip(w.findall("actor"), world.humans):
                # actor 자체 <pose>가 있으면 웨이포인트 좌표가 한 번 더 더해져 사람이 맵 밖에 나타난다
                self.assertIsNone(actor.find("pose"))
                x, y = map(float, actor.find("script/trajectory/waypoint/pose").text.split()[:2])
                self.assertAlmostEqual(x, human.x, places=2)
                self.assertAlmostEqual(y, human.y, places=2)
            self.assertEqual(w.find("include/uri").text, "model://tt_drone")
            obstacle_models = [m for m in w.findall("model") if m.get("name").startswith("obstacle_")]
            self.assertEqual(len(obstacle_models), len(world.obstacles))
            for albedo in w.iter("albedo_map"):
                self.assertTrue(Path(albedo.text).exists(), albedo.text)
            decor = next(m for m in w.findall("model") if m.get("name") == "decor")
            self.assertEqual(decor.findall("link/collision"), [], "장식에는 충돌체가 없어야 한다")
            self.assertGreater(len(decor.findall("link/visual")), len(world.walls))   # 걸레받이 + 문틀 + 창문·그림
            walls = next(m for m in w.findall("model") if m.get("name") == "walls")
            lintels = [c for c in walls.findall("link/collision") if c.get("name").startswith("lintel_")]
            self.assertEqual(len(lintels), len(world.doors))
            self.assertLess(len(list(w.iter("visual"))), 3000, "도형이 너무 많으면 Gazebo가 느려진다")

            known = json.loads(paths["known_map"].read_text(encoding="utf-8"))
            truth = json.loads(paths["ground_truth"].read_text(encoding="utf-8"))
            self.assertEqual(len(known["obstacles"]), sum(o["in_known_map"] for o in truth["obstacles"]))
            self.assertLess(len(known["obstacles"]), len(truth["obstacles"]))
            self.assertNotIn("humans", known)   # 사람 위치는 정답 파일에만

    def test_simple_humans(self):
        world = generate(5, 3)
        with tempfile.TemporaryDirectory() as tmp:
            paths = write_world(world, Path(tmp), human_model="simple")
            w = ET.parse(paths["world"]).getroot().find("world")
            self.assertEqual(w.findall("actor"), [])
            self.assertEqual(len([m for m in w.findall("model") if m.get("name").startswith("human_")]),
                             len(world.humans))

    def test_person_blocker_gets_body_and_collision(self):
        world = generate(3, 2, LayoutParams(blockers=2))
        people = [o for o in world.obstacles if o.kind == "person"]
        self.assertEqual(len(people), 1)
        for model, kind in (("actor", "actor"), ("simple", "model")):
            with tempfile.TemporaryDirectory() as tmp:
                paths = write_world(world, Path(tmp), human_model=model)
                w = ET.parse(paths["world"]).getroot().find("world")
                names = [e.get("name") for e in w.findall(kind)]
                self.assertIn(f"{people[0].box.name}_body", names)
                box_model = next(m for m in w.findall("model") if m.get("name") == people[0].box.name)
                self.assertIsNotNone(box_model.find("link/collision"), "사람 자리에 충돌 상자가 있어야 한다")
                truth = json.loads(paths["ground_truth"].read_text(encoding="utf-8"))
                person = next(o for o in truth["obstacles"] if o["kind"] == "person")
                self.assertTrue(person["blocker"] and person["hidden"] and not person["in_known_map"])

    def test_same_seed_same_files(self):
        world = generate(9, 2)
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            pa, pb = write_world(world, Path(a)), write_world(generate(9, 2), Path(b))
            self.assertEqual(pa["ground_truth"].read_text(encoding="utf-8"),
                             pb["ground_truth"].read_text(encoding="utf-8"))
            for name in ("floor_0.png", "art_0.png", "paint.png"):
                self.assertEqual((Path(a) / "textures" / name).read_bytes(),
                                 (Path(b) / "textures" / name).read_bytes())
            sdf_a = pa["world"].read_text(encoding="utf-8").replace(Path(a).resolve().as_posix(), "")
            sdf_b = pb["world"].read_text(encoding="utf-8").replace(Path(b).resolve().as_posix(), "")
            self.assertEqual(sdf_a, sdf_b)


class DroneModelTest(unittest.TestCase):
    """models/tt_drone/model.sdf가 TT 사양과 맞는지."""

    @classmethod
    def setUpClass(cls):
        cls.model = ET.parse(ROOT / "models" / "tt_drone" / "model.sdf").getroot().find("model")

    def sensor(self, name):
        return self.model.find(f"link/sensor[@name='{name}']")

    def test_mass(self):
        self.assertAlmostEqual(float(self.model.findtext("link/inertial/mass")), 0.087)

    def test_front_camera(self):
        cam = self.sensor("front_camera")
        self.assertEqual(cam.get("type"), "camera")
        self.assertEqual(cam.findtext("camera/image/width"), "960")
        self.assertEqual(cam.findtext("camera/image/height"), "720")
        self.assertEqual(cam.findtext("update_rate"), "30")
        self.assertAlmostEqual(math.degrees(float(cam.findtext("camera/horizontal_fov"))), 82.6, places=1)
        # TT에는 깊이 카메라가 없다. 평가 전용 정답 깊이(depth_gt)만 두고, 비행 코드는 그것을 쓰지 않는다
        depth = self.model.findall("link/sensor[@type='depth_camera']")
        self.assertEqual([d.get("name") for d in depth], ["depth_gt"])
        for folder in ("nodes", "autonomy", "perception"):
            for path in (ROOT / folder).glob("*.py"):
                self.assertNotIn("depth_gt", path.read_text(encoding="utf-8"), f"{path.name}가 평가 전용 깊이를 쓴다")

    def test_front_tof(self):
        tof = self.sensor("tof_front")
        self.assertEqual(tof.get("type"), "gpu_lidar")
        self.assertEqual(float(tof.findtext("lidar/range/max")), 1.2)
        # 시야 ±12.5° (nodes/mission_node.py의 TOF_HALF와 같아야 한다). 가운데·위·아래 세 줄
        for name, pitch in (("tof_front", 0.0), ("tof_front_up", -12.5), ("tof_front_down", 12.5)):
            row = self.sensor(name)
            self.assertIsNotNone(row, name)
            self.assertAlmostEqual(math.degrees(float(row.findtext("lidar/scan/horizontal/max_angle"))), 12.5, places=1)
            self.assertEqual(row.findtext("lidar/scan/vertical/samples"), "1", "브리지는 한 줄만 넘긴다")
            self.assertAlmostEqual(math.degrees(float(row.findtext("pose").split()[4])), pitch, places=1)
            self.assertEqual(float(row.findtext("lidar/range/max")), 1.2)

    def test_velocity_control(self):
        names = [p.get("name") for p in self.model.findall("plugin")]
        self.assertIn("gz::sim::systems::VelocityControl", names)
        self.assertIn("gz::sim::systems::OdometryPublisher", names)


if __name__ == "__main__":
    unittest.main()
