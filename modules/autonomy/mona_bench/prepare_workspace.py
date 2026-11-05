"""Fetch a pinned MoNA-Bench and stage clean ROS 1 packages (no flight commands)."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
LOCK = json.loads((HERE / "upstream.lock.json").read_text(encoding="utf-8"))
SELECTION = [
    "rmtt_driver_ws/src/rmtt_driver", "rmtt_driver_ws/src/rmtt_teleop",
    "scale_recovery_ws/src/scale_recovery", "orb_slam_2_ws/src/orb_slam_2_ros",
    "fast_planner_ws/src/Fast-Planner/fast_planner",
    "fast_planner_ws/src/Fast-Planner/uav_simulator/Utils/quadrotor_msgs",
    "fast_planner_ws/src/Fast-Planner/uav_simulator/Utils/cmake_utils",
    "fast_planner_ws/src/Fast-Planner/uav_simulator/Utils/waypoint_generator",
    "mav_controller_ws/src/mav_controller",
    "monocular_depth_ROS/sc_depth_pl_ws/src/sc_depth_pl_ros",
]


def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()


def fetch(root):
    root.mkdir(parents=True, exist_ok=False)
    subprocess.run(["git", "init", str(root)], check=True)
    git(root, "remote", "add", "origin", LOCK["repository"])
    git(root, "config", "remote.origin.promisor", "true")
    git(root, "config", "remote.origin.partialclonefilter", "blob:none")
    git(root, "fetch", "--depth", "1", "--filter=blob:none", "origin", LOCK["revision"])
    git(root, "sparse-checkout", "init", "--cone")
    git(root, "sparse-checkout", "set", *SELECTION)
    git(root, "checkout", "--detach", "FETCH_HEAD")
    if git(root, "rev-parse", "HEAD") != LOCK["revision"]:
        raise RuntimeError("upstream revision mismatch")


def patch(path, old, new):
    content = path.read_text(encoding="utf-8")
    if old not in content:
        raise RuntimeError("patch context not found: " + str(path))
    path.write_text(content.replace(old, new), encoding="utf-8")


def stage(source, workspace):
    if git(source, "rev-parse", "HEAD") != LOCK["revision"]:
        raise RuntimeError("only the locked upstream revision is supported")
    target = workspace / "src"
    target.mkdir(parents=True, exist_ok=False)
    packages = []
    for selected in SELECTION:
        folder = source / selected
        roots = [folder] if (folder / "package.xml").exists() else sorted({p.parent for p in folder.glob("*/package.xml")})
        for root in roots:
            name = ET.parse(root / "package.xml").getroot().findtext("name")
            if name in packages:
                raise RuntimeError("duplicate ROS package: " + name)
            packages.append(name)
            shutil.copytree(root, target / name, ignore=shutil.ignore_patterns(
                "build", "devel", "install", ".git", "__pycache__", "*.pyc", "*.pcd", "*.bag"))
    shutil.copy2(source / "LICENSE", workspace / "UPSTREAM-LICENSE")
    # Ubuntu 20.04 ships system NLopt, rather than a catkin package named nlopt.
    cmake = target / "bspline_opt/CMakeLists.txt"
    patch(cmake, "  nlopt\n", "")
    patch(cmake, "find_package(Eigen3 REQUIRED)", "find_package(Eigen3 REQUIRED)\nfind_package(NLopt REQUIRED)")
    patch(cmake, "${catkin_LIBRARIES} \n", "${catkin_LIBRARIES} ${NLOPT_LIBRARIES}\n")
    patch(target / "plan_manage/CMakeLists.txt", "target_link_libraries(fast_planner_node",
          "add_dependencies(fast_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})\ntarget_link_libraries(fast_planner_node")
    # PCL 1.10 in Noetic requires C++14, including consumers of PCL headers.
    for cmake_file in target.rglob("CMakeLists.txt"):
        content = cmake_file.read_text(encoding="utf-8")
        cmake_file.write_text(content.replace("-std=c++11", "-std=c++14"), encoding="utf-8")
    # Preserve upstream topics, but fix launch parameters being ignored and an
    # accidental input/output topic alias when topic_color_img is supplied.
    node = target / "sc_depth_pl_ros/src/sc_depth_node.py"
    patch(node, 'get_param("~topic_color_img", "/tello/image_raw")', 'get_param("~sub_color_img", "/tello/image_raw")')
    patch(node, 'get_param("~topic_color_img", "/sc_depth_pl/color/image_raw")', 'get_param("~pub_color_img", "/sc_depth_pl/color/image_raw")')
    patch(node, 'get_param("~topic_depth_img",', 'get_param("~pub_depth_img",')
    patch(node, "self.model = system.load_from_checkpoint(package_path + models_path)",
          "self.model = system.load_from_checkpoint(rospy.get_param('~checkpoint', package_path + models_path), map_location='cpu')")
    patch(node, "self.model.cuda()", "self.device = rospy.get_param('~device', 'cuda')\n        self.model.to(self.device)")
    patch(node, ".unsqueeze(0).cuda()", ".unsqueeze(0).to(self.device)")
    patch(node, "pred_depth = self.model.inference_depth(tensor_img)",
          "with torch.inference_mode():\n            pred_depth = self.model.inference_depth(tensor_img)")
    patch(node, "image = np.array(image)", "image = cv2.cvtColor(np.array(image), cv2.COLOR_BGR2RGB)")
    ground = target / "scale_recovery/src/ground_seg_planner.cpp"
    patch(ground, "10/180 * M_PI", "10.0/180.0 * M_PI")
    patch(ground, "if (abs(coefficients->values[1]) > 0.90)",
          "if (coefficients->values.size() < 4 || inliers->indices.empty()) { return; }\n  if (abs(coefficients->values[1]) > 0.90)")
    patch(ground, 'output.header.frame_id = "camera_link";',
          'output.header = input->header;\n  output.header.frame_id = "camera_link";')
    # When RANSAC finds no level plane, upstream reuses the last plane but keeps
    # every point on one side of it (signed distance). Keep points near the plane.
    patch(ground, "if (distance <= this->distance_threshold_) {",
          "if (std::fabs(distance) <= this->distance_threshold_) {")
    # The SC-Depth band (0.001) is in relative-depth units; metric models need their own.
    patch(ground, "      this->distance_threshold_ = 0.001;\n",
          "      this->distance_threshold_ = 0.001;\n"
          "      nh.param<double>(\"distance_threshold\", this->distance_threshold_, this->distance_threshold_);\n")
    # Fast-Planner falls off the end of non-void functions. GCC 9 (Noetic) treats
    # that as undefined behaviour; replay crashed right after kinodynamic search.
    for search in ("path_searching/src/astar.cpp", "path_searching/src/kinodynamic_astar.cpp"):
        patch(target / search, "  int idx = floor((time - time_origin_) * inv_time_resolution_);\n}",
              "  int idx = floor((time - time_origin_) * inv_time_resolution_);\n  return idx;\n}")
    # getSamples steps back one search segment per B-spline sample. With max_vel 0.2 the sample
    # interval (control_points_distance / max_vel = 2.5 s) exceeds a segment (<= 0.8 s), so the
    # leftover negative time piled up and samples were extrapolated tens of metres away
    # ("near end" replans). Step back as many segments as needed, and take the end velocity
    # from the last node instead of the start node.
    kino = target / "path_searching/src/kinodynamic_astar.cpp"
    patch(kino, "      // samples on searched traj\n",
          "      // samples on searched traj\n"
          "      while (t < -1e-5 && node->parent->parent != NULL)\n"
          "      {\n        node = node->parent;\n        t += node->duration;\n      }\n")
    patch(kino, "end_vel = node->state.tail(3);", "end_vel = path_nodes_.back()->state.tail(3);")
    edt = target / "plan_env/src/edt_environment.cpp"
    patch(edt, "  grad[0] *= resolution_inv_;\n}",
          "  grad[0] *= resolution_inv_;\n  return std::make_pair(value, grad);\n}")
    patch(edt, "  interpolateTrilinear(dists, diff, dist, grad);\n}",
          "  return interpolateTrilinear(dists, diff, dist, grad);\n}")
    patch(target / "poly_traj/include/poly_traj/polynomial_traj.h",
          "    double mean_vel = length / time_sum;\n  }",
          "    double mean_vel = length / time_sum;\n    return mean_vel;\n  }")
    # Manual goals were forced to z=1.0 m in the SLAM map, so a 1 m forward goal
    # from a start near z=0 planned a 45-degree climb. Use the published height.
    patch(target / "plan_manage/src/kino_replan_fsm.cpp",
          "end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;",
          "end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z;")
    patch(target / "plan_manage/src/topo_replan_fsm.cpp",
          "bool TopoReplanFSM::callSearchAndOptimization() {}",
          "bool TopoReplanFSM::callSearchAndOptimization() { return false; }")
    # Upstream appends twice, so its buffer grows past 10 and metric-depth
    # publication (guarded by len == 10) permanently stops. Bound it explicitly.
    scale = target / "scale_recovery/scripts/scale_recovery_planner.py"
    content = scale.read_text(encoding="utf-8")
    begin = content.index("    def scale_factor_callback(")
    end = content.index("    def metric_depth_callback(", begin)
    # Replay of a 1.5 m hand-held capture showed the original filter locking onto
    # a plane above the camera (abs() hid the sign) and then rejecting every real
    # floor sample because of the absolute 0.5 band. Require a floor below the
    # camera (RANSAC keeps only ~20-90 of 3072 points, so count cannot separate a
    # ceiling from the floor; 10 only drops empty fits), compare relatively, and
    # re-seed when a sustained,
    # self-consistent run of samples disagrees with the locked value.
    callback = '''    def scale_factor_callback(self, point_msg, height_msg):
        relative_height = self.relative_height_estimate(point_msg)
        metric_height = height_msg.range
        points = int(getattr(point_msg, "width", 0)) * int(getattr(point_msg, "height", 0))
        tolerance = getattr(self, "_relative_tolerance", 0.2)
        if not np.isfinite(relative_height) or relative_height >= -1e-6:
            return  # the ground plane must be below the camera
        if points < getattr(self, "_min_ground_points", 10):
            return
        if not np.isfinite(metric_height) or not 0.1 < metric_height < 6.0:
            return
        scale_factor = float(metric_height / -relative_height)
        if not np.isfinite(scale_factor) or scale_factor <= 0:
            return
        if len(self._SCALE_FACTOR_FILTER) >= self._filter_shape:
            median = np.median(self._SCALE_FACTOR_FILTER)
            if abs(scale_factor / median - 1) > tolerance:
                rejected = (list(getattr(self, "_REJECTED_SCALES", [])) + [scale_factor])[-self._filter_shape:]
                self._REJECTED_SCALES = rejected
                candidate = np.median(rejected)
                if len(rejected) == self._filter_shape and np.all(np.abs(np.array(rejected) / candidate - 1) <= tolerance):
                    self._SCALE_FACTOR_FILTER = np.array(rejected)
                    self._REJECTED_SCALES = []
                return
        self._REJECTED_SCALES = []
        self._SCALE_FACTOR_FILTER = np.append(self._SCALE_FACTOR_FILTER, scale_factor)[-self._filter_shape:]


'''
    scale.write_text(content[:begin] + callback + content[end:], encoding="utf-8")
    # ROS Noetic invokes Python 3. Upstream files have mixed legacy shebangs.
    for script in target.rglob("*.py"):
        if "site-packages" in script.parts:
            continue
        content = script.read_text(encoding="utf-8", errors="strict")
        if content.startswith("#!"):
            lines = content.splitlines(keepends=True)
            lines[0] = "#!/usr/bin/env python3\n"
            script.write_text("".join(lines), encoding="utf-8")
            script.chmod(script.stat().st_mode | 0o111)
    # Keep text vocabulary extraction bounded to the single known member.
    import tarfile
    vocabulary = target / "orb_slam2_ros/orb_slam2/Vocabulary"
    with tarfile.open(vocabulary / "ORBvoc.txt.tar.gz") as archive:
        member = archive.getmember("ORBvoc.txt")
        if not member.isfile():
            raise RuntimeError("invalid ORB vocabulary member")
        with archive.extractfile(member) as reader, (vocabulary / "ORBvoc.txt").open("wb") as writer:
            shutil.copyfileobj(reader, writer)
    manifest = dict(LOCK, packages=packages, patches=[
        "system NLopt binding", "planner message generation dependency", "C++14 for Noetic PCL", "SC-Depth ROS topic/checkpoint/device parameters",
        "Python3 shebangs", "RGB inference input", "ground RANSAC integer division and empty result check",
        "preserve ground cloud timestamps", "bounded median scale filter and invalid-height rejection",
        "Fast-Planner missing return values (GCC 9 undefined behaviour)",
        "manual planner goal keeps the published height instead of z=1.0",
        "scale samples need a ground plane below the camera, relative band, re-seed on sustained disagreement",
        "reused ground plane keeps points near the plane (absolute distance)",
        "ground RANSAC distance threshold is a parameter (metric depth models)",
        "kinodynamic getSamples walks back across all short segments and uses the last node's end velocity"],
        files={str(p.relative_to(target)): hashlib.sha256(p.read_bytes()).hexdigest()
               for p in target.rglob("CMakeLists.txt")})
    (workspace / "source-manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(dict(packages=packages, revision=LOCK["revision"])))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("fetch", "stage"))
    parser.add_argument("source", type=Path)
    parser.add_argument("workspace", type=Path, nargs="?")
    args = parser.parse_args()
    if args.action == "fetch":
        fetch(args.source)
    elif args.workspace:
        stage(args.source, args.workspace)
    else:
        parser.error("stage requires a new workspace path")
