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
    # Planar planning (search/planar, MoNA): the PID sends no vertical speed, so plans must stay at the start
    # height. 11/06 flight: the search sampled vertical accelerations, the goal relocation moved the goal
    # +-0.3 m per try (0 -> 0.3 -> 0.59), and the B-spline optimizer moved z freely (+-10 m bounds); every plan
    # left the flight height and the route hovered, then failed. Fix z in all three when planar.
    kino_h = target / "path_searching/include/path_searching/kinodynamic_astar.h"
    patch(kino_h, "  double max_vel_, max_acc_;", "  double max_vel_, max_acc_;\n  bool planar_ = false;")
    patch(kino, '  nh.param("search/max_acc", max_acc_, -1.0);\n',
          '  nh.param("search/max_acc", max_acc_, -1.0);\n  nh.param("search/planar", planar_, false);\n')
    patch(kino, "          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)",
          "          for (double az = (planar_ ? 0.0 : -max_acc_); az <= (planar_ ? 0.0 : max_acc_) + 1e-3;\n"
          "               az += max_acc_ * res)")
    patch(kino, "      inputs.push_back(start_acc_);\n",
          "      inputs.push_back(planar_ ? Eigen::Vector3d(start_acc_(0), start_acc_(1), 0.0) : start_acc_);\n")
    fsm = target / "plan_manage/src/kino_replan_fsm.cpp"
    patch(target / "plan_manage/include/plan_manage/kino_replan_fsm.h", "  double no_replan_thresh_, replan_thresh_;",
          "  double no_replan_thresh_, replan_thresh_;\n  bool planar_ = false;")
    patch(fsm, '  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);\n',
          '  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);\n  nh.param("search/planar", planar_, false);\n')
    patch(fsm, "          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {",
          "          for (double nz = (planar_ ? 0.0 : 1 * dz); nz >= (planar_ ? 0.0 : -1 * dz) - 1e-6; nz -= dz) {")
    # Goal relocation picks the point farthest from obstacles within 2.5 m of a goal that is near one. Indoors that
    # pulled the goal sideways through a side wall into unseen space (sim 11/06: (3, 0) -> (3.9, 3.0), the drone
    # stopped 0.15 m from the wall). In planar mode take the first (nearest) ring with a free point instead.
    patch(fsm, "              max_dist = dist;\n            }\n          }\n        }\n      }\n",
          "              max_dist = dist;\n            }\n          }\n        }\n"
          "        if (planar_ && max_dist > 0.3) break;  // MoNA: nearest free ring, not the farthest point\n      }\n")
    # A plan that stops short of a blocked goal ended in WAIT_TARGET (goal dropped): the drone hovered beside a
    # person 1 m ahead and the route failed (sim 11/08). In planar mode plan again when the trajectory ends > 0.3 m
    # from the goal; if no plan comes, the bridge's no-progress rule fails the route and the gateway lands.
    patch(fsm, "      if (t_cur > info->duration_ - 1e-2) {\n        have_target_ = false;\n",
          "      if (t_cur > info->duration_ - 1e-2) {\n"
          "        if (planar_ && (end_pt_ - pos).norm() > 0.3) {  // MoNA: stopped short of the goal: plan again\n"
          "          changeFSMExecState(REPLAN_TRAJ, \"FSM\");\n"
          "          return;\n"
          "        }\n"
          "        have_target_ = false;\n")
    # The reference runs on wall time. When the drone falls behind (Tello slower than asked, a stall) the PID chases a
    # point far ahead and cuts the corner toward it: sim 11/09 left the plan by 0.87 m around a person and touched
    # them, and a Tello at 0.6x speed left it by 0.59 m. The trajectory server now holds the reference time while the
    # drone is more than wait_near_m behind (stopped at wait_far_m) and publishes the held seconds; the FSM shifts its
    # trajectory start by the same amount, so replanning and collision checks use the same (held) time.
    srv = target / "plan_manage/src/traj_server.cpp"
    patch(srv, '#include "std_msgs/Empty.h"\n', '#include "std_msgs/Empty.h"\n#include "std_msgs/Float64MultiArray.h"\n')
    patch(srv, "ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;\n",
          "ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;\n"
          "// MoNA: reference time held while the drone lags (seconds held for this trajectory)\n"
          "ros::Publisher hold_pub;\n"
          "double hold_s_ = 0.0, last_raw_t_ = -1.0, wait_near_m_ = 0.2, wait_far_m_ = 0.45;\n")
    patch(srv, "  traj_duration_ = traj_[0].getTimeSum();\n\n  receive_traj_ = true;\n",
          "  traj_duration_ = traj_[0].getTimeSum();\n  hold_s_ = 0.0;\n  last_raw_t_ = -1.0;\n\n  receive_traj_ = true;\n")
    patch(srv, "  double t_stop = (time_now - start_time_).toSec() + time_out;\n",
          "  double t_stop = (time_now - start_time_).toSec() - hold_s_ + time_out;\n")
    patch(srv, "  double t_cur = (time_now - start_time_).toSec();\n\n  Eigen::Vector3d pos, vel, acc, pos_f;\n",
          "  double raw_t = (time_now - start_time_).toSec();\n"
          "  if (last_raw_t_ >= 0.0 && raw_t > last_raw_t_ && !odom.header.stamp.isZero()) {\n"
          "    double t_ref = min(max(raw_t - hold_s_, 0.0), traj_duration_);\n"
          "    Eigen::Vector3d ref = traj_[0].evaluateDeBoorT(t_ref);\n"
          "    double d = hypot(ref(0) - odom.pose.pose.position.x, ref(1) - odom.pose.pose.position.y);\n"
          "    double wait = min(max((d - wait_near_m_) / (wait_far_m_ - wait_near_m_), 0.0), 1.0);\n"
          "    if (t_ref < traj_duration_) hold_s_ += wait * (raw_t - last_raw_t_);\n"
          "  }\n"
          "  last_raw_t_ = raw_t;\n"
          "  double t_cur = raw_t - hold_s_;\n"
          "  std_msgs::Float64MultiArray hold_msg;\n"
          "  hold_msg.data.push_back(traj_id_);\n"
          "  hold_msg.data.push_back(hold_s_);\n"
          "  hold_pub.publish(hold_msg);\n\n"
          "  Eigen::Vector3d pos, vel, acc, pos_f;\n")
    patch(srv, '  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);\n',
          '  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);\n'
          '  hold_pub = node.advertise<std_msgs::Float64MultiArray>("/planning/traj_hold", 10);\n')
    patch(srv, '  nh.param("traj_server/time_forward", time_forward_, -1.0);\n',
          '  nh.param("traj_server/time_forward", time_forward_, -1.0);\n'
          '  nh.param("traj_server/wait_near_m", wait_near_m_, 0.2);\n'
          '  nh.param("traj_server/wait_far_m", wait_far_m_, 0.45);\n')
    fsm_h = target / "plan_manage/include/plan_manage/kino_replan_fsm.h"
    patch(fsm_h, "#include <std_msgs/Empty.h>\n", "#include <std_msgs/Empty.h>\n#include <std_msgs/Float64MultiArray.h>\n")
    patch(fsm_h, "  ros::Subscriber waypoint_sub_, odom_sub_;\n",
          "  ros::Subscriber waypoint_sub_, odom_sub_;\n"
          "  ros::Subscriber hold_sub_;           // MoNA: reference time held by the trajectory server\n"
          "  ros::Time hold_base_;\n  int hold_id_ = -1;\n")
    patch(fsm_h, "  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);\n",
          "  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);\n"
          "  void holdCallback(const std_msgs::Float64MultiArrayConstPtr& msg);\n")
    patch(fsm, '  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);\n',
          '  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);\n'
          '  hold_sub_ = nh.subscribe("/planning/traj_hold", 10, &KinoReplanFSM::holdCallback, this);\n')
    patch(fsm, "void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {\n",
          "void KinoReplanFSM::holdCallback(const std_msgs::Float64MultiArrayConstPtr& msg) {\n"
          "  // MoNA: shift this trajectory's start by the seconds the trajectory server held its reference\n"
          "  LocalTrajData* info = &planner_manager_->local_data_;\n"
          "  if (msg->data.size() < 2 || int(msg->data[0]) != info->traj_id_) return;\n"
          "  if (hold_id_ != info->traj_id_) {\n"
          "    hold_id_ = info->traj_id_;\n"
          "    hold_base_ = info->start_time_;\n"
          "  }\n"
          "  info->start_time_ = hold_base_ + ros::Duration(msg->data[1]);\n"
          "}\n\n"
          "void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {\n")
    opt = target / "bspline_opt/src/bspline_optimizer.cpp"
    patch(target / "bspline_opt/include/bspline_opt/bspline_optimizer.h", "  int    order_;                  // bspline degree",
          "  int    order_;                  // bspline degree\n  bool   planar_ = false;          // keep z of control points")
    patch(opt, '  nh.param("optimization/order", order_, -1);\n',
          '  nh.param("optimization/order", order_, -1);\n  nh.param("search/planar", planar_, false);\n')
    patch(opt, "      lb[i] = q[i] - bound;\n      ub[i] = q[i] + bound;\n",
          "      const double b = (planar_ && dim_ == 3 && i % dim_ == 2) ? 1e-4 : bound;   // z held in planar mode\n"
          "      lb[i] = q[i] - b;\n      ub[i] = q[i] + b;\n")
    # PID (mav_controller): it chases the trajectory server's position with per-axis limits (x 0.2, y 0.15 m/s,
    # lateral gain x1.6) and ignores the planned velocity. When the reference runs ahead the limits bend the
    # direction and the drone cuts corners toward it (closed-loop sim 11/06: 0.43 m off the plan). Add the planned
    # velocity as feedforward and limit the horizontal speed by length so the direction is kept.
    pid = target / "mav_controller/scripts/pid_controller.py"
    # The horizontal limit is a parameter (~max_speed; planner.launch passes plan_max_vel).
    patch(pid, "        self.vel_max_x = 0.2 # 0.36\n",
          "        self.vel_max_x = float(rospy.get_param('~max_speed', 0.2))  # MoNA\n")
    patch(pid, "        self.vel_max_y = 0.15\n", "        self.vel_max_y = self.vel_max_x\n")
    patch(pid, "            if u_x > self.vel_max_x:\n",
          "            # MoNA: follow the planned velocity (feedforward) and keep the direction when limiting speed\n"
          "            u_x += msg.velocity.x\n"
          "            u_y += msg.velocity.y\n"
          "            n = (u_x * u_x + u_y * u_y) ** 0.5\n"
          "            if n > self.vel_max_x:\n"
          "                u_x, u_y = u_x * self.vel_max_x / n, u_y * self.vel_max_x / n\n"
          "            if u_x > self.vel_max_x:\n")
    # Virtual floor beside the upstream virtual ceiling. The floor filter blanks floor pixels, so the
    # space below an obstacle looks free and the planner routed under it; the PID has no vertical speed,
    # so such a path would be flown level into the obstacle. Same one-layer sheet as the ceiling.
    sdf = target / "plan_env/src/sdf_map.cpp"
    patch(target / "plan_env/include/plan_env/sdf_map.h", "virtual_ceil_height_, ground_height_;",
          "virtual_ceil_height_, ground_height_, virtual_floor_height_;")
    patch(sdf, '  node_.param("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);\n',
          '  node_.param("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);\n'
          '  node_.param("sdf_map/virtual_floor_height", mp_.virtual_floor_height_, -1000.0);\n')
    patch(sdf, "        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;\n      }\n  }\n",
          "        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;\n      }\n  }\n"
          "  // virtual floor: keep plans at flight height (no vertical speed downstream)\n"
          "  if (mp_.virtual_floor_height_ > mp_.map_origin_(2)) {\n"
          "    int floor_id = floor((mp_.virtual_floor_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);\n"
          "    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)\n"
          "      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {\n"
          "        md_.occupancy_buffer_inflate_[toAddress(x, y, floor_id)] = 1;\n"
          "      }\n"
          "  }\n")
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
    # Fixed depth scale (~fixed_scale > 0). Metric depth models (DA-V2) already output metres; multiplying by the
    # ToF/ground scale made distances worse and the scale kept changing, which RGB-D SLAM reads as motion.
    # Replays 11/08 with taped ground truth (3 m, 3 m, 1 m): ToF scale -16/+1/-37 %, fixed 1.0 +6/+17/-9 %.
    patch(scale, "        self._trans_mat = TransPointCloud().transform_matrix(self._trans, self._euler)\n",
          "        self._trans_mat = TransPointCloud().transform_matrix(self._trans, self._euler)\n"
          "        self._fixed_scale = float(rospy.get_param('~fixed_scale', 0.0))  # MoNA: > 0 skips the ToF scale\n"
          "        if self._fixed_scale > 0:\n"
          "            self._SCALE_FACTOR_FILTER = np.full(self._filter_shape, self._fixed_scale)\n")
    patch(scale, "    def scale_factor_callback(self, point_msg, height_msg):\n",
          "    def scale_factor_callback(self, point_msg, height_msg):\n"
          "        if self._fixed_scale > 0:\n"
          "            return\n")
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
        "kinodynamic getSamples walks back across all short segments and uses the last node's end velocity",
        "SDF map virtual floor (sdf_map/virtual_floor_height) beside the virtual ceiling",
        "planar planning (search/planar): no vertical search input, goal relocation and B-spline z held",
        "planar goal relocation takes the nearest free ring instead of the farthest point",
        "planar: a trajectory ending short of the goal replans instead of dropping the goal",
        "trajectory server holds the reference time while the drone lags; the FSM shifts its start time to match",
        "PID: planned-velocity feedforward and direction-keeping horizontal limit (~max_speed)",
        "scale recovery: ~fixed_scale (metric depth models skip the ToF scale)"],
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
