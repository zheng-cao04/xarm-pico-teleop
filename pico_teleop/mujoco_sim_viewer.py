from __future__ import annotations

import copy
import logging
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

try:
    from transformations import Transformations
except ModuleNotFoundError:
    from ufactory_devices.umi.vive_tracker.transformations import Transformations


logger = logging.getLogger("pico_teleop.mujoco_sim")


class MujocoSimViewer:
    """Dual MuJoCo xArm7 visualizer for PICO teleop target poses."""

    _VISUAL_OFFSETS = {
        "L": np.array([0.0, 0.35, 0.0], dtype=float),
        "R": np.array([0.0, -0.35, 0.0], dtype=float),
    }

    def __init__(self, config, arms):
        try:
            import mujoco
            import mujoco.viewer
            from robot_descriptions import xarm7_mj_description
        except ImportError as exc:
            raise RuntimeError(
                "MuJoCo sim viewer requires: pip install mujoco robot_descriptions"
            ) from exc

        self.mujoco = mujoco
        self.focus_arm_name = "L" if config.mujoco_arm == "left" else "R"
        self.axis_length = config.axis_length / 1000.0
        self.ik_damping = 0.08
        self.ik_orientation_weight = 0.25
        self.max_q_step = 0.04
        self._reset_counters = {}

        self._scene_xml = self._build_dual_xarm_scene(Path(xarm7_mj_description.MJCF_PATH))
        self.model = mujoco.MjModel.from_xml_path(str(self._scene_xml))
        self.data = mujoco.MjData(self.model)
        self._arm_state = self._make_arm_state(arms)
        self._initialize_qpos(arms)

        try:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        except Exception as exc:
            raise RuntimeError(
                "Failed to open MuJoCo viewer. Check that this shell has a GUI display "
                "and working OpenGL/GLFW support."
            ) from exc
        self.viewer.cam.azimuth = 135
        self.viewer.cam.elevation = -25
        self.viewer.cam.distance = 1.75
        self.viewer.cam.lookat[:] = np.array([0.35, 0.0, 0.35])
        logger.info("MuJoCo dual xArm7 viewer started; displaying L and R arm targets")

    def _build_dual_xarm_scene(self, source_xml_path: Path) -> Path:
        source_root = ET.parse(source_xml_path).getroot()
        root = ET.Element("mujoco", {"model": "dual_xarm7_pico"})

        compiler = copy.deepcopy(source_root.find("compiler"))
        if compiler is not None:
            compiler.set("meshdir", str(source_xml_path.parent / "assets"))
            root.append(compiler)

        option = copy.deepcopy(source_root.find("option"))
        if option is not None:
            root.append(option)
        root.append(ET.Element("statistic", {"center": "0.35 0 0.35", "extent": "1.4"}))

        for tag in ("asset", "default"):
            elem = source_root.find(tag)
            if elem is not None:
                root.append(copy.deepcopy(elem))

        worldbody = ET.SubElement(root, "worldbody")
        ET.SubElement(worldbody, "light", {"name": "key", "pos": "0 -1 2", "dir": "0 1 -2"})
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "floor",
                "type": "plane",
                "size": "1.2 1.0 0.01",
                "rgba": "0.75 0.75 0.75 1",
            },
        )

        source_worldbody = source_root.find("worldbody")
        if source_worldbody is None:
            raise RuntimeError("Source xArm7 MJCF has no worldbody")
        base_body = source_worldbody.find("body")
        if base_body is None:
            raise RuntimeError("Source xArm7 MJCF has no base body")

        for name, offset in self._VISUAL_OFFSETS.items():
            body = copy.deepcopy(base_body)
            self._prefix_tree(body, f"{name}_")
            body.set("pos", self._offset_pos(base_body.get("pos", "0 0 0"), offset))
            worldbody.append(body)

        for name in self._VISUAL_OFFSETS:
            self._append_prefixed_section(source_root, root, "contact", name)
            self._append_prefixed_section(source_root, root, "tendon", name)
            self._append_prefixed_section(source_root, root, "equality", name)
            self._append_prefixed_section(source_root, root, "actuator", name)

        tmp = tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".xml",
            prefix="dual_xarm7_pico_",
            delete=False,
        )
        path = Path(tmp.name)
        tmp.write(ET.tostring(root, encoding="unicode"))
        tmp.close()
        return path

    def _append_prefixed_section(self, source_root, root, tag, arm_name):
        source_section = source_root.find(tag)
        if source_section is None:
            return
        section = root.find(tag)
        if section is None:
            section = ET.SubElement(root, tag)
        for child in source_section:
            if tag == "keyframe":
                continue
            elem = copy.deepcopy(child)
            self._prefix_tree(elem, f"{arm_name}_")
            section.append(elem)

    def _prefix_tree(self, elem, prefix):
        for node in elem.iter():
            if node.tag == "mesh":
                continue
            if "name" in node.attrib:
                node.set("name", prefix + node.get("name"))
            for attr in ("body", "body1", "body2", "joint", "joint1", "joint2", "site", "tendon"):
                if attr in node.attrib:
                    node.set(attr, prefix + node.get(attr))

    def _offset_pos(self, pos_text, offset):
        pos = np.fromstring(pos_text, sep=" ", dtype=float)
        if pos.size != 3:
            pos = np.zeros(3, dtype=float)
        pos = pos + offset
        return " ".join(f"{v:.8g}" for v in pos)

    def _make_arm_state(self, arms):
        state = {}
        for arm in arms:
            prefix = f"{arm.name}_"
            joint_ids = [
                self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}joint{i}")
                for i in range(1, 8)
            ]
            actuator_ids = [
                self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, f"{prefix}act{i}")
                for i in range(1, 8)
            ]
            gripper_actuator_id = self.mujoco.mj_name2id(
                self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, f"{prefix}gripper"
            )
            site_id = self.mujoco.mj_name2id(
                self.model, self.mujoco.mjtObj.mjOBJ_SITE, f"{prefix}link_tcp"
            )
            if min(joint_ids + actuator_ids + [gripper_actuator_id, site_id]) < 0:
                raise RuntimeError(f"MuJoCo dual xArm scene is missing objects for {arm.name}")

            state[arm.name] = {
                "joint_ids": joint_ids,
                "qpos_ids": [self.model.jnt_qposadr[i] for i in joint_ids],
                "dof_ids": [self.model.jnt_dofadr[i] for i in joint_ids],
                "actuator_ids": actuator_ids,
                "gripper_actuator_id": gripper_actuator_id,
                "site_id": site_id,
                "visual_offset": self._VISUAL_OFFSETS[arm.name],
            }
            self._reset_counters[arm.name] = getattr(arm.robot, "reset_count", 0)
        return state

    def _initialize_qpos(self, arms):
        for arm in arms:
            state = self._arm_state[arm.name]
            start_joints = np.asarray(arm.robot.config.start_joints[:7], dtype=float)
            self.data.qpos[state["qpos_ids"]] = start_joints
            self.data.ctrl[state["actuator_ids"]] = start_joints
        self.mujoco.mj_forward(self.model, self.data)
        for arm in arms:
            self._move_to_robot_pose(arm, iterations=80)
            self._apply_controls(arm.name, list(arm.robot.pose_aa) + [arm.robot.gripper_norm])
        self.mujoco.mj_forward(self.model, self.data)

    def update(self, frame, arms):
        if not self.viewer.is_running():
            return

        with self.viewer.lock():
            for arm in arms:
                self._handle_reset(arm)
                action = arm.robot.last_action
                if action is None:
                    continue
                target_pos = self._action_pos_m(arm.name, action)
                target_rot = Transformations.rxryrz_to_rotation_matrix(*action[3:6])
                self._solve_pose_ik(arm.name, target_pos, target_rot)
                self._apply_controls(arm.name, action)
            self.mujoco.mj_forward(self.model, self.data)
            self._draw_targets(arms)

        self.viewer.sync()

    def _handle_reset(self, arm):
        reset_count = getattr(arm.robot, "reset_count", 0)
        if reset_count == self._reset_counters.get(arm.name, 0):
            return
        self._reset_counters[arm.name] = reset_count
        self._move_to_robot_pose(arm, iterations=80)
        self._apply_controls(arm.name, list(arm.robot.pose_aa) + [arm.robot.gripper_norm])

    def _move_to_robot_pose(self, arm, iterations):
        state = self._arm_state[arm.name]
        target_pos = np.asarray(arm.robot.pose_rpy[:3], dtype=float) / 1000.0 + state["visual_offset"]
        target_rot = Transformations.rxryrz_to_rotation_matrix(*arm.robot.pose_aa[3:6])
        for _ in range(iterations):
            if self._solve_pose_ik(arm.name, target_pos, target_rot, iterations=1):
                break
        self.mujoco.mj_forward(self.model, self.data)

    def _action_pos_m(self, arm_name, action):
        return np.asarray(action[:3], dtype=float) / 1000.0 + self._arm_state[arm_name]["visual_offset"]

    def _solve_pose_ik(self, arm_name, target_pos_m, target_rot=None, iterations=10):
        state = self._arm_state[arm_name]
        site_id = state["site_id"]
        dof_ids = state["dof_ids"]
        qpos_ids = state["qpos_ids"]
        converged = False
        for _ in range(iterations):
            self.mujoco.mj_forward(self.model, self.data)
            pos_err = target_pos_m - self.data.site_xpos[site_id]
            rot_err = np.zeros(3, dtype=float)
            if target_rot is not None:
                site_rot = self.data.site_xmat[site_id].reshape(3, 3)
                rot_err = Transformations.rotation_matrix_to_rxryrz(target_rot @ site_rot.T)
            if np.linalg.norm(pos_err) < 0.003 and (target_rot is None or np.linalg.norm(rot_err) < 0.04):
                converged = True
                break

            jacp = np.zeros((3, self.model.nv), dtype=float)
            jacr = np.zeros((3, self.model.nv), dtype=float)
            self.mujoco.mj_jacSite(self.model, self.data, jacp, jacr, site_id)
            if target_rot is None:
                err = pos_err
                jac = jacp[:, dof_ids]
            else:
                err = np.concatenate((pos_err, self.ik_orientation_weight * rot_err))
                jac = np.vstack((jacp[:, dof_ids], self.ik_orientation_weight * jacr[:, dof_ids]))
            lhs = jac @ jac.T + (self.ik_damping**2) * np.eye(jac.shape[0])
            dq = jac.T @ np.linalg.solve(lhs, err)
            dq = np.clip(dq, -self.max_q_step, self.max_q_step)
            self.data.qpos[qpos_ids] += dq
            self._clamp_arm_qpos(arm_name)
        return converged

    def _clamp_arm_qpos(self, arm_name):
        state = self._arm_state[arm_name]
        for joint_id, qpos_id in zip(state["joint_ids"], state["qpos_ids"]):
            if not self.model.jnt_limited[joint_id]:
                continue
            low, high = self.model.jnt_range[joint_id]
            self.data.qpos[qpos_id] = np.clip(self.data.qpos[qpos_id], low, high)

    def _apply_controls(self, arm_name, action):
        state = self._arm_state[arm_name]
        self.data.ctrl[state["actuator_ids"]] = self.data.qpos[state["qpos_ids"]]
        if len(action) > 6:
            self.data.ctrl[state["gripper_actuator_id"]] = float(np.clip(action[6], 0.0, 1.0)) * 255.0

    def _draw_targets(self, arms):
        scene = self.viewer.user_scn
        scene.ngeom = 0
        for arm in arms:
            action = arm.robot.last_action
            if action is None:
                continue
            pos = self._action_pos_m(arm.name, action)
            rot = Transformations.rxryrz_to_rotation_matrix(*action[3:6])
            color = np.array([0.1, 0.35, 1.0, 1.0]) if arm.name == "L" else np.array([1.0, 0.45, 0.05, 1.0])
            self._add_sphere(scene, pos, color)
            self._add_axes(scene, pos, rot)

    def _add_sphere(self, scene, pos, rgba):
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        scene.ngeom += 1
        self.mujoco.mjv_initGeom(
            geom,
            self.mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.025, 0.0, 0.0]),
            pos,
            np.eye(3).reshape(-1),
            rgba,
        )

    def _add_axes(self, scene, pos, rot):
        colors = (
            np.array([1.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.8, 0.0, 1.0]),
            np.array([0.0, 0.2, 1.0, 1.0]),
        )
        for axis_id, color in enumerate(colors):
            if scene.ngeom >= scene.maxgeom:
                return
            endpoint = pos + rot[:, axis_id] * self.axis_length
            geom = scene.geoms[scene.ngeom]
            scene.ngeom += 1
            self.mujoco.mjv_connector(
                geom,
                self.mujoco.mjtGeom.mjGEOM_CAPSULE,
                0.006,
                pos,
                endpoint,
            )
            geom.rgba[:] = color

    def close(self):
        if not hasattr(self, "viewer"):
            return
        try:
            self.viewer.close()
        except Exception as exc:
            logger.warning("MuJoCo viewer close failed: %s", exc)
