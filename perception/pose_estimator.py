import math

from perception.types import ObjectPose
from perception.marker_detector import ArucoMarkerDetector
from perception.tnt_detector import detect_tnt_pose
from perception.quality import reprojection_error, confidence_from_reprojection
from control.transforms import (
    make_transform,
    matmul,
    quaternion_from_rotation_matrix,
    rotation_matrix_from_quaternion,
    extract_translation,
    extract_rotation,
)
from control.errors import PerceptionError


class PoseEstimator:
    def __init__(self, camera, perception_cfg: dict, markers_cfg: dict, transforms_cfg: dict, frames_cfg: dict, camera_cfg: dict, tnt_cfg: dict | None = None):
        self.camera = camera
        self.mode = (perception_cfg.get("mode") or "mock").lower()
        self.min_confidence = float(perception_cfg.get("min_confidence", 0.0))
        self.max_reproj_error = float(perception_cfg.get("max_reprojection_error_px", 1.0))
        self.marker_id = int(markers_cfg.get("marker_id", 0))
        self.marker_size_mm = float(markers_cfg.get("marker_size_mm", 50.0))
        self.marker_T_obj = markers_cfg.get("marker_T_obj")
        self.base_T_cam = transforms_cfg.get("base_T_cam")
        self.frames = frames_cfg
        self.camera_cfg = camera_cfg
        self.tnt_cfg = tnt_cfg or {}
        self._mock_pose = perception_cfg.get("mock_pose") or {}
        self.detector = ArucoMarkerDetector(markers_cfg)

    def set_mock_pose(self, position_mm, quaternion_xyzw, confidence):
        self._mock_pose = {
            "position_mm": list(position_mm),
            "quaternion_xyzw": list(quaternion_xyzw),
            "confidence": float(confidence),
        }

    def detect_object_pose(self) -> ObjectPose:
        if self.mode == "mock":
            return self._mock_object_pose()
        if self.mode == "tnt":
            return self._detect_tnt_pose()
        return self._detect_from_camera()

    def _mock_object_pose(self) -> ObjectPose:
        position = self._mock_pose.get("position_mm", [0.0, 0.0, 0.0])
        quaternion = self._mock_pose.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        confidence = float(self._mock_pose.get("confidence", 0.0))
        if confidence < self.min_confidence:
            raise PerceptionError("Mock pose confidence below threshold")
        R = rotation_matrix_from_quaternion(quaternion)
        T = make_transform(R, position)
        return ObjectPose(
            frame_id=self.frames.get("object", "obj"),
            parent_frame=self.frames.get("base", "base"),
            matrix=T,
            position_mm=tuple(position),
            quaternion_xyzw=tuple(quaternion),
            confidence=confidence,
        )

    def _detect_from_camera(self) -> ObjectPose:
        frame = self.camera.get_frame()
        observations = self.detector.detect(frame.image)
        target = None
        for obs in observations:
            if obs.marker_id == self.marker_id:
                target = obs
                break
        if target is None:
            raise PerceptionError("Marker not found")

        try:
            import numpy as np
            import cv2
        except Exception as exc:
            raise PerceptionError("OpenCV is required for real detection") from exc

        s = self.marker_size_mm
        obj_pts = np.array([
            [-s / 2.0, s / 2.0, 0.0],
            [s / 2.0, s / 2.0, 0.0],
            [s / 2.0, -s / 2.0, 0.0],
            [-s / 2.0, -s / 2.0, 0.0],
        ], dtype=np.float32)
        img_pts = target.corners.reshape(-1, 2).astype(np.float32)

        cam_cfg = self.camera_cfg
        intr = cam_cfg.get("intrinsics", {})
        camera_matrix = np.array([
            [float(intr.get("fx", 1.0)), 0.0, float(intr.get("cx", 0.0))],
            [0.0, float(intr.get("fy", 1.0)), float(intr.get("cy", 0.0))],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        cfg_size = cam_cfg.get("image_size")
        cx = float(intr.get("cx", 0.0))
        cy = float(intr.get("cy", 0.0))
        if isinstance(cfg_size, (list, tuple)) and len(cfg_size) >= 2:
            try:
                w_cfg = float(cfg_size[0])
                h_cfg = float(cfg_size[1])
                if cx > 0.0 and cy > 0.0:
                    if abs(cx - w_cfg / 2.0) > 0.15 * w_cfg or abs(cy - h_cfg / 2.0) > 0.15 * h_cfg:
                        cfg_size = [cx * 2.0, cy * 2.0]
            except Exception:
                cfg_size = None
        else:
            if cx > 0.0 and cy > 0.0:
                cfg_size = [cx * 2.0, cy * 2.0]
            else:
                cfg_size = None
        try:
            h, w = frame.image.shape[:2]
        except Exception:
            h, w = None, None
        if cfg_size and w and h:
            sx = float(w) / float(cfg_size[0]) if cfg_size[0] else 1.0
            sy = float(h) / float(cfg_size[1]) if cfg_size[1] else 1.0
            camera_matrix[0, 0] *= sx
            camera_matrix[1, 1] *= sy
            camera_matrix[0, 2] *= sx
            camera_matrix[1, 2] *= sy
        dist = np.array(cam_cfg.get("distortion", [0, 0, 0, 0, 0]), dtype=np.float32)

        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist)
        if not ok:
            raise PerceptionError("solvePnP failed")

        error_px = reprojection_error(obj_pts, img_pts, rvec, tvec, camera_matrix, dist)
        confidence = confidence_from_reprojection(error_px, self.max_reproj_error)
        if confidence < self.min_confidence:
            raise PerceptionError("Pose confidence below threshold")

        R_cam, _ = cv2.Rodrigues(rvec)
        cam_T_marker = make_transform(R_cam.tolist(), tvec.reshape(-1).tolist())
        if not self.marker_T_obj:
            raise PerceptionError("marker_T_obj missing in config")
        cam_T_obj = matmul(cam_T_marker, self.marker_T_obj)

        if not self.base_T_cam:
            raise PerceptionError("base_T_cam missing in config")
        base_T_obj = matmul(self.base_T_cam, cam_T_obj)

        R_base = extract_rotation(base_T_obj)
        quat = quaternion_from_rotation_matrix(R_base)
        pos = extract_translation(base_T_obj)

        return ObjectPose(
            frame_id=self.frames.get("object", "obj"),
            parent_frame=self.frames.get("base", "base"),
            matrix=base_T_obj,
            position_mm=tuple(pos),
            quaternion_xyzw=tuple(quat),
            confidence=confidence,
        )

    def _detect_tnt_pose(self) -> ObjectPose:
        frame = self.camera.get_frame()
        cam_T_obj = detect_tnt_pose(frame.image, self.camera_cfg, self.tnt_cfg, camera=self.camera)
        if cam_T_obj is None:
            raise PerceptionError("TNT cube not found")
        if not self.base_T_cam:
            raise PerceptionError("base_T_cam missing in config")
        base_T_obj = matmul(self.base_T_cam, cam_T_obj)
        R_base = extract_rotation(base_T_obj)
        quat = quaternion_from_rotation_matrix(R_base)
        pos = extract_translation(base_T_obj)
        return ObjectPose(
            frame_id=self.frames.get("object", "obj"),
            parent_frame=self.frames.get("base", "base"),
            matrix=base_T_obj,
            position_mm=tuple(pos),
            quaternion_xyzw=tuple(quat),
            confidence=1.0,
        )
