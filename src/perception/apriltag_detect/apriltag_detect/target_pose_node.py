#!/usr/bin/env python3
"""Publish AprilTag target pose in OpenVINS global frame for tracking."""

import math
from typing import List, Optional, Tuple

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def _normalize_quaternion(q: Quaternion) -> Quaternion:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-12:
        out = Quaternion()
        out.w = 1.0
        return out
    out = Quaternion()
    out.x = q.x / norm
    out.y = q.y / norm
    out.z = q.z / norm
    out.w = q.w / norm
    return out


def _quaternion_to_matrix(q: Quaternion) -> List[List[float]]:
    q = _normalize_quaternion(q)
    x, y, z, w = q.x, q.y, q.z, q.w
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _matrix_to_quaternion(rot: List[List[float]]) -> Quaternion:
    m00, m01, m02 = rot[0]
    m10, m11, m12 = rot[1]
    m20, m21, m22 = rot[2]
    trace = m00 + m11 + m22
    q = Quaternion()
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q.w = 0.25 * s
        q.x = (m21 - m12) / s
        q.y = (m02 - m20) / s
        q.z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q.w = (m21 - m12) / s
        q.x = 0.25 * s
        q.y = (m01 + m10) / s
        q.z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q.w = (m02 - m20) / s
        q.x = (m01 + m10) / s
        q.y = 0.25 * s
        q.z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q.w = (m10 - m01) / s
        q.x = (m02 + m20) / s
        q.y = (m12 + m21) / s
        q.z = 0.25 * s
    return _normalize_quaternion(q)


def _transform_to_matrix(transform: TransformStamped) -> List[List[float]]:
    rot = _quaternion_to_matrix(transform.transform.rotation)
    trans = transform.transform.translation
    return [
        [rot[0][0], rot[0][1], rot[0][2], trans.x],
        [rot[1][0], rot[1][1], rot[1][2], trans.y],
        [rot[2][0], rot[2][1], rot[2][2], trans.z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _multiply_matrices(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    rows = len(a)
    cols = len(b[0])
    inner = len(b)
    out = [[0.0 for _ in range(cols)] for _ in range(rows)]
    for i in range(rows):
        for j in range(cols):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(inner))
    return out


def _matrix_to_pose(matrix: List[List[float]]) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = matrix[0][3]
    pose.pose.position.y = matrix[1][3]
    pose.pose.position.z = matrix[2][3]
    pose.pose.orientation = _matrix_to_quaternion(
        [
            [matrix[0][0], matrix[0][1], matrix[0][2]],
            [matrix[1][0], matrix[1][1], matrix[1][2]],
            [matrix[2][0], matrix[2][1], matrix[2][2]],
        ]
    )
    return pose


_IDENTITY4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


class TargetPoseNode(Node):
    def __init__(self) -> None:
        super().__init__('target_pose_node')

        self.declare_parameter('world_frame', 'global')
        self.declare_parameter('camera_frame', 'camera_infra1_optical_frame')
        self.declare_parameter('openvins_camera_frame', 'cam0')
        self.declare_parameter('target_tag_id', -1)
        self.declare_parameter('tf_timeout_sec', 0.05)
        self.declare_parameter('vio_tf_use_latest', True)
        self.declare_parameter('tag_tf_use_latest_fallback', True)
        self.declare_parameter('tf_warn_throttle_sec', 5.0)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('target_pose_topic', '/apriltag/target_pose_global')
        self.declare_parameter('target_detected_topic', '/apriltag/target_detected')
        self.declare_parameter('detections_topic', '/apriltag/detections')

        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.openvins_camera_frame = self.get_parameter('openvins_camera_frame').value
        self.target_tag_id = int(self.get_parameter('target_tag_id').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.vio_tf_use_latest = bool(self.get_parameter('vio_tf_use_latest').value)
        self.tag_tf_use_latest_fallback = bool(
            self.get_parameter('tag_tf_use_latest_fallback').value
        )
        self.tf_warn_throttle_sec = float(self.get_parameter('tf_warn_throttle_sec').value)

        pose_topic = self.get_parameter('target_pose_topic').value
        detected_topic = self.get_parameter('target_detected_topic').value
        detections_topic = self.get_parameter('detections_topic').value

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.detected_pub = self.create_publisher(Bool, detected_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_tf_warn_time = {}
        # OpenVINS and RealSense are separate TF trees; skip repeat lookups after first miss.
        self._cam0_optical_bridge_available: Optional[bool] = None

        self.latest_detections: Optional[AprilTagDetectionArray] = None
        self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self._detections_callback,
            qos_profile_sensor_data,
        )

        rate_hz = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.create_timer(1.0 / rate_hz, self._publish_target_pose)

        self.get_logger().info(
            'Publishing target pose in %s (vio_tf_use_latest=%s)'
            % (self.world_frame, self.vio_tf_use_latest)
        )

    def _detections_callback(self, msg: AprilTagDetectionArray) -> None:
        self.latest_detections = msg

    def _throttle_warn(self, key: str, message: str) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self._last_tf_warn_time.get(key, 0.0)
        if now - last >= self.tf_warn_throttle_sec:
            self._last_tf_warn_time[key] = now
            self.get_logger().warning(message)

    def _lookup_matrix(
        self,
        target_frame: str,
        source_frame: str,
        stamp,
        use_latest: bool,
        warn_key: str,
        timeout_sec: Optional[float] = None,
    ) -> Optional[List[List[float]]]:
        query_time = Time() if use_latest else stamp
        timeout = self.tf_timeout_sec if timeout_sec is None else timeout_sec
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                query_time,
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
        except TransformException as exc:
            mode = 'latest' if use_latest else 'stamped'
            self._throttle_warn(
                warn_key,
                'TF %s->%s (%s) failed: %s'
                % (target_frame, source_frame, mode, exc),
            )
            return None
        return _transform_to_matrix(transform)

    def _lookup_tag_tf(
        self,
        tag_frame: str,
        stamp,
    ) -> Optional[List[List[float]]]:
        # Stamped lookup uses zero timeout so a timestamp mismatch never blocks
        # the 30 Hz timer (apriltag TF stamps often differ from detection header).
        matrix = self._lookup_matrix(
            self.camera_frame,
            tag_frame,
            stamp,
            use_latest=False,
            warn_key='tag_stamped',
            timeout_sec=0.0,
        )
        if matrix is not None or not self.tag_tf_use_latest_fallback:
            return matrix
        return self._lookup_matrix(
            self.camera_frame,
            tag_frame,
            stamp,
            use_latest=True,
            warn_key='tag_latest',
        )

    def _select_detection(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return None
        if self.target_tag_id < 0:
            return msg.detections[0]
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                return detection
        return None

    def _tag_frame_candidates(self, detection) -> List[str]:
        return [
            'tag_%d' % detection.id,
            '%s:%d' % (detection.family, detection.id),
        ]

    def _resolve_tag_frame(self, detection, stamp) -> Tuple[Optional[str], Optional[List[List[float]]]]:
        for frame in self._tag_frame_candidates(detection):
            matrix = self._lookup_tag_tf(frame, stamp)
            if matrix is not None:
                return frame, matrix
        return None, None

    def _lookup_cam0_optical(self, stamp) -> List[List[float]]:
        if self._cam0_optical_bridge_available is False:
            return _IDENTITY4
        matrix = self._lookup_matrix(
            self.openvins_camera_frame,
            self.camera_frame,
            stamp,
            use_latest=self.vio_tf_use_latest,
            warn_key='vio_cam0_optical',
            timeout_sec=0.0,
        )
        if matrix is None:
            self._cam0_optical_bridge_available = False
            return _IDENTITY4
        self._cam0_optical_bridge_available = True
        return matrix

    def _publish_target_pose(self) -> None:
        detected = Bool()
        detected.data = False

        if self.latest_detections is None:
            self.detected_pub.publish(detected)
            return

        detection = self._select_detection(self.latest_detections)
        if detection is None:
            self.detected_pub.publish(detected)
            return

        stamp = self.latest_detections.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        tag_frame, t_optical_tag = self._resolve_tag_frame(detection, stamp)
        if t_optical_tag is None:
            self.detected_pub.publish(detected)
            return

        t_global_cam0 = self._lookup_matrix(
            self.world_frame,
            self.openvins_camera_frame,
            stamp,
            use_latest=self.vio_tf_use_latest,
            warn_key='vio_global_cam0',
        )
        if t_global_cam0 is None:
            self.detected_pub.publish(detected)
            return

        t_cam0_optical = self._lookup_cam0_optical(stamp)

        t_global_tag = _multiply_matrices(
            t_global_cam0,
            _multiply_matrices(t_cam0_optical, t_optical_tag),
        )

        pose = _matrix_to_pose(t_global_tag)
        pose.header.stamp = stamp
        pose.header.frame_id = self.world_frame

        self.pose_pub.publish(pose)
        detected.data = True
        self.detected_pub.publish(detected)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
