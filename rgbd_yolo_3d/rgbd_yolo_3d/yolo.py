import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from ultralytics import YOLO

from std_srvs.srv import SetBool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

import tf2_ros
from scipy.spatial.transform import Rotation as R

from ament_index_python.packages import get_package_share_directory
import os


def quat_from_yaw(yaw):
    q = R.from_euler('z', yaw).as_quat()
    return q[0], q[1], q[2], q[3]


class LicensePlate3D(Node):

    def __init__(self):
        super().__init__('license_plate_3d')

        self.bridge = CvBridge()
        self.enabled = True

        pkg_share = get_package_share_directory('rgbd_yolo_3d')
        model_path = os.path.join(
            pkg_share,
            'config',
            'license_plate_detector.pt'
        )
        self.model = YOLO(model_path)

        # camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        self.rgb = None
        self.depth = None

        self.camera_frame = 'realsense_rgb_optical_frame'
        self.output_frame = 'base_link'

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # subs
        self.sub_rgb = self.create_subscription(
            Image, '/realsense/image_raw', self.rgb_cb, 10)

        self.sub_depth = self.create_subscription(
            Image, '/realsense/depth/image_raw', self.depth_cb, 10)

        self.sub_info = self.create_subscription(
            CameraInfo, '/realsense/camera_info', self.info_cb, 10)

        # pubs
        self.pub_pose = self.create_publisher(
            PoseStamped, '/license_plate/pose', 10)

        self.pub_det3d = self.create_publisher(
            Detection3DArray, '/license_plate/detection3d', 10)
        
        self.enable_srv = self.create_service(
            SetBool,
            '/license_plate_3d/enable',
            self.enable_cb
        )

        self.get_logger().info('License plate 3D pose node started')

    def info_cb(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def rgb_cb(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process(msg.header.stamp)

    def depth_cb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if depth.dtype == np.uint16:
            self.depth = depth.astype(np.float32) / 1000.0
        else:
            self.depth = depth.astype(np.float32)

    def enable_cb(self, request, response):
        self.enabled = request.data
        state = 'ENABLED' if self.enabled else 'DISABLED'
        self.get_logger().info(f'LicensePlate3D {state}')
        response.success = True
        response.message = state
        return response


    def lookup_T(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame,
                self.camera_frame,
                rclpy.time.Time()
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            T = np.eye(4)
            T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T[:3, 3] = [t.x, t.y, t.z]
            return T
        except Exception as e:
            self.get_logger().warn(str(e))
            return None

    def compute_license_plate_pose(
        self, x1, y1, x2, y2, T_base_cam
    ):
        pts = []
        H, W = self.depth.shape[:2]

        # bbox shrink (depth edge noise 방지)
        mx = int(0.1 * (x2 - x1))
        my = int(0.1 * (y2 - y1))
        x1s, x2s = x1 + mx, x2 - mx
        y1s, y2s = y1 + my, y2 - my

        for v in range(y1s, y2s, 3):
            for u in range(x1s, x2s, 3):
                if u < 0 or u >= W or v < 0 or v >= H:
                    continue
                z = self.depth[v, u]
                if z < 0.2 or z > 5.0:
                    continue
                X = (u - self.cx) * z / self.fx
                Y = (v - self.cy) * z / self.fy
                pts.append([X, Y, z])

        if len(pts) < 50:
            return None

        pts = np.array(pts)
        pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])
        pts_base = (T_base_cam @ pts_h.T).T[:, :3]

        # 바닥 제거
        pts_base = pts_base[pts_base[:, 2] > 0.15]
        if pts_base.shape[0] < 30:
            return None

        # position (median)
        pos = np.median(pts_base, axis=0)

        # yaw (XY PCA)
        pts_xy = pts_base[:, :2]
        cov = np.cov(pts_xy.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        axis = eigvecs[:, np.argmax(eigvals)]
        yaw = np.arctan2(axis[1], axis[0])

        return pos, yaw

    def process(self, stamp):
        if not self.enabled:
            return
        
        if self.rgb is None or self.depth is None or self.fx is None:
            return

        T = self.lookup_T()
        if T is None:
            return

        results = self.model(self.rgb, conf=0.3)
        debug_img = self.rgb.copy()

        det_array = Detection3DArray()
        det_array.header.frame_id = self.output_frame
        det_array.header.stamp = stamp

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            if self.model.names[cls_id] != 'license_plate':
                continue

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            out = self.compute_license_plate_pose(x1, y1, x2, y2, T)
            if out is None:
                continue

            pos, yaw = out
            qx, qy, qz, qw = quat_from_yaw(yaw)

            # PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.output_frame
            pose_msg.header.stamp = stamp
            pose_msg.pose.position.x = float(pos[0])
            pose_msg.pose.position.y = float(pos[1])
            pose_msg.pose.position.z = float(pos[2])
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            self.pub_pose.publish(pose_msg)

            # Detection3D
            det = Detection3D()
            det.header = pose_msg.header
            det.bbox.center = pose_msg.pose

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'license_plate'
            hyp.hypothesis.score = float(box.conf[0])
            hyp.pose.pose = pose_msg.pose

            det.results.append(hyp)
            det_array.detections.append(det)

            # debug draw
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if det_array.detections:
            self.pub_det3d.publish(det_array)

        cv2.imshow('license_plate_3d', debug_img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = LicensePlate3D()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
