#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import cv2.aruco as aruco
import numpy as np
import transforms3d as tfs
from geometry_msgs.msg import TransformStamped
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
import copy
import threading

# You might need a utility file for these functions or define them within the class
# For simplicity, I'll assume gvc-like functions are adapted or replaced.

def pq_to_se3(pos, quat):
    # input quat = [x, y, z, w] (xyzw)
    """Converts position and quaternion to a 4x4 SE(3) matrix."""
    se3 = tfs.affines.compose(pos, tfs.quaternions.quat2mat([quat[3], quat[0], quat[1], quat[2]]), [1, 1, 1])
    return se3
def se3_to_pq(se3):
    """Converts a 4x4 SE(3) matrix to position and quaternion."""
    pos, rot_mat, _, _ = tfs.affines.decompose(se3)
    quat = tfs.quaternions.mat2quat(rot_mat)  # w, x, y, z
    return pos, np.array([quat[1], quat[2], quat[3], quat[0]])  # Convert to xyzw order

def se3_to_transform_stamped(se3, parent_frame, child_frame, time):
    """Converts a 4x4 SE(3) matrix to a TransformStamped message."""
    pos, rot_mat, _, _ = tfs.affines.decompose(se3)
    quat = tfs.quaternions.mat2quat(rot_mat) # w, x, y, z

    ts = TransformStamped()
    ts.header.stamp = time
    ts.header.frame_id = parent_frame
    ts.child_frame_id = child_frame
    ts.transform.translation.x = pos[0]
    ts.transform.translation.y = pos[1]
    ts.transform.translation.z = pos[2]
    ts.transform.rotation.x = quat[1]
    ts.transform.rotation.y = quat[2]
    ts.transform.rotation.z = quat[3]
    ts.transform.rotation.w = quat[0]
    return ts

def transform_stamped_to_pq(transform_stamped):
    """Extracts position and quaternion from a TransformStamped message."""
    pos = np.array([
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z
    ])
    quat = np.array([
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w
    ])
    return pos, quat


class EyeInHandCalib(Node):

    def __init__(self):
        super().__init__("eih_calib")
        self.get_logger().info("Starting eih_calib node")

        # Configurations
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_info_topic', '/camera/color/camera_info'),
                ('rgb_topic', '/camera/color/image_raw'),
                ('marker_dict', 'DICT_6X6_250'),
                ('board_wh', [5, 7]),
                ('square_len', 0.0402),
                ('marker_len', 0.0320),
                ('camera_base_frame', 'camera_link'),
                ('camera_optical_frame', 'camera_color_optical_frame'),
                ('robot_base_frame', 'base_link'),
                ('robot_hand_frame', 'tool0') # same as the wrist_3_link for UR
            ])

        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.rgb_topic = self.get_parameter('rgb_topic').value
        marker_dict_name = self.get_parameter('marker_dict').value
        self.board_wh = self.get_parameter('board_wh').value
        self.square_len = self.get_parameter('square_len').value
        self.marker_len = self.get_parameter('marker_len').value
        self.camera_base_frame = self.get_parameter('camera_base_frame').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.robot_hand_frame = self.get_parameter('robot_hand_frame').value

        # Updated ArUco API for OpenCV 4.7+
        self.dictionary = aruco.getPredefinedDictionary(getattr(aruco, marker_dict_name))
        
        # Updated CharucoBoard creation for newer OpenCV versions
        try:
            # Try the new API first (OpenCV 4.7+)
            self.board = aruco.CharucoBoard(
                (self.board_wh[0], self.board_wh[1]), 
                self.square_len, self.marker_len, self.dictionary)
        except TypeError:
            # Fallback to older API if new one fails
            self.board = aruco.CharucoBoard_create(
                self.board_wh[0], self.board_wh[1], 
                self.square_len, self.marker_len, self.dictionary)

        self.ts_hand_to_cam = None

        self.samples = {
            "hand_world_rot": [],
            "hand_world_tr": [],
            "marker_camera_rot": [],
            "marker_camera_tr": [],
        }
        
        self.bridge = CvBridge()

        self.get_logger().info(f"Waiting for camera info: {self.camera_info_topic}")
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10)
        
        self.K = None
        self.D = None
        
        self.aruco_img_pub = self.create_publisher(Image, '/aruco_detect_img', 1)
        self.static_br = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d)
            self.get_logger().info(f"Got camera info\nK:\n{self.K}\nD:\n{self.D}")
            # Now that we have camera info, create the image subscriber
            self.rgb_sub = self.create_subscription(Image, self.rgb_topic, self.image_callback, 1)
            self.get_logger().info(f"Subscribing to RGB images on {self.rgb_topic}")
            # Destroy the info subscriber to avoid reprocessing
            self.destroy_subscription(self.camera_info_sub)

    def image_callback(self, rgb_msg):
        if self.K is None:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        # Updated detector parameters creation
        detector_params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(self.dictionary, detector_params)
        
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")

        # Updated marker detection API
        corners, ids, rejected = detector.detectMarkers(frame)
        
        if ids is not None:
            # Updated refineDetectedMarkers API
            try:
                # New API (OpenCV 4.7+)
                corners, ids, rejected, recovered = aruco.refineDetectedMarkers(
                    frame, self.board, corners, ids, rejected,
                    self.K, self.D, errorCorrectionRate=-1, 
                    parameters=detector_params)
            except TypeError:
                # Fallback for older versions
                corners, ids, rejected, recovered = aruco.refineDetectedMarkers(
                    frame, self.board, corners, ids, rejected,
                    self.K, self.D, errorCorrectionRate=-1, 
                    parameters=detector_params)
            
            # ChArUco corner detection and pose estimation
            try:
                # Detect ChArUco corners
                charuco_retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    corners, ids, frame, self.board, self.K, self.D)
                
                if charuco_retval > 3:  # Need at least 4 corners for pose estimation
                    # Estimate pose using ChArUco corners
                    retval, rvec, tvec = aruco.estimatePoseCharucoBoard(
                        charuco_corners, charuco_ids, self.board, self.K, self.D, None, None)
                    
                    if retval:
                        cv2.drawFrameAxes(frame, self.K, self.D, rvec, tvec, 0.2)
                        aruco.drawDetectedMarkers(frame, corners, ids)
                        if charuco_corners is not None:
                            aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)
                        
                        # Create and broadcast transform
                        rot = np.eye(4)
                        rot[:3, :3] = cv2.Rodrigues(rvec)[0]
                        # Convert rotation matrix to quaternion using transforms3d
                        quat_wxyz = tfs.quaternions.mat2quat(rot[:3, :3])
                        quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])

                        ts_cam_to_board = TransformStamped()
                        ts_cam_to_board.header.stamp = self.get_clock().now().to_msg()
                        ts_cam_to_board.header.frame_id = self.camera_optical_frame
                        ts_cam_to_board.child_frame_id = "aruco_markerboard"
                        ts_cam_to_board.transform.translation.x = float(tvec[0, 0])
                        ts_cam_to_board.transform.translation.y = float(tvec[1, 0])
                        ts_cam_to_board.transform.translation.z = float(tvec[2, 0])
                        ts_cam_to_board.transform.rotation.x = quat_xyzw[0]
                        ts_cam_to_board.transform.rotation.y = quat_xyzw[1]
                        ts_cam_to_board.transform.rotation.z = quat_xyzw[2]
                        ts_cam_to_board.transform.rotation.w = quat_xyzw[3]
                        
                        # Use a regular broadcaster for a frequently updated transform
                        br = tf2_ros.TransformBroadcaster(self)
                        br.sendTransform(ts_cam_to_board)
                        
            except Exception as e:
                self.get_logger().warn(f"ChArUco detection failed: {e}")

        self.aruco_img_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        # if self.ts_hand_to_cam is not None:
        #     self.static_br.sendTransform(self.ts_hand_to_cam)

    def transform_stamped_to_opencv_rot_tr(self, transform_stamped):
        pos, quat = transform_stamped_to_pq(transform_stamped)
        # Note the quaternion order for transforms3d (w, x, y, z)
        rot = tfs.quaternions.quat2mat([quat[3], quat[0], quat[1], quat[2]])
        return rot, pos


    def run_calibration_logic(self):
        while rclpy.ok():
            key = input("s: capture / c: compute / q: quit \n")
            if key == "s":
                try:
                    ts_base2hand = self.tf_buffer.lookup_transform(self.robot_base_frame, self.robot_hand_frame, rclpy.time.Time())
                    ts_cam2target = self.tf_buffer.lookup_transform(self.camera_optical_frame, "aruco_markerboard", rclpy.time.Time())
                    
                    (hbr, hbt) = self.transform_stamped_to_opencv_rot_tr(ts_base2hand)
                    (mcr, mct) = self.transform_stamped_to_opencv_rot_tr(ts_cam2target)
                    
                    self.samples["hand_world_rot"].append(hbr)
                    self.samples["hand_world_tr"].append(hbt)
                    self.samples["marker_camera_rot"].append(mcr)
                    self.samples["marker_camera_tr"].append(mct)
                    self.get_logger().info(f"Collected {len(self.samples['hand_world_rot'])} samples")

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().warn(f"Could not get transform: {e}. Try again.")
                    continue
            
            elif key == "c":
                self.get_logger().info("Computing calibration result")
                if len(self.samples["hand_world_rot"]) < 3:
                    self.get_logger().warn(f'Current number of samples: {len(self.samples["hand_world_rot"])}')
                    self.get_logger().warn("Eye-in-hand calibration requires at least 3 samples")
                    continue
                
                hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(
                    np.array(self.samples["hand_world_rot"]), np.array(self.samples["hand_world_tr"]), 
                    np.array(self.samples["marker_camera_rot"]), np.array(self.samples["marker_camera_tr"]), 
                    method=cv2.CALIB_HAND_EYE_DANIILIDIS)
                
                pos = np.squeeze(hand_camera_tr)
                quat_wxyz = tfs.quaternions.mat2quat(hand_camera_rot) # w,x,y,z
                quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])

                H_hand_to_cam_optical = pq_to_se3(pos, quat_xyzw)

                self.get_logger().info(f"Computed results {self.robot_hand_frame}_to_{self.camera_optical_frame}")
                self.get_logger().info(f"matrix: \n{str(H_hand_to_cam_optical)}")
                self.get_logger().info(f"pos: {str(pos)}, quat (xyzw): {str(quat_xyzw)}")

                ts_base_to_cam_optical = self.tf_buffer.lookup_transform(self.camera_base_frame, self.camera_optical_frame, rclpy.time.Time())
                pos, quat = transform_stamped_to_pq(ts_base_to_cam_optical)
                H_cam_base_to_cam_optical = pq_to_se3(pos, quat)
                H_hand_to_cam_base = np.matmul(H_hand_to_cam_optical, np.linalg.inv(H_cam_base_to_cam_optical))
                pos, quat = se3_to_pq(H_hand_to_cam_base)
                self.get_logger().info(f"Computed results {self.robot_hand_frame}_to_{self.camera_base_frame}")
                self.get_logger().info(f"matrix: \n{str(H_hand_to_cam_base)}")
                self.get_logger().info(f"pos: {str(pos)}, quat (xyzw): {str(quat)}")
                # Publish the static transform
                self.ts_hand_to_cam = se3_to_transform_stamped(H_hand_to_cam_base, self.robot_hand_frame, self.camera_base_frame, self.get_clock().now().to_msg())
                self.get_logger().info(f"Publishing the hand-eye calibration result as a static TF ({self.robot_hand_frame} -> {self.camera_base_frame})")
                self.static_br.sendTransform(self.ts_hand_to_cam)

            elif key == "q":
                self.get_logger().info("Quit")
                break
        
        # Shutdown rclpy
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    eih_calib_node = EyeInHandCalib()

    # Run the user input loop in a separate thread
    thread = threading.Thread(target=eih_calib_node.run_calibration_logic)
    thread.start()

    # Spin the node to process callbacks
    rclpy.spin(eih_calib_node)
    
    eih_calib_node.destroy_node()
    thread.join()

if __name__ == "__main__":
    main()