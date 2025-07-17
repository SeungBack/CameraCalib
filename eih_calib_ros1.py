#!/usr/bin/env python

import rospy
import ros_numpy
import cv2, cv_bridge              
import cv2.aruco as aruco

import transforms3d as tfs
import numpy as np
import tf.transformations as t
import utils as gvc
import tf2_ros
import copy
from sensor_msgs.msg import Image, CameraInfo


class EyeInHandCalib:

    def __init__(self):


        rospy.init_node("eih_calib")
        rospy.loginfo("Starting eih_calib node")

        ###############
        ## configurations
        # camera
        self.camera_info_topic = "/camera/color/camera_info"
        self.rgb_topic = '/camera/color/image_raw'
        # self.camera_info_topic = "/azure_kinect_12/rgb/camera_info"
        # self.rgb_topic = 'azure_kinect_12/rgb/image_raw'
        # marker board
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)   # dictionary id
        self.board_wh = [5, 7]
        self.square_len = 0.0327
        self.marker_len = 0.041 
        self.camera_base_frame = 'camera_link'
        self.camera_optical_frame = 'camera_color_optical_frame'
        # self.camera_base_frame = 'azure_kinect_12_camera_base'
        # self.camera_optical_frame = 'azure_kinect_12_rgb_camera_link'
        
        self.board = aruco.GridBoard_create(self.board_wh[0], self.board_wh[1], 
                                            self.square_len, self.marker_len, self.dictionary)
        # robot
        self.robot_base_frame = "base"
        self.robot_hand_frame = "tool0" # same as the wrist_3_link for UR
        self.ts_hand_to_cam = None
        ###############

        self.samples = {
            "hand_world_rot": [],
            "hand_world_tr": [],
            "marker_camera_rot": [],
            "marker_camera_tr": [],
                                }
        
        self.bridge = cv_bridge.CvBridge()


        rospy.loginfo("Waiting for camera info: {}".format(self.camera_info_topic))
        camera_info = rospy.wait_for_message(self.camera_info_topic, CameraInfo)
        #### before intrinsic calibration ####
        self.K = np.array(camera_info.K).reshape(3, 3)
        self.D = np.array(camera_info.D)
        #### after intrinsic calibration ####
        # self.K = np.array([[904.040734, 0, 651.973517],
        #                    [0, 904.756029, 372.392403],
        #                    [0, 0, 1]])
        # self.D = np.array([0.147420, -0.472030, 0, 0, 0.415045])
        #### after intrinsic calibration for 1920x1080####
        # self.K = np.array([[1358.977792, 0, 980.360612],
        #                    [0, 1360.623184, 559.798508],
        #                    [0, 0, 1]])
        # self.D = np.array([0.151053, -0.472815, 0, 0, 0.401638])
        
        rospy.loginfo(f"Got camera info\nK:\n{self.K}\nD:\n{self.D}")
        # self.K = np.array([1.3768229243954224e+03, 0., 9.8286031395270891e+02, 0., 1.3786953135011129e+03, 5.4394562588585768e+02, 0., 0., 1. ]).reshape(3, 3)
        # self.D =  np.array([ 1.9300402790103433e-01, -6.1719934315041469e-01, -1.0224342009699603e-04, -7.1887380220275927e-04, 5.7727360695362007e-01 ])
        rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.get_camera_pose_from_single_markerboard)
        
        rospy.loginfo("Publish aruco image to {}".format(self.rgb_topic))
        rospy.loginfo("Publish markerboard tf to /aruco_markerboard")
        self.aruco_img_pub = rospy.Publisher('/aruco_detect_img', Image, queue_size=1)

        self.br = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)


    def get_camera_pose_from_single_markerboard(self, rgb_msg):

        parameters =  aruco.DetectorParameters_create()
        # detect marker from image
        frame = copy.deepcopy(np.uint8(ros_numpy.numpify(rgb_msg)[:, :, :3]))
        corners, ids, rejected = aruco.detectMarkers(frame, self.dictionary, parameters=parameters)
        corners, ids, rejected, recovered = aruco.refineDetectedMarkers(frame, self.board, corners, ids, rejected,
                                                                            self.K, self.D,
                                                                            errorCorrectionRate=-1,
                                                                            parameters=parameters)
        N, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.K, self.D, None, None)
        
        if N:
            frame = cv2.drawFrameAxes(frame, self.K, self.D, rvec, tvec, 0.2)
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            self.aruco_img_pub.publish(self.bridge.cv2_to_imgmsg(frame))
            pos = tvec 
            rot = np.eye(4)
            rot[:3, :3] = np.squeeze(cv2.Rodrigues(rvec)[0])
            quat = t.quaternion_from_matrix(rot)
            
            ts_cam_to_board = gvc.pq_to_transform_stamped(pos, quat, self.camera_optical_frame, "aruco_markerboard")
            self.br.sendTransform(ts_cam_to_board)

        if self.ts_hand_to_cam is not None:
            self.br.sendTransform(self.ts_hand_to_cam)


    def transform_stamped_to_opencv_rot_tr(self, transform_stamped):
        
        pos, quat = gvc.transform_stamped_to_pq(transform_stamped)
        rot = tfs.quaternions.quat2mat((quat[3], quat[0], quat[1], quat[2]))
        return rot, pos

    def run(self):
        
        key = input("s: capture / c: compute / q: quit \n")
        if key == "s":
            
            """
                subscribe_tf_transform_stamped 함수는 target_frame에서 source_frame으로의 좌표계 변환을 구독하는 함수임.
                https://wiki.ros.org/tf/Overview/Using%20Published%20Transforms#lookupTransform  참고
                target_frame은 frame_id, source_frame은 child_frame_id
                해당 변환을 데이터에 적용하면, source_frame의 좌표를 target_frame의 좌표로 변환할 수 있음. (좌표계 변환과 역 방향)
            """
            ts_base2hand = gvc.subscribe_tf_transform_stamped(self.robot_base_frame, self.robot_hand_frame)
            ts_cam2target = gvc.subscribe_tf_transform_stamped(self.camera_base_frame, "aruco_markerboard")
            
            if ts_base2hand is None or ts_cam2target is None:
                rospy.logwarn("Could not get transform. Try again")
                return
            
            (hbr, hbt) = self.transform_stamped_to_opencv_rot_tr(ts_base2hand)      ## (hbr, hbt): base -> hand 좌표 변환  / ts_base2hand : base -> hand 좌표계 변환
            (mcr, mct) = self.transform_stamped_to_opencv_rot_tr(ts_cam2target)     ## (mcr, mct): cam -> target 좌표 변환 / ts_cam2target: cam -> target 좌표계 변환
            self.samples["hand_world_rot"].append(hbr)
            self.samples["hand_world_tr"].append(hbt)
            self.samples["marker_camera_rot"].append(mcr)
            self.samples["marker_camera_tr"].append(mct)
            rospy.loginfo("Collected {} samples".format(len(self.samples["hand_world_rot"])))
        
        if key == "c":
            rospy.loginfo("Computing calibration result")
            if len(self.samples["hand_world_rot"]) < 3:
                rospy.logwarn('current number of samples: {}'.format(len(self.samples["hand_world_rot"])))
                rospy.logwarn("eye in hand calibration requires at least 3 samples")
                return
            
            hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(
                self.samples["hand_world_rot"], self.samples["hand_world_tr"], 
                self.samples["marker_camera_rot"], self.samples["marker_camera_tr"], 
                method=cv2.CALIB_HAND_EYE_DANIILIDIS)
            result = tfs.affines.compose(np.squeeze(hand_camera_tr), hand_camera_rot, [1, 1, 1])
            rospy.loginfo("Computed calibration: {}".format(str(result)))
            (hcqw, hcqx, hcqy, hcqz) = [float(i) for i in tfs.quaternions.mat2quat(hand_camera_rot)]
            (hctx, hcty, hctz) = [float(i) for i in hand_camera_tr]
            pos = np.array([hctx, hcty, hctz])
            quat = np.array([hcqx, hcqy, hcqz, hcqw])
            H_hand_to_cambase = gvc.pq_to_se3(pos, quat)
            
            rospy.loginfo("Computed results {}_to_{}".format(self.robot_hand_frame, self.camera_base_frame))
            rospy.loginfo("matrix: \n{}".format(str(H_hand_to_cambase)))
            rospy.loginfo("pos: {}, quat: {}".format(str(pos), str(quat)))
            # rospy.loginfo("pos: {}, quat: {}".format(str(pos), str(quat)))
            self.ts_hand_to_cam = gvc.se3_to_transform_stamped(H_hand_to_cambase, self.robot_hand_frame, self.camera_base_frame)
            rospy.loginfo("Publishing the hand-eye calibration result as tf ({} -> {})".format(self.robot_hand_frame, self.camera_base_frame))

        if key == "q":
            rospy.loginfo("Quit")
            exit()
        
if __name__ == "__main__":
    
    eih_calib = EyeInHandCalib()
    while True:
        eih_calib.run()
        rospy.sleep(0.1)