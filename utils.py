import rospy
import tf2_ros
import ros_numpy
import numpy as np
import tf.transformations as t
import rospy
import copy
import image_geometry
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped, Vector3
import numpy as np
import numpy.matlib as npm


def subscribe_tf_transform_stamped(target_frame, source_frame, wait_time=1):
    """[summary] Return transform from target_frame to source_frame

    Args:
        target_frame ([type]): [description]
        source_frame ([type]): [description]
        wait_time (int, optional): [description]. Defaults to 1.

    Returns:
        [type]: [description]
    """

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10.0)
    itr = 0
    while not rospy.is_shutdown():
        try:
            transform_stamped = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            itr += 1
            if itr / 10 > wait_time:
                rospy.logwarn("Lookup the transfrom {} to {} for {} seconds, but failed!".format(target_frame, source_frame, wait_time))
                return None
            else:
                continue
        else:
            return transform_stamped

def publish_tf_transform_stamped(transform_stamped):
    br = tf2_ros.TransformBroadcaster()
    br.sendTransform(transform_stamped)
    rospy.spin()

def xyzw_to_wxyz(q):
    return [q[3], q[0], q[1], q[2]]

def pose_to_pq(pose):
    """ convert a ROS PoseS message into position/quaternion np arrays
    Args:
        pose (geometry_msgs/Pose): ROS geometric message to be converted
    Returns:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
    """
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    q = np.array([pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w])
    return p, q


def pose_stamped_to_pq(pose_stamped):
    """ convert a ROS PoseStamped message into position/quaternion np arrays
    Args:
        pose_stamped (geometry_msgs/PoseStamped): ROS geometric message to be converted
    Returns:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
    """
    return pose_to_pq(pose_stamped.pose)


def transform_to_pq(transform):
    """ convert a ROS Transform message into position/quaternion np arrays
    Args:
        transform (geometry_msgs/Transform): ROS geometric message to be converted
    Returns:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
    """
    p = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    q = np.array([transform.rotation.x, transform.rotation.y,
                  transform.rotation.z, transform.rotation.w])
    return p, q


def transform_to_pose(transform):
    """ convert a ROS Transform message into geometry_msgs/Pose
    Args:
        transform (geometry_msgs/Transform): ROS geometric message to be converted
    Returns:
        pose (geometry_msgs/Pose): ROS geometric message to be converted of given p and q
    """

    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation.x = transform.rotation.x
    pose.orientation.y = transform.rotation.y
    pose.orientation.z = transform.rotation.z
    pose.orientation.w = transform.rotation.w
    return pose


def transform_stamped_to_pq(transform_stamped):
    """ convert a ROS TransformStamped message into position/quaternion np arrays
    Args:
        transform_stamped (geometry_msgs/TransformStamped): ROS geometric message to be converted
    Returns:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
    """
    return transform_to_pq(transform_stamped.transform)


def transform_stamped_to_pose(transform_stamped):
    """ convert a ROS TransformStamped message into geometry_msgs/Pose
    Args:
        transform_stamped (geometry_msgs/TransformStamped): ROS geometric message to be converted
    Returns:
        pose (geometry_msgs/Pose): ROS geometric message to be converted of given p and q
    """
    return transform_to_pose(transform_stamped.transform)


def transform_stamped_to_se3(transform_stamped):
    """ convert a ROS TransformStamped message into position/quaternion np arrays
    Args:
        transform_stamped (geometry_msgs/TransformStamped): ROS geometric message to be converted
    Returns:
        se3 (np.array): a 4x4 SE(3) matrix as a numpy array
    """
    p, q = transform_to_pq(transform_stamped.transform)
    se3 = pq_to_se3(p,q)
    return se3


def msg_to_se3(msg):
    """ convert geometric ROS messages to SE(3)
    Args:
        msg (geometry_msgs/Pose, geometry_msgs/PoseStamped, 
        geometry_msgs/Transform, geometry_msgs/TransformStamped): ROS geometric messages to be converted
    Returns:
        se3 (np.array): a 4x4 SE(3) matrix as a numpy array
    source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
    """
    if isinstance(msg, Pose):
        p, q = pose_to_pq(msg)
    elif isinstance(msg, PoseStamped):
        p, q = pose_stamped_to_pq(msg)
    elif isinstance(msg, Transform):
        p, q = transform_to_pq(msg)
    elif isinstance(msg, TransformStamped):
        p, q = transform_stamped_to_pq(msg)
    else:
        raise TypeError("Invalid type for conversion to SE(3)")
    norm = np.linalg.norm(q)
    if np.abs(norm - 1.0) > 1e-3:
        raise ValueError(
            "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                str(q), np.linalg.norm(q)))
    elif np.abs(norm - 1.0) > 1e-6:
        q = q / norm
    se3 = t.quaternion_matrix(q)
    se3[0:3, -1] = p
    return se3


def pq_to_pose_stamped(p, q, target_frame, source_frame, stamp=None):
    """ convert position, quaternion to geometry_msgs/PoseStamped
    Args:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
        target_frame (string): name of tf target frame
        source_frame (string): name of tf source frame
    Returns:
        pose_stamped (geometry_msgs/PoseStamped): ROS geometric message to be converted of given p and q
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = target_frame
    if stamp is None: stamp = rospy.Time.now() 
    pose_stamped.header.stamp = stamp
    pose_stamped.child_frame_id = source_frame
    pose_stamped.pose = pq_to_pose(p, q)

    return pose_stamped


def pq_to_pose(p, q):
    """ convert position, quaternion to geometry_msgs/Pose
    Args:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    Returns:
        pose (geometry_msgs/Pose): ROS geometric message to be converted of given p and q
    """
    pose = Pose()
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

def pq_to_se3(p, q):
    """ convert position, quaternion to 4x4 SE(3)
    Args:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    Returns:
        se3 (np.array): 4x4 SE(3) 
    """
    
    se3 = t.quaternion_matrix(q)
    se3[:3, 3] = p
    return se3

def pq_to_transform(p, q):
    """ convert position, quaternion to geometry_msgs/Transform
    Args:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
    Returns:
        transform (geometry_msgs/Transform): ROS transform of given p and q
    """
    transform = Transform()
    transform.translation.x = p[0]
    transform.translation.y = p[1]
    transform.translation.z = p[2]
    transform.rotation.x = q[0]
    transform.rotation.y = q[1]
    transform.rotation.z = q[2]
    transform.rotation.w = q[3]
    return transform

def pq_to_transform_stamped(p, q, target_frame, source_frame, stamp=None):
    """ convert position, quaternion to geometry_msgs/TransformStamped
    Args:
        p (np.array): position array of [x, y, z]
        q (np.array): quaternion array of [x, y, z, w]
        target_frame (string): name of tf target frame
        source_frame (string): name of tf source frame
    Returns:
        transform_stamped (geometry_msgs/TransformStamped): ROS transform_stamped of given p and q
    """

    transform_stamped = TransformStamped()
    transform_stamped.header.frame_id = target_frame
    if stamp is None: stamp = rospy.Time.now() 
    transform_stamped.header.stamp = stamp
    transform_stamped.child_frame_id = source_frame
    transform_stamped.transform = pq_to_transform(p, q)

    return transform_stamped

def se3_to_transform(transform_nparray):
    """ convert 4x4 SE(3) to geometry_msgs/Transform
    Args:
        transform_nparray (np.array): 4x4 SE(3) 
    Returns:
        transform (geometry_msgs/Transform): ROS transform of given SE(3)
    """
    pos = transform_nparray[:3, 3] 
    quat = t.quaternion_from_matrix(transform_nparray)
    transform = pq_to_transform(pos, quat)
    return transform


def se3_to_transform_stamped(transform_nparray, target_frame, source_frame, stamp=None):
    """ convert 4x4 SE(3) to geometry_msgs/TransformStamped
    Args:
        transform_nparray (np.array): 4x4 SE(3) 
        source_frame (string): name of tf source frame
        target_frame (string): name of tf target frame
    Returns:
        transform_stamped (geometry_msgs/TransformStamped): ROS transform_stamped of given SE(3)
    """
    pos = transform_nparray[:3, 3] 
    quat = t.quaternion_from_matrix(transform_nparray)
    if stamp is None: stamp = rospy.Time.now() 
    transform_stamped = pq_to_transform_stamped(pos, quat, target_frame, source_frame, stamp)
    return transform_stamped

def se3_to_pq(se3):
    p = se3[:3, 3] 
    q = t.quaternion_from_matrix(se3)
    return p, q

def se3_to_pose(se3):
    """[summary] convert  to geometry_msgs/Pose

    Args:
        se3 (np.array): a 4x4 SE(3) matrix as a numpy array

    Returns:
        pose (geometry_msgs/Pose): ROS geometric message to be converted of given p and q
    """

    p = se3[:3, 3] 
    q = t.quaternion_from_matrix(se3)
    pose = Pose()
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def se3_to_pose_stamped(se3, target_frame, source_frame, stamp=None):
    """ convert 4x4 SE(3) to geometry_msgs/PoseStamped
    Args:
        se3 (np.array): a 4x4 SE(3) matrix as a numpy array
        target_frame (string): name of tf target frame
        source_frame (string): name of tf source frame
    Returns:
        pose_stamped (geometry_msgs/PoseStamped): ROS geometric message to be converted of given p and q
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = target_frame
    if stamp is None: stamp = rospy.Time.now() 
    pose_stamped.header.stamp = stamp
    pose_stamped.pose = se3_to_pose(se3)
    return pose_stamped