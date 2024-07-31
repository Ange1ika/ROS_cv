#!/usr/bin/env python3

import argparse
import rospy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf

class ArucoDetectorNode:
    def __init__(self, image_topic, pose_topic, vis_topic):
        rospy.init_node('aruco_6dof_node', anonymous=True)

        # Подписка на топик изображений
        self.image_sub = rospy.Subscriber(image_topic, CompressedImage, self.image_callback)

        # Публикация поз ArUco меток
        self.pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)

        # Публикация визуализаций
        self.vis_pub = rospy.Publisher(vis_topic, Image, queue_size=10)

        # Публикация TF трансформаций
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Инициализация CvBridge для преобразования сообщений изображений ROS в OpenCV
        self.bridge = CvBridge()

        # Параметры ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Калибровка камеры (примерные значения)
        self.camera_matrix = np.array([[640, 0, 320], [0, 480, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)  # Поправить это на реальные коэффициенты калибровки

    def quaternion_from_matrix(self, matrix):
        qw = np.sqrt(1 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2]) / 2.0
        qx = (matrix[2, 1] - matrix[1, 2]) / (4.0 * qw)
        qy = (matrix[0, 2] - matrix[2, 0]) / (4.0 * qw)
        qz = (matrix[1, 0] - matrix[0, 1]) / (4.0 * qw)
        return [qw, qx, qy, qz]

    def detect_aruco(self, cv_image):
        # Преобразование изображения в градации серого
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Детектирование ArUco меток
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        markers = []
        if ids is not None:
            # Оценка позы для каждой метки
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.035, self.camera_matrix, self.dist_coeffs)
            for i, marker_id in enumerate(ids):
                marker = {}
                marker['id'] = int(marker_id[0])
                marker['position'] = tvecs[i][0]
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                marker['orientation'] = self.quaternion_from_matrix(rotation_matrix)
                markers.append(marker)
        return markers, corners, ids

    def image_callback(self, image_msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # Детектирование ArUco меток
            markers, corners, ids = self.detect_aruco(cv_image)

            # Публикация поз ArUco для каждой метки
            for marker in markers:
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = "camera_frame"
                pose_msg.pose.position.x = marker['position'][0]
                pose_msg.pose.position.y = marker['position'][1]
                pose_msg.pose.position.z = marker['position'][2]
                pose_msg.pose.orientation.x = marker['orientation'][1]
                pose_msg.pose.orientation.y = marker['orientation'][2]
                pose_msg.pose.orientation.z = marker['orientation'][3]
                pose_msg.pose.orientation.w = marker['orientation'][0]
                self.pose_pub.publish(pose_msg)

                # Публикация TF трансформации
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "camera_frame"
                t.child_frame_id = "aruco_marker_" + str(marker['id'])
                t.transform.translation.x = marker['position'][0]
                t.transform.translation.y = marker['position'][1]
                t.transform.translation.z = marker['position'][2]
                t.transform.rotation.x = marker['orientation'][1]
                t.transform.rotation.y = marker['orientation'][2]
                t.transform.rotation.z = marker['orientation'][3]
                t.transform.rotation.w = marker['orientation'][0]
                self.tf_broadcaster.sendTransform((t.transform.translation.x,
                                                   t.transform.translation.y,
                                                   t.transform.translation.z),
                                                  (t.transform.rotation.x,
                                                   t.transform.rotation.y,
                                                   t.transform.rotation.z,
                                                   t.transform.rotation.w),
                                                  t.header.stamp,
                                                  t.child_frame_id,
                                                  t.header.frame_id)

            # Визуализация ArUco меток на изображении
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Публикация визуализированного изображения
            vis_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.vis_pub.publish(vis_msg)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()

def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-topic', type=str, default="/camera/color/image_raw/compressed")
    parser.add_argument('--pose-topic', type=str, default="/aruco_pose")
    parser.add_argument('--vis-topic', type=str, default="/aruco_visualization")
    return parser

if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()

    try:
        node = ArucoDetectorNode(args.image_topic, args.pose_topic, args.vis_topic)
        node.run()
    except rospy.ROSInterruptException:
        pass

