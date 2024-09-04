#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from husky_tidy_bot_cv.msg import Objects3d, Marker
from geometry_msgs.msg import Pose



def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                            [marker_size / 2, marker_size / 2, 0],
                            [marker_size / 2, -marker_size / 2, 0],
                            [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

class ArucoNode:
    def __init__(self):
        # Инициализация ноды
        rospy.init_node('aruco_node', anonymous=True)

        # Параметры ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()

        # Инициализация конвертера изображений
        self.bridge = CvBridge()

        # Модель камеры
        self.camera_model = PinholeCameraModel()

        # Подписка на топики
        print("podpiska na topik")
        self.image_sub = rospy.Subscriber('IMAGE_TOPIC', CompressedImage, self.image_callback)
        self.info_sub = rospy.Subscriber('INFO_TOPIC', CameraInfo, self.info_callback)

        # Публикация результата
        self.objects3d_pub = rospy.Publisher('/aruco_image', Objects3d, queue_size=1)

        # Матрица камеры и векторы искажений
        self.camera_matrix = None
        self.dist_coeffs = None

        rospy.loginfo("Aruco node initialized")

    def info_callback(self, camera_info):
        # Обновляем параметры модели камеры
        self.camera_model.fromCameraInfo(camera_info)
        self.camera_matrix = self.camera_model.fullIntrinsicMatrix()
        self.dist_coeffs = self.camera_model.distortionCoeffs()
        rospy.loginfo("Camera info updated")

    def image_callback(self, data):
        try:
            # Преобразование ROS CompressedImage в OpenCV формат
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            print("preobr cv2ros")
        except CvBridgeError as e:
            rospy.logerr(e)
            print("error preobr")
            return

        # Обнаружение меток
        detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, _ = detector.detectMarkers(cv_image)

        # Создание сообщения Objects3d
        objects3d_msg = Objects3d()
        objects3d_msg.header = data.header

        if ids is not None:
            objects3d_msg.num = len(ids)
            for i, corner in enumerate(corners):
                marker = Marker()
                marker.id = ids[i][0]
                marker.corners = corner.flatten().tolist()

                # Определение позы маркера (позиция и ориентация)
                rvec, tvec, _ = my_estimatePoseSingleMarkers(corner, 0.05, self.camera_matrix, self.dist_coeffs)
                pose = Pose()
                pose.position.x = tvec[0][0][0]
                pose.position.y = tvec[0][0][1]
                pose.position.z = tvec[0][0][2]
                # Конвертация вектора поворота в кватернион
                rotation_matrix, _ = cv2.Rodrigues(rvec[0][0])
                pose.orientation = self.rotation_matrix_to_quaternion(rotation_matrix)

                marker.pose = pose
                objects3d_msg.markers.append(marker)

                # Визуализация меток
                cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv_image = cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], 0.1)

        try:
            # Публикация результата
            self.objects3d_pub.publish(objects3d_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        # Преобразование матрицы поворота в кватернион
        q = [0] * 4
        t = np.trace(rotation_matrix)
        if t > 0:
            s = np.sqrt(t + 1.0) * 2
            q[0] = 0.25 * s
            q[1] = (rotation_matrix[2][1] - rotation_matrix[1][2]) / s
            q[2] = (rotation_matrix[0][2] - rotation_matrix[2][0]) / s
            q[3] = (rotation_matrix[1][0] - rotation_matrix[0][1]) / s
        else:
            if (rotation_matrix[0][0] > rotation_matrix[1][1]) and (rotation_matrix[0][0] > rotation_matrix[2][2]):
                s = np.sqrt(1.0 + rotation_matrix[0][0] - rotation_matrix[1][1] - rotation_matrix[2][2]) * 2
                q[0] = (rotation_matrix[2][1] - rotation_matrix[1][2]) / s
                q[1] = 0.25 * s
                q[2] = (rotation_matrix[0][1] + rotation_matrix[1][0]) / s
                q[3] = (rotation_matrix[0][2] + rotation_matrix[2][0]) / s
            elif rotation_matrix[1][1] > rotation_matrix[2][2]:
                s = np.sqrt(1.0 + rotation_matrix[1][1] - rotation_matrix[0][0] - rotation_matrix[2][2]) * 2
                q[0] = (rotation_matrix[0][2] - rotation_matrix[2][0]) / s
                q[1] = (rotation_matrix[0][1] + rotation_matrix[1][0]) / s
                q[2] = 0.25 * s
                q[3] = (rotation_matrix[1][2] + rotation_matrix[2][1]) / s
            else:
                s = np.sqrt(1.0 + rotation_matrix[2][2] - rotation_matrix[0][0] - rotation_matrix[1][1]) * 2
                q[0] = (rotation_matrix[1][0] - rotation_matrix[0][1]) / s
                q[1] = (rotation_matrix[0][2] + rotation_matrix[2][0]) / s
                q[2] = (rotation_matrix[1][2] + rotation_matrix[2][1]) / s
                q[3] = 0.25 * s
        return q

if __name__ == '__main__':
    try:
        # Создаем и запускаем ноду
        aruco_node = ArucoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

