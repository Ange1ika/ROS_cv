#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
import numpy as np
from geometry_msgs.msg import Point
from husky_tidy_bot_cv.srv import DetectArucoMarker, DetectArucoMarkerResponse
from threading import Lock

class ArucoNode:
    def __init__(self):
        # Инициализация ноды
        rospy.init_node('aruco_node', anonymous=True)

        # Параметры ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()

        # Инициализация конвертера изображений
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()

        # Подписка на топики
        self.image_sub = rospy.Subscriber('/realsense_gripper/color/image_raw/compressed', CompressedImage, self.image_callback)
        self.info_sub = rospy.Subscriber('/realsense_gripper/color/camera_info', CameraInfo, self.info_callback)

        # Публикация результата
        self.image_pub = rospy.Publisher('/aruco_image', Image, queue_size=1)

        # Сервис для детектирования ArUco меток
        rospy.loginfo("Service detect_aruco_marker is being registered.")
        self.service = rospy.Service('detect_aruco_marker', DetectArucoMarker, self.handle_detect_aruco_marker)
        rospy.loginfo("Service detect_aruco_marker has been registered.")

        # Мьютексы для защиты данных
        self.image_lock = Lock()
        self.info_lock = Lock()

        # Последние данные изображения и информации о камере
        self.latest_image = None
        self.latest_camera_info = None

        rospy.loginfo("Aruco node initialized")

    def image_callback(self, data):
        try:
            # Преобразование ROS CompressedImage в OpenCV формат
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            with self.image_lock:
                self.latest_image = cv_image
        except CvBridgeError as e:
            rospy.logerr(e)

    def info_callback(self, camera_info):
        # Обновляем параметры модели камеры
        with self.info_lock:
            self.camera_model.fromCameraInfo(camera_info)
            self.latest_camera_info = camera_info
        rospy.loginfo("Camera info updated")

    def handle_detect_aruco_marker(self, req):
        # Ожидание поступления изображения
        rospy.loginfo("Waiting for image data...")
        while True:
            with self.image_lock:
                if self.latest_image is not None:
                    cv_image = self.latest_image.copy()
                    break
            rospy.sleep(0.1)

        # Обнаружение меток
        detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, _ = detector.detectMarkers(cv_image)

        if ids is not None and req.marker_id in ids.flatten():
            index = list(ids.flatten()).index(req.marker_id)
            corner = corners[index][0]

            # Вычисление центра метки
            center_x = (corner[0][0] + corner[2][0]) / 2
            center_y = (corner[0][1] + corner[2][1]) / 2
            center = Point(x=center_x, y=center_y, z=0.0)

            # Вычисление угла поворота метки
            angle = cv2.minAreaRect(corners[index])[2]

            # Визуализация меток на изображении
            aruco.drawDetectedMarkers(cv_image, [corners[index]], np.array([[req.marker_id]]))
            try:
                # Преобразование OpenCV изображения обратно в ROS Image и публикация
                image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.image_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(e)

            return DetectArucoMarkerResponse(True, center, angle)

        return DetectArucoMarkerResponse(False, Point(), 0.0)

if __name__ == '__main__':
    aruco_node = ArucoNode()
    rospy.spin()

