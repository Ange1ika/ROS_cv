#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

class ArucoNode:
    def __init__(self):
        # Инициализация ноды
        rospy.init_node('aruco_node', anonymous=True)

        # Параметры ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()

        # Инициализация конвертера изображений
        self.bridge = CvBridge()
        print(dir(aruco))

        # Модель камеры
        self.camera_model = PinholeCameraModel()

        # Подписка на топики
        self.image_sub = rospy.Subscriber('/realsense_gripper/color/image_raw/compressed', CompressedImage, self.image_callback)
        self.info_sub = rospy.Subscriber('/realsense_gripper/color/camera_info', CameraInfo, self.info_callback)

        # Публикация результата
        self.image_pub = rospy.Publisher('/aruco_image', Image, queue_size=1)

        rospy.loginfo("Aruco node initialized")

    def image_callback(self, data):
        try:
            # Преобразование ROS CompressedImage в OpenCV формат
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Обнаружение меток
        detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        #corners, ids, _ = aruco.ArucoDetector.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        corners, ids, _ = detector.detectMarkers(cv_image)        

        # Если метки найдены, визуализируем их
        if ids is not None:
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
             
        try:
            # Преобразование OpenCV изображения обратно в ROS CompressedImage и публикация
            compressed_image_msg = self.bridge.cv2_to_imgmsg(cv_image)
            self.image_pub.publish(compressed_image_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def info_callback(self, camera_info):
        # Обновляем параметры модели камеры
        self.camera_model.fromCameraInfo(camera_info)
        rospy.loginfo("Camera info updated")

if __name__ == '__main__':
    try:
        # Создаем и запускаем ноду
        aruco_node = ArucoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
