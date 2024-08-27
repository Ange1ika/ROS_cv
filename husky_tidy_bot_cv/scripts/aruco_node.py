#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from husky_tidy_bot_cv.msg import Objects3d, Marker

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
        #print("podpiska na topik")
        #self.image_sub = rospy.Subscriber('/camera2/camera2/color/image_raw/compressed', CompressedImage, self.image_callback)
        #self.info_sub = rospy.Subscriber('/camera2/camera2/color/camera_info', CameraInfo, self.info_callback)
        self.image_sub = rospy.Subscriber('IMAGE_TOPIC', CompressedImage, self.image_callback)
        self.info_sub = rospy.Subscriber('INFO_TOPIC', CameraInfo, self.info_callback)

        # Публикация результата
        self.objects3d_pub = rospy.Publisher('/aruco_image', Objects3d, queue_size=1)

        rospy.loginfo("Aruco node initialized")

    def image_callback(self, data):
        try:
            # Преобразование ROS CompressedImage в OpenCV формат
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            #print("preobr cv2ros")
        except CvBridgeError as e:
            rospy.logerr(e)
            #print("error preobr")
            return

        # Обнаружение меток
        detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, _ = detector.detectMarkers(cv_image)

        # Создание сообщения Objects3d
        objects3d_msg = Objects3d()
        objects3d_msg.header = data.header

        if ids is not None:
            for i, corner in enumerate(corners):
                marker = Marker()
                marker.id = ids[i][0]
                marker.corners = corner.flatten().tolist()
                objects3d_msg.markers.append(marker)

            # Визуализация меток
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)

        try:
            # Публикация результата
            self.objects3d_pub.publish(objects3d_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def info_callback(self, camera_info):
        # Обновляем параметры модели камеры
        self.camera_model.fromCameraInfo(camera_info)
        #rospy.loginfo("Camera info updated")

if __name__ == '__main__':
    try:
        # Создаем и запускаем ноду
        aruco_node = ArucoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
