import rospy
from husky_tidy_bot_cv.msg import Objects, Objects3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from kas_utils.visualization import draw_objects  # импортируем draw_objects для визуализации

class ArucoSegmentCombiner:
    def __init__(self):
        rospy.loginfo("Initializing ArucoSegmentCombiner...")
        self.bridge = CvBridge()

        # Подписываемся на топики с использованием message_filters для синхронизации
        aruco_sub = message_filters.Subscriber("/aruco_image", Objects3d)
        segmentation_sub = message_filters.Subscriber("/segmentation_openseed", Objects)
        camera_sub = message_filters.Subscriber("/cam2/zed_node_1/left/image_rect_color/compressed  ", Image)

        # Создаем синхронизатор сообщений
        ts = message_filters.ApproximateTimeSynchronizer([aruco_sub, segmentation_sub, camera_sub], 10, 0.1)
        ts.registerCallback(self.callback)

        self.combined_pub = rospy.Publisher("/combined_detections", Objects, queue_size=10)
        self.visualization_pub = rospy.Publisher("/visualized_detections", Image, queue_size=10)

        # Параметры для визуализации
        self.palette = (
            (0, 0, 255), (180, 120, 120),
            (6, 230, 230), (80, 50, 50), (4, 200, 3), (120, 120, 80),
            (140, 140, 140), (204, 5, 255), (230, 230, 230), (4, 250, 7),
            (224, 5, 255), (235, 255, 7), (150, 5, 61), (120, 120, 70),
            (8, 255, 51), (255, 6, 82), (143, 255, 140), (204, 255, 4),
            (255, 51, 7), (204, 70, 3), (0, 102, 200), (61, 230, 250),
            (255, 6, 51), (11, 102, 255), (255, 7, 71), (255, 9, 224),
            (9, 7, 230), (220, 220, 220), (255, 9, 92), (112, 9, 255),
            (8, 255, 214), (7, 255, 224), (255, 184, 6), (10, 255, 71),
            (255, 41, 10), (7, 255, 255), (224, 255, 8), (102, 8, 255),
            (255, 61, 6), (255, 194, 7), (255, 122, 8), (0, 255, 20),
            (255, 8, 41), (255, 5, 153), (6, 51, 255), (235, 12, 255),
            (160, 150, 20), (0, 163, 255), (140, 140, 140), (250, 10, 15),
            (20, 255, 0), (31, 255, 0), (255, 31, 0), (255, 224, 0),
            (153, 255, 0), (255, 71, 0), (0, 235, 255),
            (0, 173, 255), (31, 0, 255), (11, 200, 200), (255, 82, 0),
            (0, 255, 245), (0, 61, 255), (0, 255, 112), (0, 255, 133),
            (255, 0, 0), (255, 163, 0), (255, 102, 0), (194, 255, 0),
            (0, 143, 255), (51, 255, 0), (0, 82, 255), (0, 255, 41),
            (0, 255, 173), (10, 0, 255), (173, 255, 0),
            (0, 255, 153), (255, 92, 0), (255, 0, 255), (255, 0, 245),
            (255, 0, 102), (255, 173, 0), (255, 0, 20), (255, 184, 184),
            (0, 31, 255), (0, 255, 61), (0, 71, 255), (255, 0, 204),
            (0, 255, 194), (0, 255, 82), (0, 10, 255), (0, 112, 255),
            (51, 0, 255), (0, 194, 255), (0, 122, 255), (0, 255, 163),
            (255, 153, 0), (0, 255, 10), (255, 112, 0), (143, 255, 0),
            (82, 0, 255), (163, 255, 0), (255, 235, 0), (8, 184, 170),
            (133, 0, 255), (0, 255, 92), (184, 0, 255), (255, 0, 31),
            (0, 184, 255), (0, 214, 255), (255, 0, 112), (92, 255, 0),
            (0, 224, 255), (112, 224, 255), (70, 184, 160), (163, 0, 255),
            (153, 0, 255), (71, 255, 0), (255, 0, 163), (255, 204, 0),
            (255, 0, 143), (0, 255, 235), (133, 255, 0), (255, 0, 235),
            (245, 0, 255), (255, 0, 122), (255, 245, 0), (10, 190, 212),
            (214, 255, 0), (0, 204, 255), (20, 0, 255), (255, 255, 0),
            (0, 153, 255), (0, 41, 255), (0, 255, 204), (41, 0, 255),
            (41, 255, 0), (173, 0, 255), (0, 245, 255), (71, 0, 255),
            (122, 0, 255), (0, 255, 184), (0, 92, 255), (184, 255, 0),
            (0, 133, 255), (255, 214, 0), (25, 194, 194), (102, 255, 0),
            (92, 0, 255)
        )

    def callback(self, aruco_data, segmentation_data, image_msg):
        rospy.loginfo("Received synchronized ArUco, segmentation data, and camera image.")

        # Отладочный вывод: выводим информацию о полученных сообщениях
        rospy.loginfo(f"ArUco data: {aruco_data}")
        rospy.loginfo(f"Segmentation data: {segmentation_data}")
        rospy.loginfo(f"Image message: {image_msg}")

        if aruco_data is None or segmentation_data is None or image_msg is None:
            rospy.logwarn("One of the input data streams is empty!")
            return

        self.aruco_data = aruco_data
        self.segmentation_data = segmentation_data
        self.image_msg = image_msg

        # Комбинируем данные только если они валидны
        if self.aruco_data and self.segmentation_data and self.image_msg:
            self.combine_data()
        else:
            rospy.logwarn("Data is not available yet.")

    def combine_data(self):
        rospy.loginfo("Combining ArUco, segmentation data, and camera image...")

        if self.aruco_data is None or self.segmentation_data is None or self.image_msg is None:
            rospy.logwarn("Data is not available yet.")
            return

        combined_detection = self.create_combined_detection()
        visualized_msg = self.create_visualized_msg(combined_detection)

        self.publish_combined_detection(combined_detection)
        self.publish_visualization(visualized_msg)

    def create_combined_detection(self):
        combined_detection = Objects()
        combined_detection.header = self.segmentation_data.header
        combined_detection.num = self.segmentation_data.num
        combined_detection.scores = self.segmentation_data.scores
        combined_detection.classes_ids = self.segmentation_data.classes_ids
        combined_detection.tracking_ids = self.segmentation_data.tracking_ids
        combined_detection.boxes = self.segmentation_data.boxes
        combined_detection.masks = self.segmentation_data.masks
        combined_detection.marker_ids = []

        for i, mask in enumerate(self.segmentation_data.masks):
            rospy.loginfo(f"Processing mask {i}")

            # Восстанавливаем полную маску из ROI
            mask_img = self.get_full_mask(mask)

            # Преобразуем маску в numpy массив
            if mask_img is None:
                rospy.logwarn(f"Mask {i} is empty or invalid!")
                continue

            rospy.loginfo(f"Mask {i}: size {mask_img.shape}")

            marker_ids = self.find_markers_in_mask(mask_img)
            unique_marker_id = self.determine_unique_marker_id(marker_ids)
            rospy.loginfo(f"Mask {i}: Marker IDs found: {marker_ids}, Unique ID: {unique_marker_id}")
            combined_detection.marker_ids.append(unique_marker_id)

        return combined_detection

    def get_full_mask(self, mask):
        """
        Восстанавливаем полную маску из ROI и маски в ROI.
        """
        full_mask = np.zeros((mask.height, mask.width), dtype=np.uint8)
        mask_img = np.frombuffer(mask.mask_in_roi, dtype=np.uint8).reshape(mask.roi.height, mask.roi.width)

        full_mask[mask.roi.y:mask.roi.y+mask.roi.height, mask.roi.x:mask.roi.x+mask.roi.width] = mask_img
        return full_mask

    def create_visualized_msg(self, combined_detection):
        # Преобразуем изображение из сообщения в формат OpenCV
        image = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")

        # Визуализируем ArUco маркеры на изображении
        for marker in self.aruco_data.markers:
            corners = np.array(marker.corners).reshape((4, 2)).astype(int)
            cv2.polylines(image, [corners], isClosed=True, color=(0, 0, 255), thickness=2)
            center = np.mean(corners, axis=0).astype(int)
            cv2.putText(image, str(marker.id), tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            rospy.loginfo(f"ArUco Marker {marker.id} at position {center}")

        # Визуализация сегментированных объектов
        boxes = [(box.x1, box.y1, box.x2, box.y2) for box in self.segmentation_data.boxes]
        draw_objects(image, self.segmentation_data.scores, self.segmentation_data.classes_ids,
                    boxes=boxes, masks=self.segmentation_data.masks,
                    draw_scores=True, draw_ids=True, draw_masks=True, draw_boxes=True,
                    palette=self.palette, color_by_object_id=True)

        vis_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        vis_msg.header = self.image_msg.header
        return vis_msg

    def find_markers_in_mask(self, mask_img):
        marker_ids = []
        for marker in self.aruco_data.markers:
            corners = np.array(marker.corners).reshape((4, 2)).astype(int)
            marker_mask = np.zeros_like(mask_img)

            # Create a filled polygon for the marker
            cv2.fillPoly(marker_mask, [corners], 255)

            # Check intersection of the marker polygon with the segmentation mask
            intersection = cv2.bitwise_and(mask_img, marker_mask)
            if np.any(intersection):
                marker_ids.append(marker.id)
        return marker_ids

    def determine_unique_marker_id(self, marker_ids):
        if marker_ids:
            return max(set(marker_ids), key=marker_ids.count)
        return -1  

    def publish_combined_detection(self, combined_detection):
        self.combined_pub.publish(combined_detection)

    def publish_visualization(self, visualized_msg):
        self.visualization_pub.publish(visualized_msg)


if __name__ == "__main__":
    rospy.init_node("aruco_segment_combiner_node")

    combiner = ArucoSegmentCombiner()

    rospy.loginfo("Aruco-Segment Combiner node spinning...")
    rospy.spin()
