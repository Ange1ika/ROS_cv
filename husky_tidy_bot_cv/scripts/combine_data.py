import rospy
from husky_tidy_bot_cv.msg import Objects, Objects3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoSegmentCombiner:
    def __init__(self):
        rospy.loginfo("Initializing ArucoSegmentCombiner...")
        self.bridge = CvBridge()
        self.aruco_sub = rospy.Subscriber("/aruco_image", Objects3d, self.aruco_callback)
        self.segmentation_sub = rospy.Subscriber("/segmentation_vis", Objects, self.segmentation_callback)
        self.combined_pub = rospy.Publisher("/combined_detections", Objects, queue_size=10)
        self.visualization_pub = rospy.Publisher("/visualized_detections", Image, queue_size=10)

        self.aruco_data = None
        self.segmentation_data = None

    def aruco_callback(self, data):
        rospy.loginfo("Received ArUco data.")
        rospy.loginfo(f"ArUco data: {data}")
        self.aruco_data = data

    def segmentation_callback(self, data):
        rospy.loginfo("Received segmentation data.")
        rospy.loginfo(f"Segmentation data: {data}")
        self.segmentation_data = data
        self.combine_data()


    def combine_data(self):
        rospy.loginfo("Combining ArUco and segmentation data...")
        if self.aruco_data is None:
            rospy.logwarn("ArUco data is not available yet.")
            return
        if self.segmentation_data is None:
            rospy.logwarn("Segmentation data is not available yet.")
            return

        rospy.loginfo("ArUco markers count: {}".format(len(self.aruco_data.markers)))
        rospy.loginfo("Segmentation masks count: {}".format(len(self.segmentation_data.masks)))
        
        combined_detection = Objects()
        combined_detection.header = self.segmentation_data.header
        combined_detection.num = self.segmentation_data.num
        combined_detection.scores = self.segmentation_data.scores
        combined_detection.classes_ids = self.segmentation_data.classes_ids
        combined_detection.tracking_ids = self.segmentation_data.tracking_ids
        combined_detection.boxes = self.segmentation_data.boxes
        combined_detection.masks = self.segmentation_data.masks
        combined_detection.marker_ids = []

        segmented_img = self.bridge.imgmsg_to_cv2(self.segmentation_data.image, "bgr8")
        rospy.loginfo("Segmented image size: {}".format(segmented_img.shape))

        for i, mask in enumerate(self.segmentation_data.masks):
            mask_img = self.bridge.imgmsg_to_cv2(mask, "mono8")
            rospy.loginfo("Processing mask {}: size {}".format(i, mask_img.shape))
            marker_ids = self.find_markers_in_mask(mask_img)
            unique_marker_id = self.determine_unique_marker_id(marker_ids)
            rospy.loginfo(f"Mask {i}: Marker IDs found: {marker_ids}, Unique ID: {unique_marker_id}")
            combined_detection.marker_ids.append(unique_marker_id)

            segmented_img[mask_img > 0] = [0, 255, 0]  # Зеленый цвет для масок

        for marker in self.aruco_data.markers:
            corners = np.array(marker.corners).reshape((4, 2)).astype(int)
            cv2.polylines(segmented_img, [corners], isClosed=True, color=(0, 0, 255), thickness=2)  # Красный цвет для ArUco
            center = np.mean(corners, axis=0).astype(int)
            cv2.putText(segmented_img, str(marker.id), tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            rospy.loginfo(f"ArUco Marker {marker.id} at position {center}")

        self.combined_pub.publish(combined_detection)
        rospy.loginfo("Published combined detection.")
        visualized_msg = self.bridge.cv2_to_imgmsg(segmented_img, "bgr8")
        self.visualization_pub.publish(visualized_msg)
        rospy.loginfo("Published visualized detections.")


    def find_markers_in_mask(self, mask_img):
        marker_ids = []
        for marker in self.aruco_data.markers:
            marker_img = self.bridge.imgmsg_to_cv2(marker.image, "mono8")
            if self.is_marker_in_mask(mask_img, marker_img):
                marker_ids.append(marker.id)
        return marker_ids

    def determine_unique_marker_id(self, marker_ids):
        if marker_ids:
            return max(set(marker_ids), key=marker_ids.count)
        return -1  

    def is_marker_in_mask(self, mask_img, marker_img):
        intersection = cv2.bitwise_and(mask_img, marker_img)
        rospy.loginfo(f"Intersection sum: {np.sum(intersection)}")  # Вывод суммы пересечения
        return np.any(intersection)


if __name__ == "__main__":
    rospy.init_node("aruco_segment_combiner")
    combiner = ArucoSegmentCombiner()
    rospy.spin()

