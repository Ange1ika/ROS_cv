import rospy
from sensor_msgs.msg import CompressedImage
from husky_tidy_bot_cv.msg import Objects
from std_srvs.srv import Empty
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from bot_sort import BoTSORT_wrapper
from kas_utils.visualization import draw_objects
from kas_utils.masks import get_full_masks
import numpy as np
import cv2
import time
import os
import os.path as osp
import message_filters

class BoTSORT_node(BoTSORT_wrapper):
    def __init__(self, segmentation_topic, image_topic, out_tracking_topic,
                 out_visualization_topic=None, **kwargs):
        super().__init__(**kwargs)

        self.segmentation_topic = segmentation_topic
        self.image_topic = image_topic
        self.out_tracking_topic = out_tracking_topic
        self.out_visualization_topic = out_visualization_topic

        self.tracking_pub = rospy.Publisher(self.out_tracking_topic, Objects, queue_size=10)
        self.tracking_ids_set = set()
        self.bridge = CvBridge()

        if self.out_visualization_topic:
            self.visualization_pub = rospy.Publisher(self.out_visualization_topic, CompressedImage, queue_size=10)
        else:
            self.visualization_pub = None

    def start(self):
        self.segmentation_sub = message_filters.Subscriber(self.segmentation_topic, Objects)
        self.image_sub = message_filters.Subscriber(self.image_topic, CompressedImage)  # Используем CompressedImage
        self.sync_sub = message_filters.TimeSynchronizer(
            [self.segmentation_sub, self.image_sub], 10)
        self.sync_sub.registerCallback(self.callback)

    def callback(self, segmentation_objects_msg, image_msg):
        scores, classes_ids, tracking_ids, boxes, masks_in_rois, rois, widths, heights = \
            from_objects_msg(segmentation_objects_msg)

        new_tracking_ids = set(tracking_ids)
        new_ids = new_tracking_ids - self.tracking_ids_set
        self.tracking_ids_set = new_tracking_ids

        for new_id in new_ids:
            self.call_point_cloud_service(new_id)

        # Обработка CompressedImage
        image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        tracked_objects = self.track(boxes, scores, classes_ids, image)
        tracked_objects_msg = to_objects_msg(segmentation_objects_msg.header, *tracked_objects)

        self.tracking_pub.publish(tracked_objects_msg)

        if self.visualization_pub:
            vis = image.copy()
            masks = get_full_masks(masks_in_rois, rois, widths, heights)
            draw_objects(vis, scores, tracking_ids, masks=masks)
            vis_msg = self.bridge.cv2_to_compressed_imgmsg(vis, dst_format='jpeg')  # Используем cv2_to_compressed_imgmsg
            vis_msg.header = segmentation_objects_msg.header
            self.visualization_pub.publish(vis_msg)

    def call_point_cloud_service(self, tracking_id):
        rospy.wait_for_service('/object_point_cloud_extraction/set_object_id')
        try:
            set_object_id = rospy.ServiceProxy('/object_point_cloud_extraction/set_object_id', Int32)
            set_object_id(tracking_id)
            rospy.loginfo(f"Called point cloud extraction service for tracking_id: {tracking_id}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    rospy.init_node("tracking_node")
    segmentation_topic = "/segmentation"
    image_topic = os.getenv("IMAGE_TOPIC")
    out_tracking_topic = "/tracking"
    out_visualization_topic = "/tracking_vis" if rospy.get_param("~enable_visualization", False) else None

    tracking_node = BoTSORT_node(segmentation_topic, image_topic, out_tracking_topic, out_visualization_topic)
    tracking_node.start()

    rospy.loginfo("Tracking node spinning...")
    rospy.spin()

