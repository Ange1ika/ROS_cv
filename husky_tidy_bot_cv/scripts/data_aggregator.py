#!/usr/bin/env python
import rospy
from std_msgs.msg import Header, Float64MultiArray, Int32MultiArray
from geometry_msgs.msg import PoseArray
from husky_tidy_bot_cv.msg import ObjectPose_new, Box

class DataAggregator:
    def __init__(self):
        self.classes_ids = []
        self.scores = []
        self.tracking_ids = []
        self.boxes = []
        self.poses = PoseArray()

        rospy.Subscriber('/openseed_node/scores', Float64MultiArray, self.scores_callback)
        rospy.Subscriber('/openseed_node/classes_ids', Int32MultiArray, self.classes_ids_callback)
        rospy.Subscriber('/openseed_node/tracking_ids', Int32MultiArray, self.tracking_ids_callback)
        rospy.Subscriber('/openseed_node/boxes', Box, self.boxes_callback)
        rospy.Subscriber('/object_pose_estimation_node/poses', PoseArray, self.poses_callback)

        self.publisher = rospy.Publisher('/object_pose_new', ObjectPose_new, queue_size=10)

    def scores_callback(self, msg):
        self.scores = msg.data
        self.publish_data()

    def classes_ids_callback(self, msg):
        self.classes_ids = msg.data
        self.publish_data()

    def tracking_ids_callback(self, msg):
        self.tracking_ids = msg.data
        self.publish_data()

    def boxes_callback(self, msg):
        self.boxes = msg.boxes  # Изменено на msg.boxes, предполагая, что это массив Box
        self.publish_data()

    def poses_callback(self, msg):
        self.poses = msg
        self.publish_data()

    def publish_data(self):
        data = ObjectPose_new()
        data.header.stamp = rospy.Time.now()
        data.classes_ids = self.classes_ids
        data.scores = self.scores
        data.tracking_ids = self.tracking_ids
        data.boxes = self.boxes
        data.poses = self.poses

        self.publisher.publish(data)

if __name__ == '__main__':
    rospy.init_node('data_aggregator')
    aggregator = DataAggregator()
    rospy.spin()

