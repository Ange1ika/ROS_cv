import argparse
import rospy
from std_msgs.msg import Header, Float64MultiArray, Int32MultiArray
from geometry_msgs.msg import PoseArray
from husky_tidy_bot_cv.msg import ObjectPose_new, Box

class DataAggregatorNode:
    def __init__(self, scores_topic, classes_ids_topic, tracking_ids_topic, boxes_topic, poses_topic, output_topic):
        self.classes_ids = []
        self.scores = []
        self.tracking_ids = []
        self.boxes = []
        self.poses = PoseArray()

        self.scores_sub = rospy.Subscriber(scores_topic, Float64MultiArray, self.scores_callback)
        self.classes_ids_sub = rospy.Subscriber(classes_ids_topic, Int32MultiArray, self.classes_ids_callback)
        self.tracking_ids_sub = rospy.Subscriber(tracking_ids_topic, Int32MultiArray, self.tracking_ids_callback)
        self.boxes_sub = rospy.Subscriber(boxes_topic, Box, self.boxes_callback)
        self.poses_sub = rospy.Subscriber(poses_topic, PoseArray, self.poses_callback)

        self.publisher = rospy.Publisher(output_topic, ObjectPose_new, queue_size=10)

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
        self.boxes = msg.boxes  
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

#def build_parser():
#    parser = argparse.ArgumentParser()
#    parser.add_argument('-scores_topic', default='/openseed_node/scores')
#    parser.add_argument('-classes_ids_topic', default='/openseed_node/classes_ids')
#    parser.add_argument('-tracking_ids_topic', default='/openseed_node/tracking_ids')
#    parser.add_argument('-boxes_topic', default='/openseed_node/boxes')
#    parser.add_argument('-poses_topic', default='/object_pose_estimation_node/poses')
#    parser.add_argument('-output_topic', default='/object_pose_new')
#    return parser

    
def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-scores_topic', default='/openseed_node')
    parser.add_argument('-classes_ids_topic', default='/openseed_node')
    parser.add_argument('-tracking_ids_topic', default='/openseed_node')
    parser.add_argument('-boxes_topic', default='/openseed_node')
    parser.add_argument('-poses_topic', default='/object_pose_estimation_node')
    parser.add_argument('-output_topic', default='/object_pose_new')
    return parser

if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()

    rospy.init_node('data_aggregator_node')
    aggregator = DataAggregatorNode(
        args.scores_topic,
        args.classes_ids_topic,
        args.tracking_ids_topic,
        args.boxes_topic,
        args.poses_topic,
        args.output_topic
    )

    print("Spinning...")
    rospy.spin()

    # Explicitly delete the aggregator object
    print()
    del aggregator

