import rospy
from husky_tidy_bot_cv.msg import Objects, Detection, Box, ObjectPose
from geometry_msgs.msg import PoseArray, Pose, Quaternion

# Global variable to store the latest pose message
latest_pose_msg = None

def objects_callback(objects_msg):
    global latest_pose_msg
    rospy.logdebug(f"Received Objects message: {objects_msg}")

    detection_msg = Detection()
    detection_msg.header = objects_msg.header
    detection_msg.classes_ids = objects_msg.classes_ids
    detection_msg.scores = objects_msg.scores
    detection_msg.tracking_ids = objects_msg.tracking_ids  # Tracking IDs from 2D tracking

    # Convert boxes
    detection_msg.boxes = []
    for obj_box in objects_msg.boxes:
        box = Box()
        box.x1 = obj_box.x1
        box.y1 = obj_box.y1
        box.x2 = obj_box.x2
        box.y2 = obj_box.y2
        detection_msg.boxes.append(box)
    rospy.logdebug(f"Converted {len(detection_msg.boxes)} boxes")

    # Convert poses from the latest received pose message
    if latest_pose_msg is not None:
        rospy.logdebug(f"Using latest pose message: {latest_pose_msg}")
        detection_msg.pose = PoseArray()
        detection_msg.pose.header = latest_pose_msg.header
        for i, pose in enumerate(latest_pose_msg.poses):
            new_pose = Pose()
            new_pose.position = pose.position
            if hasattr(pose, 'orientation'):
                new_pose.orientation = pose.orientation
            else:
                new_pose.orientation = Quaternion(0, 0, 0, 1)  # Default identity orientation
            detection_msg.pose.poses.append(new_pose)
        rospy.logdebug(f"Converted {len(detection_msg.pose.poses)} poses")
    else:
        rospy.logwarn("No latest pose message available")

    rospy.logdebug(f"Publishing Detection message: {detection_msg}")
    detection_pub.publish(detection_msg)

def pose_callback(pose_msg):
    global latest_pose_msg
    rospy.logdebug(f"Received ObjectPose message: {pose_msg}")
    latest_pose_msg = pose_msg
    rospy.logdebug(f"Updated latest pose message: {latest_pose_msg}")

if __name__ == "__main__":
    rospy.init_node('objects_to_detection_with_pose')

    # Subscribe to the 2D tracking output topic instead of segmentation
    rospy.Subscriber('/tracking', Objects, objects_callback)

    # Subscribe to the topic with poses
    rospy.Subscriber('/object_pose', ObjectPose, pose_callback)

    # Create a publisher for the new message
    detection_pub = rospy.Publisher('/detection_topic', Detection, queue_size=10)

    rospy.loginfo("Node started. Listening to /tracking and /object_pose, publishing to /detection_topic")
    rospy.spin()