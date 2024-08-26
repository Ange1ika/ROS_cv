import rospy
from husky_tidy_bot_cv.msg import Objects, Detection, Box,  ObjectPose
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

# Глобальная переменная для хранения последнего сообщения с позами
latest_pose_msg = None

def objects_callback(objects_msg):
    global latest_pose_msg

    detection_msg = Detection()
    detection_msg.header = objects_msg.header
    detection_msg.classes_ids = objects_msg.classes_ids
    detection_msg.scores = objects_msg.scores
    detection_msg.tracking_ids = objects_msg.tracking_ids  # Оставьте пустым, если пусто

    # Преобразование боксов
    detection_msg.boxes = []
    for obj_box in objects_msg.boxes:
        box = Box()
        box.x1 = obj_box.x1
        box.y1 = obj_box.y1
        box.x2 = obj_box.x2
        box.y2 = obj_box.y2
        detection_msg.boxes.append(box)

    # Преобразование поз из последнего полученного сообщения с позами
    if latest_pose_msg is not None:
        detection_msg.pose = PoseArray()
        detection_msg.pose.header = latest_pose_msg.header
        
        for i in range(len(latest_pose_msg.positions)):
            pose = Pose()
            pose.position = latest_pose_msg.positions[i]
            
            # Дополнение ориентацией, если она есть в данных
            if hasattr(latest_pose_msg, 'orientations') and len(latest_pose_msg.orientations) > i:
                pose.orientation = latest_pose_msg.orientations[i]
            else:
                pose.orientation = Quaternion(0, 0, 0, 1)  # Единичная ориентация по умолчанию
            
            detection_msg.pose.poses.append(pose)
    
    detection_pub.publish(detection_msg)

def pose_callback(pose_msg):
    global latest_pose_msg
    latest_pose_msg = pose_msg

if __name__ == "__main__":
    rospy.init_node('objects_to_detection_with_pose')

    # Подписываемся на топик, где публикуются сообщения типа Objects
    rospy.Subscriber('/segmentation_openseed', Objects, objects_callback)

    # Подписываемся на топик с позами
    rospy.Subscriber('/object_pose', ObjectPose, pose_callback)

    # Создаем паблишер для нового сообщения
    detection_pub = rospy.Publisher('/detection_topic', Detection, queue_size=10)

    rospy.spin()

