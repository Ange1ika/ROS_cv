import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import ros_numpy

def point_cloud_callback(data):
    # Преобразуем сообщение PointCloud2 в numpy массив
    pc = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    np.save('/resources/point_cloud.npy', pc)

def listener():
    rospy.init_node('point_cloud_listener', anonymous=True)
    rospy.Subscriber('/object_point_cloud_extraction_node.py', PointCloud2, point_cloud_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

