#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


coord = Pose()

def callback(data):   
    global coord
    coord.position.x = data.pose.position.x
    coord.position.y = data.pose.position.y
    coord.position.z = -data.pose.position.z

    coord.orientation.x = 1
    rospy.loginfo(coord)




if __name__ == '__main__':
    rospy.init_node('listener')
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, callback)
    pub_coord = rospy.Publisher("diff_pose", Pose, queue_size=10)
    while not rospy.is_shutdown():


        pub_coord.publish(coord)

        rate.sleep()