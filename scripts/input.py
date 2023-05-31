#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose

pose_enviar = Pose()

pose_enviar.position.x = 0.1
pose_enviar.position.y = 0.1
pose_enviar.position.z = 0.2

def talker():
    pub = rospy.Publisher('pose', Pose, queue_size=10)
    rospy.init_node('pose_input')
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        
        #pub.publish(pose_enviar)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass