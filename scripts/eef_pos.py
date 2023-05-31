#!/usr/bin/env python3
import rospy
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('link_eef', 'world', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        print(trans)
        
        rate.sleep()