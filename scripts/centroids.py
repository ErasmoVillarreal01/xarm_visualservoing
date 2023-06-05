#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

class ColorTracker():
    def __init__(self):

        self.bridge = CvBridge()

        self.diff_pose_pub = rospy.Publisher('diff_pose',Pose,queue_size=10)
        self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.image_cb)

        self.result_pub = rospy.Publisher('result_frame', Image)
        # self.object_cx

    def image_cb(self, data):

        try:
            input_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        hsv_frame = cv2.cvtColor(input_frame,cv2.COLOR_BGR2HSV)

        orange_mask = cv2.inRange(hsv_frame, (165, 70, 180), (179, 255  , 255))
    
        contours,_  = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = input_frame.copy()

        diff_x, diff_y = 0, 0
        height, width,_ = result.shape
        center_x, center_y = int(width/2), int(height/2)
        
        cv2.circle(result,(center_x,center_y),7, (255,0,0), -1)
        ## TODO: Hacer este procedimiento solo con el que tenga el area mas grande
        for c in contours:
            area = cv2.contourArea(c)
            if area > 400:
                M = cv2.moments(c)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                diff_x, diff_y = cx-center_x, center_y-cy

                msg = ""
                epsilon = 50
                if(abs(diff_x)<epsilon and abs(diff_y)<epsilon):
                    msg="center"
                elif(diff_x>0 and diff_y>0):
                    msg="1"
                elif(diff_x<0 and diff_y>0):
                    msg="2"
                elif(diff_x<0 and diff_y<0):
                    msg="3"
                elif(diff_x>0 and diff_y<0):
                    msg="4"
    
                rospy.loginfo(msg)
                rospy.loginfo("{}, {}".format(diff_x,diff_y))
                rospy.loginfo("----")
                nuevoContorno = cv2.convexHull(c)
                # rospy.loginfo("{}, {}".format(cx,cy))
                cv2.putText(result, '{},{}'.format(diff_x,diff_y), (cx+10,cy),cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),1,cv2.LINE_AA) # diff x y diff y
                cv2.putText(result, msg, (center_x+10,center_y+10),cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),1,cv2.LINE_AA) # en q cuadrante esta o si esta en el centro
                cv2.putText(result, "{}".format(area), (center_x+40,center_y+40),cv2.FONT_HERSHEY_SIMPLEX, 0.75,(255,255,255),1,cv2.LINE_AA) # area
                # cv2.putText(result, '{}'.format(), (cx+10,cy-10),cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),1,cv2.LINE_AA)
                cv2.circle(result,(cx,cy),7, (0,255,0), -1) # circulo centro
                cv2.drawContours(result, [nuevoContorno], -1, (255,0,0), 3)

        difference_Pose = Pose()
        difference_Pose.position.x = diff_x
        difference_Pose.position.y = diff_y

        self.diff_pose_pub.publish(difference_Pose)
        try:
            self.result_pub.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node('position_node')
    rate = rospy.Rate(1)
    ic = ColorTracker()
    try:
        rospy.spin()
        rate.sleep()
    except KeyboardInterrupt:
        print("Shutting Down!")
    cv2.destroyAllWindows()
    

    '''
    rqt_image_view (terminal 1)
    rosrun xarm_planner centroids.py (terminal 2)
    '''