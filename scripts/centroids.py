import cv2
import numpy as np


cap = cv2.VideoCapture(0)

while(1):

    _, frame = cap.read()

    #converting to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result

    # Normal masking algorithm

    # green_filtered_image = cv2.inRange(hsv, (49, 39, 110), (98, 255, 255))
#     green_filtered_image = cv2.inRange(hsv, (70, 143, 18), (125, 255, 255))
    orange_lower = np.array([10, 90, 150], np.uint8)
    orange_upper = np.array([25, 255, 255], np.uint8)
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    cv2.imshow("mask", orange_mask)
    
    contours,_  = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = frame.copy()

    for c in contours:
        area = cv2.contourArea(c)
        if area > 80:
            M = cv2.moments(c)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            nuevoContorno = cv2.convexHull(c)
            cv2.putText(result, '{},{}'.format(cx,cy), (cx+10,cy),cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),1,cv2.LINE_AA)
            # cv2.putText(result, '{}'.format(), (cx+10,cy-10),cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),1,cv2.LINE_AA)
            cv2.circle(result,(cx,cy),7, (0,255,0), -1)
            cv2.drawContours(result, [nuevoContorno], -1, (255,0,0), 3)

    # result = cv2.bitwise_and(frame,frame,mask = green_filtered_image)

    cv2.imshow('result',result)

    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()