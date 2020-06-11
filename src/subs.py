#! /usr/bin/env python
"""
Project Title: Line Follower Turtlebot

Tech used :
- OpenCV
- Robot Operating System (ROS)

Aim : Navigate Turtlebot to follow Line based on computer vision processing using OpenCV

"""


import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist

class line_follower_turtlebot():
    def __init__(self):
        self.node_name = "line_follower_turtlebot"
        
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)

        # Determine size of window size and name
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)
        
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the Camera Image Raw
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)
        
        # Publish to Robot Velocity
        self.pub=rospy.Publisher('/cmd_vel_mux/input/teleop',Twist, queue_size=1)
        self.rate=rospy.Rate(10)

        #Login Info 
        rospy.loginfo("Bela Ciao")
        rospy.wait_for_message("/camera/rgb/image_raw", Image)
        rospy.loginfo("Line Following Turtlebot")

    def image_callback(self, data):

        frame = self.convert_image(data)
                
        # Process the image to detect and track objects or features
        processed_image = self.process_image(frame)
        
        #self.drive_bot()       
        # Display the image.
        cv2.imshow(self.node_name, processed_image)
        
        # Process any keyboard commands
        self.keystroke = cv2.waitKey(5)
        if self.keystroke != -1:
            cc = chr(self.keystroke & 255).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

    def convert_image(self, ros_image):
        
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")       
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e
    
    def process_image(self, frame):
        e1 = cv2.getTickCount()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #Red Color Representation
        low_red = np.array([0, 50, 50])
        high_red = np.array([10, 255, 255])
        #Masking the detected Shape
        red_mask = cv2.inRange(hsv_frame, low_red, high_red)
        red = cv2.bitwise_and(frame, frame, mask=red_mask)

        #Find centroid of Line Shape
        M =cv2.moments(red_mask)
        
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        

        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
        cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        e2 = cv2.getTickCount()
        t=(e2 - e1)/cv2.getTickFrequency()
        move_pos=Twist()

        #Navigate Turtlebot moves based on CentroidX Value
        centroidx=cX
        if centroidx < 110:                         #Turn Left
            move_pos.linear.x=0.3
            move_pos.angular.z=1.0
        elif centroidx >= 110 and centroidx <400:   #Move Forward
            move_pos.linear.x=1.0
            move_pos.angular.z=0.2
        elif centroidx >=400:                       #Turn Right
            move_pos.linear.x=0.2
            move_pos.angular.z=-1.0
        else:
            print("error")
        self.pub.publish(move_pos)
        # print ("Centroid X point=",cX)
        # print ("Centroid Y point=",cY)
        print(t,"seconds")
        return frame

    def cleanup(self):
        print "Done. Bela Ciao"
        cv2.destroyAllWindows()   

    
    
def main(args):       
    try:
        line_follower_turtlebot()
        rospy.spin()
    except KeyboardInterrupt:
        print "Done. Bela Ciao"
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)