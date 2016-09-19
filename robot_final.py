#!/usr/bin/env python

import rospy
import cv2
import numpy

from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY
from cv2 import blur, Canny
from numpy import mean
from math import radians

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from cv_bridge import CvBridge

class turtlebot_aggression:

    def __init__(self):
        
        namedWindow("Image Window - Colour", 1)
        namedWindow("Image window - Sliced", 1)
        namedWindow("Left Half", 1)
        namedWindow("Right Half", 1)
        self.bridge = CvBridge()
        startWindowThread()
        
        # Subscribe to turtlebot RGB camera
        # Robot Code
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.block_detection)
        # Simulator Code
        #self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",Image,self.block_detection)
        
        # Subscribe to turtlebot LaserScan
        # Robot Code
        self.laser_sub = rospy.Subscriber("/scan",LaserScan,self.laserscan_avoidance)
        # Simulator Code
        #self.laser_sub = rospy.Subscriber("/turtlebot_1/scan",LaserScan,self.laserscan_avoidance)
        
        # Publish to twist on turtlebot
        # Robot Code
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)     
        # Simulator Code
        #self.pub = rospy.Publisher("/turtlebot_1/cmd_vel", Twist, queue_size=1)
        
        self.green_intensity = 0
        
        self.twist_drive = Twist()
        self.twist_drive.linear.x = 0.1
        
        
    def laserscan_avoidance(self, data):
        
        # Twist turn init      
        twist_turn = Twist()
                
        # Get distance readings from laserscan and print to console.
        min_range = data.range_max
        for v in data.ranges[150:490]:
            if v < min_range:
                min_range = v
        #print ("Min Range = "), min_range
        
        # If min ranage falls below 0.9 implement twist_turn and publish to cmd_vel.
        # Turns robot away from obstacle.
        if min_range < 1.5 and self.green_intensity == 0:
            twist_turn.linear.x = 0
            twist_turn.angular.z = radians(120)
            print("Obstacle detected, Turning...")
            self.pub.publish(twist_turn)
        # If range is over threshold then publish twist_drive to cmd_vel.
        # Drives robot forward.
        if min_range > 0.9:
            self.pub.publish(self.twist_drive)
            
        return min_range
    
    def block_detection(self, data):
        
        rospy.Rate(2)
        
        # Gets cv image from camera
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        cv_image =  cv2.medianBlur(cv_image, 17)            
        
        left_half_image = cv_image[0:480,0:320]
        right_half_image = cv_image[0:480,320:640]
        
        # Sets green upper and lower limits         
        green_lower_limit = numpy.array([40,100,50],numpy.uint8)
        green_upper_limit = numpy.array([90,255,255],numpy.uint8)
        
        # Converts BGR to HSV image
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        left_half_image_hsv = cv2.cvtColor(left_half_image,cv2.COLOR_BGR2HSV)
        right_half_image_hsv = cv2.cvtColor(right_half_image,cv2.COLOR_BGR2HSV)
        
        # Thresholds the images using upper and lower limits
        image_threshed = cv2.inRange(hsv_image, green_lower_limit, green_upper_limit)
        left_image_threshed = cv2.inRange(left_half_image_hsv, green_lower_limit, green_upper_limit)
        right_image_threshed = cv2.inRange(right_half_image_hsv, green_lower_limit, green_upper_limit)
        
        result_image = cv2.bitwise_and(cv_image,cv_image, mask= image_threshed)
        left_result_image = cv2.bitwise_and(left_half_image,left_half_image, mask= left_image_threshed)
        right_result_image = cv2.bitwise_and(right_half_image,right_half_image, mask= right_image_threshed)

        # Calculates mean intensity for left, right and whole image
        mean_intensity = numpy.mean(result_image)
        #print("Whole Image - Mean intensity = "), mean_intensity
        left_mean_intensity = numpy.mean(left_result_image)
        print("Left Image - Mean intensity = "), left_mean_intensity
        right_mean_intensity = numpy.mean(right_result_image)
        print("Right Image - Mean intensity = "), right_mean_intensity
        
        self.green_intensity = mean_intensity
        
        # Shows colour image in image window
        cv2.imshow("Image Window - Colour", cv_image)
        cv2.imshow("Left Half", left_result_image)
        cv2.imshow("Right Half", right_result_image)
        
        # Shows theshed image in image window
        cv2.imshow("Image window - Sliced", result_image)
        
        # Use mean intensity as speed, causes robot to speed up the closer it gets to detected block
        if 1.5 > mean_intensity > 0:       
            speed = mean_intensity
        else:
            # Caps speed at 0.4
           speed = 0.4
           
       
        if mean_intensity > 0.2:          
            print ("Green box detected... passing over to aggression...")
            # If left side of image has an intensity > 0, send wheel speeds to enable turning to the right           
            if left_mean_intensity > 0:
                twist_drive = self.aggression(speed/2 , speed)
                self.pub.publish(twist_drive)  
            # If right side of image has an intensity > 0, send wheel speeds to enable turning to the left 
            if right_mean_intensity > 0:
                twist_drive = self.aggression(speed, speed/2)
                self.pub.publish(twist_drive)   
        
        print ("Speed = "), speed

            
    def aggression(self, left_wheel_power, right_wheel_power):
        # Aggression actions go here!!!
        print("Recieved... Getting Aggressive...")
        
        if right_wheel_power == None:
            right_wheel_power = left_wheel_power

        left_wheel = [0.1*left_wheel_power, -1.0*left_wheel_power]
        right_wheel = [0.1*right_wheel_power, 1.0*right_wheel_power]

        # Create twist to publish to robot, twsit message is published to cmd_vel
        twist = Twist()
        # Sets the linear speed to the total of both wheels
        twist.linear.x = left_wheel[0] + right_wheel[0]
        
        twist.angular.z = left_wheel[1] + right_wheel[1]
        
        return twist
            
if __name__ == "__main__":
    
    rospy.init_node("turtlebot_aggression")
            
    turtlebot_aggression = turtlebot_aggression();    

    rospy.spin()

    destroyAllWindows()