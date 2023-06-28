#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest
from mavros_msgs.srv import CommandTOL
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from mavros_msgs.msg import CameraImageCaptured
from cv_bridge import CvBridge
import cv2

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.current_pose = PoseStamped()
        self.current_state = State()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mavros/camera/image_captured', CameraImageCaptured, self.image_callback)
	#rospy.Subscriber('/mavros/camera/image_captured', ImageCaptured, self.image_captured_callback)
        #rospy.Subscriber('/mavros/state', State, self.state_callback)
        #rospy.sleep(1)  # Wait for subscribers to initialize

        #self.waypoints = [(0, 0, 2), (5, 5, 2), (0, 5, 2)]  # List of waypoints (x, y, z)
	
    #def state_callback(self, msg):
        #self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg
        
    def image_callback(self, image):
    	self.cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    	cv2.imshow("Image", self.cv_image)
    	cv2.waitKey(1)
	#cv2.imshow(self.cv_image)
	#cv.waitKey(1)
    	
    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            self.mode_service(custom_mode=mode)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to set mode: %s" % e)

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.arm_service(True)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to arm: %s" % e)

    def takeoff(self, height):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            self.takeoff_service(altitude=height)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to takeoff: %s" % e)
            
    def land(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            self.land_service(altitude=0)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to takeoff: %s" % e)
            

    def navigate(self):
        for waypoint in range(1, 10):
            rospy.loginfo("Waypoint %s",  waypoint)
            x_position = self.current_pose.pose.position.x + 5
            y_position = self.current_pose.pose.position.y
            z_position = self.current_pose.pose.position.z

            target_pose = PoseStamped()
            target_pose.pose.position.x = x_position
            target_pose.pose.position.y = y_position
            target_pose.pose.position.z = z_position
            target_pose.pose.orientation.w = 1.0  # Default orientation (no rotation)

            self.setpoint_pub.publish(target_pose)
            rospy.sleep(5)  # Wait for 5 seconds at each waypoint

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.set_mode("GUIDED")  # Set mode to "GUIDED"
        controller.arm()  # Arm the drone
        controller.takeoff(5)  # Set desired takeoff height in meters
        controller.navigate()
        controller.land()
    except rospy.ROSInterruptException:
        pass

