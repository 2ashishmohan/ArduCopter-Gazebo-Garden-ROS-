#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest
from mavros_msgs.srv import CommandTOL
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.waypoints = [(0, 0, 2), (5, 5, 2), (0, 5, 2)]  # List of waypoints (x, y, z)

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

    def navigate_to_waypoints(self):
        for waypoint in self.waypoints:
            target_pose = PoseStamped()
            target_pose.pose.position.x = waypoint[0]
            target_pose.pose.position.y = waypoint[1]
            target_pose.pose.position.z = waypoint[2]
            target_pose.pose.orientation.w = 1.0  # Default orientation (no rotation)

            self.setpoint_pub.publish(target_pose)
            rospy.sleep(5)  # Wait for 5 seconds at each waypoint

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.set_mode("GUIDED")  # Set mode to "GUIDED"
        controller.arm()  # Arm the drone
        controller.takeoff(2)  # Set desired takeoff height in meters
        controller.navigate_to_waypoints()
        controller.land()
    except rospy.ROSInterruptException:
        pass

