#!/usr/bin/env python3

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=False)
        
        # Create a SimpleActionClient for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

    def create_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion (z-axis rotation)
        # q = [0, 0, sin(yaw/2), cos(yaw/2)]
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return goal

    def run(self):
        # Define a list of waypoints (x, y, yaw)
        # You can modify these coordinates based on your map
        waypoints = [
            (1.0, 1.0, 0.0),
            (2.0, 0.0, 0.0),
            (0.0, 2.0, 1.57) # ~90 degrees
        ]

        for i, (x, y, yaw) in enumerate(waypoints):
            rospy.loginfo(f"Navigating to waypoint {i+1}: x={x}, y={y}, yaw={yaw}")
            
            goal = self.create_goal(x, y, yaw)
            self.client.send_goal(goal)
            
            wait = self.client.wait_for_result()
            
            if not wait:
                rospy.logerr("Action server not available!")
                break
            else:
                result = self.client.get_result()
                if result:
                    rospy.loginfo(f"Waypoint {i+1} reached successfully!")
                else:
                    rospy.loginfo(f"Failed to reach waypoint {i+1}")
                    
            # Optional: Pause between waypoints
            rospy.sleep(1)

        rospy.loginfo("All waypoints completed.")

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
