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

    def create_goal(self, x, y, yaw=None, z=None, w=None):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        
        if yaw is not None:
            # Convert yaw to quaternion (z-axis rotation)
            goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        elif z is not None and w is not None:
            # Use provided quaternion components
            goal.target_pose.pose.orientation.z = z
            goal.target_pose.pose.orientation.w = w
        else:
            # Default to no rotation (identity quaternion)
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0
        
        return goal

    def run(self):
        # Define a list of waypoints. Each waypoint can specify 'yaw' or 'z' and 'w' for orientation.
        waypoints = [
            {'x': 19.174346923828125, 'y': 52.53007507324219, 'z': 0.5130100344772857, 'w': 0.858382609636061},
            {'x': 29.486310958862305, 'y': 76.05609893798828, 'z': 0.5304967450945943, 'w': 0.8476869725577013},
            {'x': 52.20865249633789, 'y': 126.1520004272461, 'z': 0.5135103783467525, 'w': 0.8580833825043899},
            {'x': 68.62261962890625, 'y': 191.34918212890625, 'z': 0.5672666283999825, 'w': 0.8235341961957112},
            {'x': 79.8429183959961, 'y': 248.24130249023438, 'z': 0.5199162712638233, 'w': 0.854217226983349}
        ]

        for i, wp in enumerate(waypoints):
            x = wp['x']
            y = wp['y']
            yaw = wp.get('yaw')
            z = wp.get('z')
            w = wp.get('w')
            
            rospy.loginfo(f"Navigating to waypoint {i+1}: x={x}, y={y}")
            
            goal = self.create_goal(x, y, yaw=yaw, z=z, w=w)
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
