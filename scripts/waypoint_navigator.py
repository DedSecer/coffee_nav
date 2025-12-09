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
            {'x': 18.904834747314453, 'y': 50.74214553833008, 'z': 0.566377833465898, 'w': 0.8241457090578556},
            {'x': 57.68058776855469, 'y':139.3214874267578, 'z': 0.5574984821314145, 'w': 0.8301779582843481},
            {'x': 47.24309158325195, 'y': 160.61888122558594, 'z': 0.5798534086898137, 'w': 0.8147208260691535},
            {'x': 52.50185775756836, 'y': 191.88926696777344, 'z': 0.9125285312088497, 'w': 0.4090130556960491},
            {'x': 55.07625198364258, 'y': 200.88497924804688, 'z': 0.40340233810135256, 'w': 0.9150227066113508},
            {'x': 59.85647964477539, 'y': 203.67294311523438, 'z': 0.5693342714159336, 'w': 0.8221061290315795},
            {'x': 79.39704132080078, 'y': 249.03604125976562, 'z': 0.9766663864712506, 'w': 0.21476212314369997},
            {'x': 67.77154541015625, 'y': 252.67991638183594, 'z': 0.9942378532461007, 'w': 0.10719650727791952}
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
