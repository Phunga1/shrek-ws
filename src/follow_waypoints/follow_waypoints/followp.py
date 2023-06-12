#! /usr/bin/env python3

import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from .basicfol import BasicNavigator
import rclpy
import sys
from geometry_msgs.msg import PoseStamped

from rclpy.duration import Duration
import time

def main(argv=sys.argv[1:]):
    rclpy.init()
    AMOUNTOFPOINTS = 3
    navigator = BasicNavigator()
    xlist = [0.5, 2.4, 1.9]
    ylist = [-1.6, -1.2, 0.0]
    # Set our demo's initial pose
    initial_pose = Pose()
    initial_pose.position.x = 0.0 #1.35
    initial_pose.position.y = 0.0 #-0.125
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    
   
    while True:
        points = 0
        while points < AMOUNTOFPOINTS: #doe dit tot alle punten doorlopen zijn
            goal_pose = PoseStamped() #Maak de goal message aan en zet de juiste waarden erin.
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = xlist[points]
            goal_pose.pose.position.y = ylist[points]
            goal_pose.pose.orientation.w = 1.0
            navigator.goToPose(goal_pose) #stuur het punt
            while not navigator.isNavComplete(): #wachten tot het punt bereikt is
                print(f"Navigating to: ({xlist[points]},{ylist[points]})")
                
            
            points += 1 # ga naar het volgende punt
        # Do something depending on the return code
        result = navigator.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            print('Goal succeeded!')
        elif result == GoalStatus.STATUS_CANCELED:
            print('Goal was canceled!')
        elif result == GoalStatus.STATUS_ABORTED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        
    exit(0)