#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time
from copy import deepcopy
"""
Basic navigation demo to go to poses.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    inspection_route = [
        [3.0, 3.0],
        [-3.0,3.0],
        [3.0, -3.0]
    ]

    # Set our demo's initial pose
    #initial_pose = PoseStamped()
    #initial_pose.header.frame_id = 'map'
    #initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    #initial_pose.pose.position.x = 3.45
    #initial_pose.pose.position.y = 2.15
    #initial_pose.pose.orientation.z = 1.0
    #initial_pose.pose.orientation.w = 0.0
    #navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()
    while rclpy.ok():

        inspection_points = []
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        inspection_pose.pose.position.z = 1.0
        inspection_pose.pose.position.w = 0.0
        for pt in inspection_route:
            inspection_pose.pose.position.x = pt[0]
            inspection_pose.pose.position.y = pt[1]
            inspection_points.append(deepcopy(inspection_pose))
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(inspection_points)

    
        i = 0
        while not navigator.isNavComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))
               
           

    # Do something depending on the return code
        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded! returning to start')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        while not navigator.isNavComplete:
            pass

    
if __name__ == '__main__':
    main()
