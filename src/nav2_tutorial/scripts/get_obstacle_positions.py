#! /usr/bin/env python3

import rclpy
#import rospy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

list_of_points = []
x = 0.0
y = 0.0
odometry_pose_x = 0.0
odometry_pose_y = 0.0

class ObstaclePublisherNode(Node):

    def __init__(self):
        super().__init__('target_obstacle_publisher')
        self.subscriber = self.create_subscription(MarkerArray, 'visualization_marker_array', self.marker_array_callback, 10)
        self.subscriber2 = self.create_subscription(Odometry, 'odometry/filtered', self.odometry_callback, 10)
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.goal_pose_timer_ = self.create_timer(0.1, self.publish_goal_pose)
        
        
    def publish_goal_pose(self):     
        global x
        global y
        if len(list_of_points) > 0:
        	x = list_of_points[0][0] + odometry_pose_x
        	y = list_of_points[0][1] + odometry_pose_y
        #print(list_of_points[0][0], odometry_pose_x, x)
        #print(list_of_points[0][1], odometry_pose_y, y)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'odom'
        goal_pose.header.stamp.sec = 0
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        #speed_limit.speed_limit = 1.0
        self.publisher_.publish(goal_pose)
        
    def marker_array_callback(self, msg):
    	global list_of_points
    	list_of_points = []
    	for i in range(len(msg.markers)):
    	    point = []
    	    point.append(msg.markers[i].pose.position.x)
    	    point.append(msg.markers[i].pose.position.y)
    	    point.append(msg.markers[i].pose.position.z)
    	    list_of_points.append(point)
    	#print(list_of_points[0][0])

    def odometry_callback(self, msg):
        global odometry_pose_x
        global odometry_pose_y
        odometry_pose_x = msg.pose.pose.position.x
        odometry_pose_y = msg.pose.pose.position.y
  	
 
def main(args=None):  
    rclpy.init(args=args)
    node = ObstaclePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
