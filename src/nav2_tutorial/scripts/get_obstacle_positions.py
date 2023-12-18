#! /usr/bin/env python3

# Convert pointcloud obstacle readout to list of penguins

import rclpy
#import rospy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from nav2_tutorial.msg import Penguin
from nav2_tutorial.msg import PenguinList
import math

list_of_points = []
penguin_list = []
penguin_label_count = 0

x = 0.0
y = 0.0
odometry_pose_x = 0.0
odometry_pose_y = 0.0

class ObstaclePublisherNode(Node):

    def __init__(self):
        super().__init__('target_obstacle_publisher')
        self.subscriber = self.create_subscription(PenguinList, 'penguin_list',self.penguin_list_callback,10)
        self.subscriber1 = self.create_subscription(MarkerArray, 'visualization_marker_array', self.marker_array_callback, 10)
        self.subscriber2 = self.create_subscription(Odometry, 'odometry/filtered', self.odometry_callback, 10)
        
        self.publisher_ = self.create_publisher(PenguinList, 'penguin_list', 10)
        #self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        #self.goal_pose_timer_ = self.create_timer(0.1, self.publish_goal_pose)
        self.penguin_timer_ = self.create_timer(0.1, self.publish_penguins)
        

    def publish_penguins(self):     
        global penguin_list
        global list_of_points
        global penguin_label_count
        final_penguin_list = PenguinList()
        sorted_penguin_list = []
        already_exists = False
        for i in range(len(list_of_points)):
            for j in range(len(penguin_list)):
                dist = (penguin_list[j].point.x - list_of_points[i][0])**2 + (penguin_list[j].point.y - list_of_points[i][1])**2
                if dist < 5.0:
                    print('Penguin Exists')
                    sorted_penguin_list.append(penguin_list[j])
                    already_exists = True
            if not already_exists:
                new_penguin = Penguin()
                new_penguin.point.x = list_of_points[i][0]
                new_penguin.point.y = list_of_points[i][1]
                new_penguin.point.z = 0.0
                new_penguin.label = str(penguin_label_count)
                penguin_label_count += 1
                sorted_penguin_list.append(new_penguin)
        final_penguin_list.penguins = sorted_penguin_list    
        self.publisher_.publish(final_penguin_list)
    
    def penguin_list_callback(self, msg):
        global penguin_list
        penguin_list = []
        for i in range(len(msg.penguins)):
            #print(msg.penguins[i])
            penguin_list.append(msg.penguins[i])

    def marker_array_callback(self, msg):
        global list_of_points
        list_of_points = []
        for i in range(len(msg.markers)):
            point = []
            point.append(msg.markers[i].pose.position.x)
            point.append(msg.markers[i].pose.position.y)
            point.append(msg.markers[i].pose.position.z)
            list_of_points.append(point)
            #print(point)
    	
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
