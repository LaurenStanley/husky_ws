#! /usr/bin/env python3

import rclpy
#import rospy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math
import inference

point_x = 0.0
point_y = 0.0
point_z = 0.0

class PenguinPointPublisherNode(Node):

    def __init__(self):
        super().__init__('penguin_point_publisher')
        self.publisher_ = self.create_publisher(Point, 'penguin_point', 10)
        self.goal_pose_timer_ = self.create_timer(0.1, self.publish_penguin_point)
        
        
    def publish_penguin_point(self):
    
        path2images = {'/home/lauren/TensorFlow/penguin_detection_model/images/900.jpeg'}
        com_list = inference.inference_as_raw_output(path2images)
        print(com_list)
        
        point_x = com_list[0][0]
        point_y = com_list[0][1]
        point_z = 0.0
        
        penguin_point = Point()
        penguin_point.x = point_x
        penguin_point.y = point_y
        penguin_point.z = point_z
        
        self.publisher_.publish(penguin_point)
        
    
def main(args=None):  
    rclpy.init(args=args)
    node = PenguinPointPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
