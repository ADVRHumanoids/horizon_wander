#!/usr/bin/python3

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class ReferenceTrajecytoryCreator : 

    def __init__(self):
        
        # self.robot_id = rospy.get_param('~robot_id', 'wander')
        rospy.Subscriber("robotnik_base_control/odom", Odometry, self.odom_callback)
        self.executed_path_publisher = rospy.Publisher('executed_path', Path, queue_size=1)
        self.executed_path = Path()

    def odom_callback(self, data):

        transformed_pose_pub = PoseStamped()
        # transformed_pose_pub.header.frame_id = self.robot_id + "_odom"
        transformed_pose_pub.header.frame_id = "wander_odom"
        transformed_pose_pub.header.stamp = rospy.Time.now()
        transformed_pose_pub.pose.position.x = data.pose.pose.position.x
        transformed_pose_pub.pose.position.y = data.pose.pose.position.y
        transformed_pose_pub.pose.position.z =  data.pose.pose.position.z
        transformed_pose_pub.pose.orientation.x = data.pose.pose.orientation.x
        transformed_pose_pub.pose.orientation.y = data.pose.pose.orientation.y
        transformed_pose_pub.pose.orientation.z = data.pose.pose.orientation.z
        transformed_pose_pub.pose.orientation.w = data.pose.pose.orientation.w

        self.executed_path.poses.append(transformed_pose_pub)
        
        # print('past position: ', self.past_x_position)
        if  len(self.executed_path.poses) > 200 :  self.executed_path.poses = self.executed_path.poses[1:]
    
    def path_publisher(self):
        self.executed_path_publisher.publish(self.executed_path)

if __name__ == '__main__': 

    rospy.init_node("reference_trajectory_node", anonymous=True)   
    planner = ReferenceTrajecytoryCreator()
    while not rospy.is_shutdown():
        planner.path_publisher()



