#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

import tf2_geometry_msgs  

from geometry_msgs.msg import Pose

eef_pose = Pose()

eef_pose.position.x = 0.1
eef_pose.position.y = 0.2
eef_pose.position.z = 0.3

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)
            
    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = eef_pose
    pose_stamped.header.frame_id = 'ee_link'
    pose_stamped.header.stamp = rospy.Time.now()
    
    while not rospy.is_shutdown():
        try:
            # print("in try")
            # print("yes")
            # output_pose_stamped = tf_buffer.transform(pose_stamped, 'world')

#            print("yo")
            trans = tf_buffer.lookup_transform('world', 'ee_link', rospy.Time())
            
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, trans)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()

            continue
        
        print(trans)
        # msg = geometry_msgs.msg.Twist()

        # msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        # msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        # turtle_vel.publish(msg)

        rate.sleep()
