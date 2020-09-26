#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import radians
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import tf2_ros
import tf2_geometry_msgs  
import csv

from geometry_msgs.msg import Pose
from pathlib import Path


from moveit_msgs.msg import  Constraints, OrientationConstraint

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import argparse
global_plan = 0

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator" 
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planner_id("RRTkConfigDefault")

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    eefpose = move_group.get_current_pose()
    euler_ang = euler_from_quaternion([eefpose.pose.orientation.x, eefpose.pose.orientation.y, eefpose.pose.orientation.z, eefpose.pose.orientation.w])
    print("roll, pitch, yaw ", euler_ang[0], euler_ang[1],euler_ang[2])
    print("all config",eefpose)

    self.tf_buffer = tf2_ros.Buffer() 
    self.listener = tf2_ros.TransformListener(self.tf_buffer)      

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    # self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def do_transform(self,point_trans, from_frame, to_frame):

    tf_buffer = self.tf_buffer 
    listener = self.listener  
  
    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = point_trans
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()
    trans = tf_buffer.lookup_transform(to_frame ,from_frame, rospy.Time())

    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, trans)
    return pose_transformed.pose

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return False
      

  def go_to_joint_state(self,j1,j2,j3,j4,j5,j6):
    move_group = self.move_group
    joint0 = radians(j1)
    joint1 = radians(j2)
    joint2 = radians(j3)
    joint3 = radians(j4)
    joint4 = radians(j5)
    joint5 = radians(j6)

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = joint0
    joint_goal[1] = joint1
    joint_goal[2] = joint2
    joint_goal[3] = joint3
    joint_goal[4] = joint4
    joint_goal[5] = joint5


    move_group.go(joint_goal, wait=True)
    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def plan_cartesian_path(self, xcor,ycor,zcor,pose_goal_base, vel_scale, scale=1 ):

    move_group = self.move_group

    waypoints = []
    pose_goal = geometry_msgs.msg.Pose()

    print("length of x coordinate ", len(xcor))
    print("base pose = ", pose_goal_base.position)
    
    eef_pose = Pose()

    for i in range(0,len(xcor)):

      eef_pose.position.x = float(xcor[i])/1000.0 
      eef_pose.position.y = float(ycor[i])/1000.0 
      eef_pose.position.z = float(zcor[i])/1000.0 

      eef_in_baseframe = self.do_transform(eef_pose,'ee_link','world')
 
      pose_goal.position.x = eef_in_baseframe.position.x
      pose_goal.position.y = eef_in_baseframe.position.y
      pose_goal.position.z = eef_in_baseframe.position.z

      pose_goal.orientation.x = pose_goal_base.orientation.x
      pose_goal.orientation.y = pose_goal_base.orientation.y
      pose_goal.orientation.z = pose_goal_base.orientation.z
      pose_goal.orientation.w = pose_goal_base.orientation.w

      waypoints.append(copy.deepcopy(pose_goal))


    orient = OrientationConstraint()
    orient.link_name = self.eef_link
    orient.header.frame_id = self.planning_frame

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.001,        # eef_step
                                       0.0)         # jump_threshold
    # move_group = self.move_group
    # move_group.execute(plan, wait=True)

    ref_state = self.robot.get_current_state()
    
    replanned = move_group.retime_trajectory(ref_state,plan,velocity_scaling_factor=vel_scale,acceleration_scaling_factor=vel_scale)

    return replanned, fraction




  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan, wait=True)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return False




def main():
  print "starting"
  velocity_scaling = rospy.get_param("/velocity_scaling")
  print("vel scaling is ", velocity_scaling)

  fle_name = rospy.get_param("/file_name")

  home = str(Path.home())    
  path = os.path.join(home,file_name)


#  path = rospy.get_param("/file_loc")

  xcordinates = []
  ycoordinates = []
  zcoordinates = []
  i = 0
  last_row = [0,0,0]
  with open(path) as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
      if(last_row[0]==row[0]):
        last_row = row
        continue

      xcordinates.append(row[2])
      ycoordinates.append(row[1])
      zcoordinates.append(row[0])
      last_row = row
        

  del xcordinates[0]
  del ycoordinates[0]
  del zcoordinates[0]  

  tutorial = MoveGroupPythonIntefaceTutorial()

  print "============ Press `Enter` to go to forming position ..."
  raw_input()

  tutorial.go_to_joint_state(-135,-96,103,-30,0,0)
  tutorial.go_to_joint_state(-135,-96,103,-30,-93,0)
  tutorial.go_to_joint_state(-145,-96,104,30,-93,0)
  tutorial.go_to_joint_state(-141,-58,68,-40,-93,0)
  tutorial.go_to_joint_state(-141,-52,69,-113,-93,0)
  tutorial.go_to_joint_state(-140,-51,69,-113,-3,0)

  print "============ Press `Enter` start forming ..."
  raw_input()

  pose_goal_base = tutorial.move_group.get_current_pose().pose
  cartesian_plan, fraction = tutorial.plan_cartesian_path(xcordinates,ycoordinates,zcoordinates,pose_goal_base,velocity_scaling)
  tutorial.execute_plan(cartesian_plan)

  print "============ Press `Enter` to go back to home position..."
  raw_input()

  tutorial.go_to_joint_state(-141,-52,69,-113,-93,0)
  tutorial.go_to_joint_state(-141,-58,68,-40,-93,0)
  tutorial.go_to_joint_state(-145,-96,104,30,-93,0)
  tutorial.go_to_joint_state(-135,-96,103,-30,-93,0)
  tutorial.go_to_joint_state(-135,-96,103,-30,0,0)
  tutorial.go_to_joint_state(-135,-180,150,-30,0.1,0)

  print "Done ..."

if __name__ == '__main__':
  while True:
    try:
      main()
    except KeyboardInterrupt:
        break
