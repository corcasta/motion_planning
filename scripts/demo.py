#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose

# The 3 main classes that we need: RobotCommander PlanningSceneInterface MoveGroupCommander
# RobotCommander: provides information of robots kinematics and joint states

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3_lite

class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories"""
    def __init__(self):
        # Initialize the node
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')
        
        self.arm_joint_home_positions = []
        self.gripper_joint_home_positions = []
        
        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            print(f"debug: {rospy.get_namespace()}")
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander(rospy.get_namespace()+"robot_description")
            print("debug")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True


    def set_planner(self, planner_id):
        if planner_id:
            self.arm_group.set_planner_id(planner_id)
            self.gripper_group.set_planner_id(planner_id)


    def reach_pose(self, group, pose):
        """
        group: type string, represening a move group commander (arm, gripper)
        pose:  type geometry_msgs.msg.Pose(), representing target pose
        """
        if group == "arm":
            move_group = self.arm_group
        elif group == "gripper":
            move_group = self.gripper_group
        else:
            print("Error: Invalid group name")
            return False

        move_group.set_pose_target(pose)
        (success_flag, trajectory, planning_time, error_code)  = move_group.plan()
        
        if success_flag:
            return move_group.execute(trajectory)
        else:
            print("Can NOT find a plan for that pose")
            return False

    def set_home_joint_angles(self):
        # Get the current joint positions
        self.arm_joint_home_positions = self.arm_group.get_current_joint_values()
        self.gripper_joint_home_positions = self.gripper_group.get_current_joint_values()
    

    def reach_home_joint_angles(self):
        success_arm = True
        success_gripper = True
        
        success_arm &= self.reach_joint_angles("arm", self.arm_joint_home_positions, tolerance=0.01)
        success_gripper &= self.reach_joint_angles("gripper", self.gripper_joint_home_positions, tolerance=0.01)

        return success_arm & success_gripper
    

    def reach_joint_angles(self, group, joint_positions, tolerance):
        """
        group: type string, represening a move group commander (arm, gripper)
        joint_positions: type list of floats, size equal to the number of joints in the group
        tolerance: type float 
        """
        if group == "arm":
            move_group = self.arm_group
        elif group == "gripper":
            move_group = self.gripper_group
        else:
            print("Error: Invalid group name")
            return False
        
        success = True

        # Get the current joint positions
        current_joint_positions = move_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions BEFORE movement :")
        for p in current_joint_positions: rospy.loginfo(p)

        # Set the goal joint tolerance
        move_group.set_goal_joint_tolerance(tolerance)
        move_group.set_joint_value_target(joint_positions)

        # Plan and execute in one command
        success &= move_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = move_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions AFTER movement :")
        for p in new_joint_positions: rospy.loginfo(p)
        return success


    def reach_named_position(self, target):
        arm_group = self.arm_group
        
        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(trajectory_message, wait=True)


    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose


    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group
        
        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)


    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
        
        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False 


def main():
    example = ExampleMoveItTrajectories()
    
    success = example.is_init_success

    example.set_home_joint_angles()

    
    if success:
        demo_pose = Pose()
        demo_pose.position.x = 0.28
        demo_pose.position.y = -0.2
        demo_pose.position.z = 0.5
        demo_pose.orientation.w = 1.0
        
        success &= example.reach_pose("arm", demo_pose)
        print(f"Test 1: {success}")

    if success:
        success &= example.reach_home_joint_angles()
        print(f"Test 2: {success}")
    
    if success:
        joint_positions = example.arm_group.get_current_joint_values()
        if example.degrees_of_freedom == 7:
            joint_positions[0] = pi/2
            joint_positions[1] = 0
            joint_positions[2] = pi/4
            joint_positions[3] = -pi/4
            joint_positions[4] = 0
            joint_positions[5] = pi/2
            joint_positions[6] = 0.2
        elif example.degrees_of_freedom == 6:
            joint_positions[0] = 0
            joint_positions[1] = 0
            joint_positions[2] = pi/2
            joint_positions[3] = pi/4
            joint_positions[4] = 0
            joint_positions[5] = pi/2
        success &= example.reach_joint_angles("arm", joint_positions, 0.01)
        print(f"Test 3: {success}")

    """
    # For testing purposes
    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    if success:
        rospy.loginfo("Reaching Named Target Vertical...")
        success &= example.reach_named_position("vertical")
        print(success)
  
    if success:
        rospy.loginfo("Reaching Named Target Home...")
        success &= example.reach_named_position("home")
        print(success)

    if success:
        rospy.loginfo("Reaching Joint Angles...")  
        success &= example.reach_joint_angles(tolerance=0.01) #rad
        print(success)

    if success:
        rospy.loginfo("Reaching Cartesian Pose...")
        actual_pose = example.get_cartesian_pose()
        actual_pose.position.z -= 0.2
        success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
        print(success)
    
    if example.degrees_of_freedom == 7 and success:
        rospy.loginfo("Reach Cartesian Pose with constraints...")
        # Get actual pose
        actual_pose = example.get_cartesian_pose()
        actual_pose.position.y -= 0.3
    
        # Orientation constraint (we want the end effector to stay the same orientation)
        constraints = moveit_msgs.msg.Constraints()
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.orientation = actual_pose.orientation
        constraints.orientation_constraints.append(orientation_constraint)

        # Send the goal
        success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

    if example.is_gripper_present and success:
        rospy.loginfo("Opening the gripper...")
        success &= example.reach_gripper_position(0)
        print (success)

        rospy.loginfo("Closing the gripper 50%...")
        success &= example.reach_gripper_position(0.5)
        print (success)

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")

    """

if __name__ == "__main__":
    main()