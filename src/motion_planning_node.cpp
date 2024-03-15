#include <iostream>
#include <ros/ros.h>
#include <memory>
// MoveitCpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


// To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
// rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

class Commander
{

};


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "motion_planner_node");
    ros::NodeHandle nh();
    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string PLANNING_GROUP = "arm";
    const std::string ROBOT_DESCRIPTION = "/my_gen3_lite/robot_description";

    moveit::planning_interface::MoveGroupInterface::Options options(PLANNING_GROUP, ROBOT_DESCRIPTION);

    // Construct a client for the MoveGroup action for a particular group.
    moveit::planning_interface::MoveGroupInterface move_group_interface(options);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group =
          move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    
    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


    // Home pose of manipulator
    moveit::core::RobotStatePtr current_state_ptr = move_group_interface.getCurrentState(); 
    std::vector<double> joint_group_home_positions;
    current_state_ptr->copyJointGroupPositions(PLANNING_GROUP, joint_group_home_positions);
    

    // Plan object use to plan and execute motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    // *********************************************************************************************
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;

    move_group_interface.setPoseTarget(target_pose1);
    
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Planning time: %f", my_plan.planning_time_);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal): %s", success ? "GOOD" : "FAILED");
    move_group_interface.execute(my_plan);
    // *********************************************************************************************


    // *********************************************************************************************
    move_group_interface.setJointValueTarget(joint_group_home_positions);

    success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Planning time: %f", my_plan.planning_time_);
    ROS_INFO_NAMED("tutorial", "Visualizing plan home: %s", success ? "GOOD" : "FAILED");
    move_group_interface.execute(my_plan);

    // *********************************************************************************************






    /*
    //std::cout << move_group_interface.getPlannerId() << std::endl;
    const std::vector<std::string>& joint_names = move_group_interface.getJointNames();
    const std::string& planning_pipeline = move_group_interface.getPlanningPipelineId();
    const std::string& planner_id = move_group_interface.getPlannerId();
    
    ROS_INFO("JOINT: %s", joint_names[0].c_str());
    ROS_INFO("PLANNING_PIPELINE: %s", planning_pipeline.c_str());
    ROS_INFO("PLANNER_IDl %s", planner_id.c_str());
    */


    // const moveit::core::JointModelGroup* joint_model_group_ptr = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    // moveit::planning_scene::PlanningSceneInterface planning_scene_interface;

    return 0;

}