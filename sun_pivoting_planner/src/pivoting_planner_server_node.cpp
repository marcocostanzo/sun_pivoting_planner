#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sun_pivoting_planner_msgs/PivotingPlanAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.h"

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include "std_msgs/Float64MultiArray.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sun_pivoting_planner/utility.h"
#include "sun_pivoting_planner/exceptions.h"

namespace sun
{

class PivotingPlannerActionServer
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sun_pivoting_planner_msgs::PivotingPlanAction> as_; // NodeHandle instance must be created before this line.

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_transform_listener_;

  ros::Publisher pub_simulated_joint_;

  Joint_Conf_Constraint joint_conf_constraint_;

  double plan_time = 5.0;
  int num_planning_attempts = 4;
std::string plannerID = "PRMstar";

public:

  PivotingPlannerActionServer(const std::string& name, const ros::NodeHandle& nh = ros::NodeHandle()) :
	nh_(nh),
    as_(nh_, name, boost::bind(&PivotingPlannerActionServer::executePlanning, this, _1), false),
	tf2_transform_listener_(buffer_)
  {
    as_.start();
	pub_simulated_joint_ = nh_.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states",1);

	//joint_conf_constraint_.joint_names.push_back("iiwa_joint_1");
	joint_conf_constraint_.joint_names.push_back("iiwa_joint_2");
	joint_conf_constraint_.joint_names.push_back("iiwa_joint_3");
	joint_conf_constraint_.joint_names.push_back("iiwa_joint_4");
	joint_conf_constraint_.move_range.push_back((2.0*120.0)/4.0 *M_PI/180.0);	
	joint_conf_constraint_.move_range.push_back((2.0*170.0)/4.0 *M_PI/180.0);	
	joint_conf_constraint_.move_range.push_back((2.0*120.0)/4.0 *M_PI/180.0);	
	// joint_conf_constraint_.move_range.push_back((2.0*120.0)/1.0 *M_PI/180.0);	
	// joint_conf_constraint_.move_range.push_back((2.0*170.0)/1.0 *M_PI/180.0);	
	// joint_conf_constraint_.move_range.push_back((2.0*120.0)/1.0 *M_PI/180.0);
  }

  ~PivotingPlannerActionServer(void)
  {
  } 

  void set_simulation_configuration(
	  const std::vector<double>& joint_position, 
	  const std::vector<std::string>& joint_names
	)
  {
	sensor_msgs::JointState msg;
	msg.name = joint_names;
	msg.position = joint_position;
	msg.velocity.resize(joint_names.size());
	msg.effort.resize(joint_names.size());
	pub_simulated_joint_.publish(msg);
	ros::Duration(0.5).sleep();
  }

  bool plan(
	  moveit::planning_interface::MoveGroupInterface& move_group,
	  const std::string& end_effector_frame_id,
	  const geometry_msgs::PoseStamped& target_pose,
	  const moveit_msgs::Constraints& path_constraints,
	  trajectory_msgs::JointTrajectory& planned_traj,
	  bool use_configuration_constraints = true
	  )
  {

	  if(target_pose.header.frame_id == "")
	  {
		  ROS_ERROR(" header.frame_id needed ");
		  return false;
	  }

		ROS_INFO("pivoting_planner INIT PLAN");

	  bool success = false;
	  double traj_length = INFINITY;

	  int num_attempts=0;

	  ros::Time time0 = ros::Time::now();

	  while( ++num_attempts <= 1 )
	  {

		  ROS_INFO_STREAM("pivoting_planner attempt #" << num_attempts);
		  ROS_INFO_STREAM("Elapsed Time: " << (ros::Time::now() - time0).toSec() / 60.0 << " m");

		//cleanup
		move_group.clearPathConstraints();
	  move_group.clearPoseTargets();

	  move_group.setEndEffectorLink(end_effector_frame_id);

		  //Add configuration constraints
	  moveit_msgs::Constraints path_constraints_ = path_constraints;
	  if(use_configuration_constraints)
		add_joint_configuration_constraints(
			joint_conf_constraint_,
			move_group,
			path_constraints,
			path_constraints_
		);

		move_group.setPathConstraints(path_constraints_);

		move_group.setPoseTarget(target_pose);	  

	  move_group.setPlannerId(plannerID);

	  move_group.setPlanningTime(plan_time);
	  move_group.setNumPlanningAttempts(num_planning_attempts);

	  ROS_INFO_STREAM( "PlannerId: " << move_group.getPlannerId() );
	  ROS_INFO_STREAM( "PlanningTime: " << plan_time );

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      ROS_INFO("PLANNING...");
	  moveit::planning_interface::MoveItErrorCode result = move_group.plan(my_plan);
	  ROS_INFO_STREAM("PLANNED with result: " << result);
      success = (result == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	  ROS_INFO_STREAM("PLAN " << (success ? "SUCCESS" : "FAILED"));

	  if(success)
	  {
		  double tmp_traj_length = compute_traj_length(my_plan.trajectory_.joint_trajectory);
	  	  ROS_INFO_STREAM("PLAN SUCCESS with length: " << tmp_traj_length);
		  if(tmp_traj_length < traj_length)
		  {
			  ROS_INFO_STREAM("PLAN found a better traj");
			 planned_traj = my_plan.trajectory_.joint_trajectory;
			 traj_length = tmp_traj_length;
		  }
	  }

	  ROS_INFO_STREAM("pivoting_planner attempt #" << num_attempts);
		  ROS_INFO_STREAM("Elapsed Time: " << (ros::Time::now() - time0).toSec() / 60.0 << " m");

	} // end while

	  if(success)
	  {
		  ROS_INFO_STREAM("FINAL TRAJ LENGTH: " << traj_length);
	  }

	  return success;

  }

  void simulate_gravity_pivoting(
	  const std::string& object_cog_frame_id,
	  const std::string& pivoting_joint_name,
	  double current_pivoting_angle,
	  const std::string& robot_description_id
	  )
  {
	auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description_id);
	planning_scene_monitor->requestPlanningSceneState(); 
	planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);
	if(!planning_scene->knowsFrameTransform(object_cog_frame_id))
	{
		throw unknown_frame_transform("Unknown frame transform");
	}
	// This transform is from the model frame to object_cog_frame_id
	// model frame is ASSUMED to be world, i.e. z axis oriented as -g
	auto transform = planning_scene->getFrameTransform(object_cog_frame_id);
	Eigen::Vector3d z_cog = transform.rotation().col(2);
	Eigen::Vector3d z_world(0,0,1);
	double angle = acos( z_cog.dot(z_world) / ( z_cog.norm() * z_world.norm() ) );

	ROS_INFO_STREAM("simulate_gravity_pivoting: angle: " << angle);

	set_simulation_configuration(
	  {( current_pivoting_angle  + angle)}, 
	  {pivoting_joint_name}
	);	

  }

  void add_pivoting_constraints(
	  moveit_msgs::Constraints& constraints, 
	  const std::string& pivoting_link,
	  const std::string& robot_reference_frame_id
	  )
  {

	geometry_msgs::PoseStamped pivoting_link_pose;
	pivoting_link_pose.header.frame_id = pivoting_link;
	pivoting_link_pose.pose.orientation.w = 1.0;
	pivoting_link_pose.header.stamp = ros::Time::now();
	pivoting_link_pose = buffer_.transform(pivoting_link_pose, robot_reference_frame_id, ros::Duration(1.0));

	//Position Constraint
	moveit_msgs::PositionConstraint pos_const;
	pos_const.header.frame_id = robot_reference_frame_id;
	pos_const.link_name = pivoting_link;
	pos_const.target_point_offset.x = 0.0;
	pos_const.target_point_offset.y = 0.0;
	pos_const.target_point_offset.z = 0.0;
	shape_msgs::SolidPrimitive box_primitive;
	box_primitive.type = shape_msgs::SolidPrimitive::BOX;
	box_primitive.dimensions.push_back(0.0005); // x
	box_primitive.dimensions.push_back(0.0005); // y
	box_primitive.dimensions.push_back(0.0005); // z
	pos_const.constraint_region.primitives.push_back(box_primitive);
	pos_const.constraint_region.primitive_poses.push_back(pivoting_link_pose.pose);
	pos_const.weight = 1.0;
	constraints.position_constraints.push_back(pos_const);

	//Orientation Constraint TODO
	// moveit_msgs::OrientationConstraint orient_constr;
	// orient_constr.header.frame_id = robot_reference_frame_id;
	// orient_constr.orientation = pivoting_link_pose.pose.orientation;
	// orient_constr.link_name = pivoting_link;
	// orient_constr.weight = 1.0;
	// orient_constr.absolute_x_axis_tolerance = 0.0175;
	// orient_constr.absolute_y_axis_tolerance = 0.0175;
	// orient_constr.absolute_z_axis_tolerance = 0.0175;
	// constraints.orientation_constraints.push_back(orient_constr);
  }

  bool isVertical(geometry_msgs::PoseStamped pose, const std::string& vertical_reference_frame)
  {


	geometry_msgs::PoseStamped pose_world;
	pose.header.stamp = ros::Time::now();
	buffer_.transform(pose, pose_world, vertical_reference_frame, ros::Duration(1.0));

	Eigen::Quaterniond pose_quaternion;

	tf2::fromMsg(pose_world.pose.orientation, pose_quaternion);

	Eigen::Vector3d z_pos = pose_quaternion.toRotationMatrix().col(2);
	Eigen::Vector3d z_world(0,0,1);
	
	return ( fabs(z_pos.dot(z_world) -1.0) < 0.01 );

  }

  geometry_msgs::PoseStamped compute_after_pivoting_pose(
	const std::vector<std::string>& joint_names,
	const std::vector<double>& initial_joint_position,
	const std::vector<double>& fake_final_joint_position,
	const std::string& gripper_grasp_frame_id,
	const std::string& robot_reference_frame
  )
  {

	  set_simulation_configuration(fake_final_joint_position, joint_names);
	  
	  geometry_msgs::PoseStamped pose_orientation_gripper;
	  pose_orientation_gripper.pose.orientation.w = 1.0;
	  pose_orientation_gripper.header.stamp = ros::Time::now();
	  pose_orientation_gripper.header.frame_id = gripper_grasp_frame_id;
	  pose_orientation_gripper = buffer_.transform(pose_orientation_gripper, robot_reference_frame, ros::Duration(1.0));

	  set_simulation_configuration(initial_joint_position, joint_names);

	  geometry_msgs::PoseStamped pose_position_gripper;
	  pose_position_gripper.pose.orientation.w = 1.0;
	  pose_position_gripper.header.stamp = ros::Time::now();
	  pose_position_gripper.header.frame_id = gripper_grasp_frame_id;
	  pose_position_gripper = buffer_.transform(pose_position_gripper, robot_reference_frame, ros::Duration(1.0));

	  geometry_msgs::PoseStamped pose_gripper = pose_position_gripper;
	  pose_gripper.pose.orientation = pose_orientation_gripper.pose.orientation;

	  return pose_gripper;
  }

  void executePlanning(const sun_pivoting_planner_msgs::PivotingPlanGoalConstPtr &goal)
  {

	//DBG
	char ans;
	 std::cout << "setto la configurazione iniziale [button]" << std::endl; std::cin >> ans;

	/* Bring Simulated Robot to the initial configuration */
	set_simulation_configuration(goal->start_config, goal->start_config_joint_names);

	//DBG
	 std::cout << "configurazione iniziale settata [button]" << std::endl; std::cin >> ans;

	moveit::planning_interface::MoveGroupInterface move_group_arm(goal->group_arm_name);

	/* Try the standard plan first */
	ROS_INFO("Try using standard planner");
	trajectory_msgs::JointTrajectory planned_traj;

	//DBG
	 std::cout << "Pianifico standard [button]" << std::endl; std::cin >> ans;

	if(
		plan(
			move_group_arm, 
			goal->end_effector_frame_id, 
			goal->target_pose, 
			goal->path_constraints, 
			planned_traj, 
			true)
	  )
	{
		set_simulation_configuration(planned_traj.points.back().positions, planned_traj.joint_names);
		sun_pivoting_planner_msgs::PivotingPlanResult res;
		res.planned_trajectories.push_back(planned_traj);
		res.pivoting_mode.push_back(false);
		ROS_INFO("Solution found in standard mode");
		as_.setSucceeded(res);
		return;
	}

	//DBG
	 std::cout << "pianifico standard fallito [button]" << std::endl; std::cin >> ans;

	ROS_INFO("Solution NOT found in standard mode");

	if(!goal->activate_pivoting)
	{
		ROS_INFO("Pivoting Mode not activated");
		as_.setAborted();
		return;
	}

	// ICRA2020 plan
	ROS_INFO("Pivoting Mode activated");	

	moveit::planning_interface::MoveGroupInterface move_group_pivoting(goal->group_arm_pivoting_name);

	//DBG
	 std::cout << "change attached obj state [button]" << std::endl; std::cin >> ans;

	// ATTACH object to the pivoting joint
	std::string link_was_attached =
		detachCollisionObject(
			goal->attached_object_id, 
			buffer_,
			"robot_description"
		);
	attachCollisionObject(
	  goal->attached_object_id, 
	  move_group_pivoting.getLinkNames().back()
	);

	//DBG
	 std::cout << "attached obj state changed [button]" << std::endl; std::cin >> ans;

	 std::cout << "simulate pivoting [button]" << std::endl; std::cin >> ans;
	simulate_gravity_pivoting(
	  goal->attached_object_id + "/" + goal->cog_subframe,
	  move_group_pivoting.getJointNames().back(),
	  move_group_pivoting.getCurrentJointValues().back(),
	  "robot_description"
	);

	//DBG
	 std::cout << "pivoting simulated [button]" << std::endl; std::cin >> ans;

	//DBG
	 std::cout << "plan fake pivoting motion [button]" << std::endl; std::cin >> ans;

	if(
		!plan(
			move_group_pivoting, 
			goal->end_effector_frame_id, 
			goal->target_pose, 
			goal->path_constraints, 
			planned_traj, 
			true)
	  )
	{
		ROS_INFO("Fail to plan the fake pivoting trajectory");
		as_.setAborted();
		return;
	}

	//DBG
	std::cout << "fake pivoting planned [button]" << std::endl; std::cin >> ans;

	std::vector<std::string> fake_traj_joint_names = planned_traj.joint_names;
	std::vector<double> post_pivoting_joint_position = planned_traj.points.front().positions;
	std::vector<double> fake_traj_final_joint_position = planned_traj.points.back().positions;

	sun_pivoting_planner_msgs::PivotingPlanResult res;

	moveit_msgs::Constraints pivoting_fixed_position_constraint = goal->path_constraints;
	add_pivoting_constraints(
		pivoting_fixed_position_constraint, 
		move_group_pivoting.getLinkNames().back(), 
		move_group_pivoting.getPoseReferenceFrame()
	);

	//DBG
	 std::cout << "Constraints: " << pivoting_fixed_position_constraint << std::endl;

	if( isVertical(goal->target_pose, move_group_arm.getPoseReferenceFrame() ) )
	{
		res.pivoting_mode.push_back(true);
		res.pivoting_mode.push_back(false);
	}
	else
	{
		ROS_ERROR("NON VERTICAL CASE NOT IMPLEMENTED");
		as_.setAborted();
		return;
		// res.pivoting_mode.push_back(true);
		// res.pivoting_mode.push_back(false);
		// res.pivoting_mode.push_back(false);

		// geometry_msgs::PoseStamped p_s = compute_non_vertical_pivoting_pose();

		// if(!plan(move_group_arm, p_s, pivoting_fixed_position_constraint, planned_traj, false))
		// {
		// 	ROS_INFO("Fail to plan in pivoting mode");
		// 	as_.setAborted();
		// 	return;
		// }

		// res.planned_trajectories.push_back(planned_traj);
		// set_simulation_configuration(planned_traj.points.back().positions, planned_traj.joint_names);
	}

	 std::cout << "computing p_p [button]" << std::endl; std::cin >> ans;
	geometry_msgs::PoseStamped p_p = compute_after_pivoting_pose(
		fake_traj_joint_names,
		post_pivoting_joint_position,
		fake_traj_final_joint_position,
		move_group_arm.getLinkNames().back(),
		move_group_pivoting.getPoseReferenceFrame()
	);

	//DBG	
	 std::cout << p_p << std::endl;
	 std::cout << "p_p computed [button]" << std::endl; std::cin >> ans;

	 std::cout << "change attach grasp state [button]" << std::endl; std::cin >> ans;

	// TODO, is it possible to use move_group_pivoting here? In order to simulate the actual real robot motion

	// ATTACH object to the standard link
	detachCollisionObject(
	  goal->attached_object_id, 
	  buffer_,
	  "robot_description"
	);
	attachCollisionObject(
	  goal->attached_object_id, 
	  link_was_attached
	);

	//DBG
	 std::cout << "attach grasp state changed [button]" << std::endl; std::cin >> ans;

	//DBG
	 std::cout << "plan p_p [button]" << std::endl; std::cin >> ans;
	
	if(
		!plan(
			move_group_arm, 
			move_group_arm.getLinkNames().back(), 
			p_p, 
			pivoting_fixed_position_constraint, 
			planned_traj, 
			false
		)
	)
	{
		ROS_INFO("Fail to plan to p_p");
		as_.setAborted();
		return;
	}

	//DBG
	 std::cout << "p_p planned [button]" << std::endl; std::cin >> ans;
	
	res.planned_trajectories.push_back(planned_traj);
	set_simulation_configuration(planned_traj.points.back().positions, planned_traj.joint_names);

	//DBG
	 std::cout << "simulate pivoting [button]" << std::endl; std::cin >> ans;

	simulate_gravity_pivoting(
	  goal->attached_object_id + "/" + goal->cog_subframe,
	  move_group_pivoting.getJointNames().back(),
	  move_group_pivoting.getCurrentJointValues().back(),
	  "robot_description"
	);

	//DBG
	 std::cout << "pivoting simulated [button]" << std::endl; std::cin >> ans;

	//DBG
	 std::cout << "final standard plan [button]" << std::endl; std::cin >> ans;

	if(
		!plan(
			move_group_arm, 
			goal->end_effector_frame_id, 
			goal->target_pose, 
			goal->path_constraints, 
			planned_traj, 
			true
		)
		)
	{
		ROS_INFO("Fail to plan in pivoting mode");
		as_.setAborted();
		return;
	}

	//DBG
	 std::cout << "final standard plan done [button]" << std::endl; std::cin >> ans;

	res.planned_trajectories.push_back(planned_traj);
	set_simulation_configuration(planned_traj.points.back().positions, planned_traj.joint_names);

	ROS_INFO("Pivoting plan END!");

	as_.setSucceeded();
  }
};

} // namespace sun


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pivoting_planner_server");

  sun::PivotingPlannerActionServer pivoting_planner("pivoting_plan_action");
  ros::spin();

  return 0;
}
