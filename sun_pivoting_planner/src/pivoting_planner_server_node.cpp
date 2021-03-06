#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <sun_pivoting_planner_msgs/PivotingPlanAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include "std_msgs/Float64MultiArray.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #define DBG_BTN
#undef DBG_BTN

#include "sun_pivoting_planner/exceptions.h"
#include "sun_pivoting_planner/utility.h"

namespace sun
{
class PivotingPlannerActionServer
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sun_pivoting_planner_msgs::PivotingPlanAction> as_;  // NodeHandle instance must be
																					 // created before this line.

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  tf2_ros::TransformListener tf2_transform_listener_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  ros::Publisher pub_simulated_joint_;

  Joint_Conf_Constraint joint_conf_constraint_;
  std::vector<double> joint_conf_constraint_multiplier_;
  bool try_unconstrained_first = true; // debug flag. Keep it true

  double plan_time = 2.0;
  int num_planning_attempts = 4;
  std::string plannerID = "PRMstar";
  //   std::string plannerID = "RRTstar";

public:
  PivotingPlannerActionServer(const std::string& name, const ros::NodeHandle& nh = ros::NodeHandle())
	: nh_(nh)
	, as_(nh_, name, boost::bind(&PivotingPlannerActionServer::executePlanning, this, _1), false)
	, tf2_buffer_(new tf2_ros::Buffer())
	, tf2_transform_listener_(*tf2_buffer_.get())
	, planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf2_buffer_))
  {
	as_.start();
	pub_simulated_joint_ = nh_.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);

	joint_conf_constraint_.joint_names.push_back("iiwa_joint_1");
	joint_conf_constraint_.move_range.push_back((2.0 * 170.0) * M_PI / 180.0);
	joint_conf_constraint_.joint_names.push_back("iiwa_joint_2");
	joint_conf_constraint_.move_range.push_back((2.0 * 120.0) * M_PI / 180.0);
	joint_conf_constraint_.joint_names.push_back("iiwa_joint_3");
	joint_conf_constraint_.move_range.push_back((2.0 * 170.0) * M_PI / 180.0);
	joint_conf_constraint_.joint_names.push_back("iiwa_joint_4");
	joint_conf_constraint_.move_range.push_back((2.0 * 120.0) * M_PI / 180.0);
	joint_conf_constraint_.joint_names.push_back("iiwa_joint_5");
	joint_conf_constraint_.move_range.push_back((2.0 * 170.0) * 0.9 * M_PI / 180.0);
	// joint_conf_constraint_.joint_names.push_back("iiwa_joint_6");
	// joint_conf_constraint_.move_range.push_back((2.0 * 120.0) * 0.9 * M_PI / 180.0);
	// joint_conf_constraint_.joint_names.push_back("iiwa_joint_7");
	// joint_conf_constraint_.move_range.push_back((2.0 * 175.0) * 0.9 * M_PI / 180.0);
	// joint_conf_constraint_.move_range.push_back((2.0*120.0)/1.0 *M_PI/180.0);
	// joint_conf_constraint_.move_range.push_back((2.0*170.0)/1.0 *M_PI/180.0);
	// joint_conf_constraint_.move_range.push_back((2.0*120.0)/1.0 *M_PI/180.0);

	joint_conf_constraint_multiplier_ = { 1.0 / 4.0, 1.5 / 4.0, 2.0 / 4.0 };
	if (!try_unconstrained_first)
	  joint_conf_constraint_multiplier_.push_back(1.0);
  }

  ~PivotingPlannerActionServer(void)
  {
  }

  void set_simulation_configuration(const std::vector<double>& joint_position,
									const std::vector<std::string>& joint_names)
  {
	sensor_msgs::JointState msg;
	msg.name = joint_names;
	msg.position = joint_position;
	msg.velocity.resize(joint_names.size());
	msg.effort.resize(joint_names.size());
	pub_simulated_joint_.publish(msg);
	ros::Duration(0.5).sleep();
  }

  bool plan(moveit::planning_interface::MoveGroupInterface& move_group, const std::string& end_effector_frame_id,
			const geometry_msgs::PoseStamped& target_pose, const moveit_msgs::Constraints& path_constraints,
			trajectory_msgs::JointTrajectory& planned_traj, bool use_configuration_constraints = true, bool absolutely_no_joint_constraints = false)
  {
	if (target_pose.header.frame_id == "")
	{
	  ROS_ERROR(" header.frame_id needed ");
	  return false;
	}

	ROS_INFO("pivoting_planner INIT low level PLAN");

	bool success = false;
	ros::Time time0 = ros::Time::now();
	trajectory_msgs::JointTrajectory unconstrained_traj;

	////////////

	int i = try_unconstrained_first ? -1 : 0;
	while (true)
	{
	  ROS_INFO_STREAM("Plan " << (i + 1) << "/"
							  << (use_configuration_constraints ? joint_conf_constraint_multiplier_.size() : 0));

	  ROS_INFO_STREAM("Elapsed Time: " << (ros::Time::now() - time0).toSec() << " s");

	  // First plan without joint constraints
	  bool use_configuration_constraints_internal = (i != -1) && use_configuration_constraints;

	  /*Plan kernel*/
	  // cleanup
	  move_group.clearPathConstraints();
	  move_group.clearPoseTargets();
	  move_group.setEndEffectorLink(end_effector_frame_id);
	  // Add configuration constraints
	  moveit_msgs::Constraints path_constraints_ = path_constraints;
	  if (use_configuration_constraints_internal)
	  {
		add_joint_configuration_constraints(joint_conf_constraint_, move_group, path_constraints, path_constraints_,
											joint_conf_constraint_multiplier_[i]);
	  }
	  else if(!absolutely_no_joint_constraints)
	  {
		add_joint_configuration_constraints(joint_conf_constraint_, move_group, path_constraints, path_constraints_,
											1.0);
	  }
	  // plan
	  move_group.setPathConstraints(path_constraints_);
	  move_group.setPoseTarget(target_pose);
	  move_group.setPlannerId(plannerID);
	  move_group.setPlanningTime(plan_time);
	  move_group.setNumPlanningAttempts(num_planning_attempts);
	  ROS_INFO_STREAM("PlannerId: " << move_group.getPlannerId());
	  ROS_INFO_STREAM("PlanningTime: " << plan_time);
	  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	  ROS_INFO("PLANNING...");
	  moveit::planning_interface::MoveItErrorCode result = move_group.plan(my_plan);
	  ROS_INFO_STREAM("PLANNED with result: " << result);
	  success = (result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	  ROS_INFO_STREAM("PLAN " << (success ? "SUCCESS" : "FAILED"));
	  /*END Plan kernel*/

	  // first unconstrained plan failed -> plan infeasible
	  if (i == -1 && !success)
	  {
		ROS_ERROR_STREAM("Planner without joint constraint not success");
		return false;
	  }

	  if (i == -1 && success)
	  {
		unconstrained_traj = my_plan.trajectory_.joint_trajectory;
	  }

	  //   The first condition should match first
	  if (try_unconstrained_first && !use_configuration_constraints && !success)
	  {
		ROS_ERROR_STREAM("I SHOULD NOT BE HERE - Planner without joint constraint not success");
		return false;
	  }

	  if (!use_configuration_constraints && success)
	  {
		double tmp_traj_length = compute_traj_length(my_plan.trajectory_.joint_trajectory);
		ROS_INFO_STREAM("Planner without joint constraint success! traj_length=" << tmp_traj_length);
		ROS_INFO_STREAM("Elapsed Time: " << (ros::Time::now() - time0).toSec() << " s");
		planned_traj = my_plan.trajectory_.joint_trajectory;
		return true;
	  }

	  if (i != -1 && success)
	  {
		double tmp_traj_length = compute_traj_length(my_plan.trajectory_.joint_trajectory);
		ROS_INFO_STREAM("PLAN SUCCESS with length: " << tmp_traj_length << " at attempt =" << (i + 1) << "/"
													 << joint_conf_constraint_multiplier_.size());
		ROS_INFO_STREAM("Elapsed Time: " << (ros::Time::now() - time0).toSec() << " s");
		planned_traj = my_plan.trajectory_.joint_trajectory;
		return true;
	  }

	  i++;
	  if (i >= joint_conf_constraint_multiplier_.size())
	  {
		if (try_unconstrained_first)
		{
		  ROS_ERROR_STREAM("NO SUCCESS");
		  return false;
		}
		double tmp_traj_length = compute_traj_length(unconstrained_traj);
		ROS_INFO_STREAM("PLAN SUCCESS only with the unconstrained joint traj - length: " << tmp_traj_length);
		ROS_INFO_STREAM("Elapsed Time: " << (ros::Time::now() - time0).toSec() << " s");
		planned_traj = unconstrained_traj;
		return true;
	  }
	}

	ROS_ERROR_STREAM("FATAL - after while true in plan !?");
	return false;
  }

  double simulate_gravity_pivoting_not_attached(const std::string& object_cog_frame_id,
												const std::string& pivoting_joint_name, double current_pivoting_angle,
												const std::string& attached_object_id,
												const std::string& pivoting_link_name)
  {
	std::string link_was_attached =
		detachCollisionObject(planning_scene_monitor_, attached_object_id, "robot_description");
	attachCollisionObject(attached_object_id, pivoting_link_name);
	double piv_angle =
		simulate_gravity_pivoting(object_cog_frame_id, pivoting_joint_name, pivoting_link_name, current_pivoting_angle);
	// ATTACH object to the standard link
	detachCollisionObject(planning_scene_monitor_, attached_object_id, "robot_description");
	attachCollisionObject(attached_object_id, link_was_attached);
	return piv_angle;
  }

  double simulate_gravity_pivoting(const std::string& object_cog_frame_id, const std::string& pivoting_joint_name,
								   const std::string& pivoting_link_name, double current_pivoting_angle)
  {
// DBG
#ifdef DBG_BTN
	char ans;
	std::cout << "simulate_gravity_pivoting request planning state init [button]" << std::endl;
	std::cin >> ans;
#endif

	planning_scene_monitor_->requestPlanningSceneState();

// DBG
#ifdef DBG_BTN
	std::cout << "simulate_gravity_pivoting request planning state done [button]" << std::endl;
	std::cin >> ans;
#endif

	planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro_(planning_scene_monitor_);
	if (!planning_scene_ro_->knowsFrameTransform(object_cog_frame_id))
	{
	  throw unknown_frame_transform("Unknown object_cog_frame_id transform " + object_cog_frame_id);
	}
	if (!planning_scene_ro_->knowsFrameTransform(pivoting_link_name))
	{
	  throw unknown_frame_transform("Unknown pivoting_link_name transform " + pivoting_link_name);
	}

	// This transform is from the model frame to object_cog_frame_id
	// model frame is ASSUMED to be world, i.e. z axis oriented as -g
	auto transform = planning_scene_ro_->getFrameTransform(object_cog_frame_id);
	Eigen::Vector3d z_cog = transform.rotation().col(2);
	Eigen::Vector3d z_world(0, 0, 1);

	double angle = acos(z_cog.dot(z_world) / (z_cog.norm() * z_world.norm()));

	if (angle < 0.05)
	{
	  ROS_INFO_STREAM("simulate_gravity_pivoting the cog/g angle is almost zero: " << angle);
	  return 0.0;
	}

	// ASSUMPTION: pivoting joint axis is y
	auto pivoting_link_transform = planning_scene_ro_->getFrameTransform(pivoting_link_name);
	Eigen::Vector3d piv_joint = pivoting_link_transform.rotation().col(1);

// DBG
#ifdef DBG_BTN
	std::cout << "piv_joint: " << piv_joint << std::endl;
#endif

	Eigen::Vector3d cross = z_cog.cross(z_world);

// DBG
#ifdef DBG_BTN
	std::cout << "cross: " << cross << std::endl;
#endif

	double cross_angle = acos(cross.dot(piv_joint) / (cross.norm() * piv_joint.norm()));

	if (cross_angle < 0.1)
	{
// angle = angle;
// DBG
#ifdef DBG_BTN
	  std::cout << "0 cross_angle" << std::endl;
#endif
	}
	else if (fabs(cross_angle - M_PI) < 0.1)
	{
	  angle = -angle;

// DBG
#ifdef DBG_BTN
	  std::cout << "-pi cross_angle" << std::endl;
#endif
	}
	else
	{
	  throw invalid_gravity_pivoting_angle("The angle between cross and piv_joint is invalid " +
										   std::to_string(cross_angle));
	}

	ROS_INFO_STREAM("simulate_gravity_pivoting: angle: " << angle);

// DBG
#ifdef DBG_BTN
	std::cout << "simulate_gravity_pivoting check angle! [button]" << std::endl;
	std::cin >> ans;
#endif

	set_simulation_configuration({ (current_pivoting_angle + angle) }, { pivoting_joint_name });

	return angle;
  }

  void add_pivoting_constraints(moveit_msgs::Constraints& constraints, const std::string& pivoting_link,
								const std::string& robot_reference_frame_id)
  {
	geometry_msgs::PoseStamped pivoting_link_pose;
	pivoting_link_pose.header.frame_id = pivoting_link;
	pivoting_link_pose.pose.orientation.w = 1.0;
	pivoting_link_pose.header.stamp = ros::Time::now();
	pivoting_link_pose = tf2_buffer_->transform(pivoting_link_pose, robot_reference_frame_id, ros::Duration(1.0));

	// Position Constraint
	moveit_msgs::PositionConstraint pos_const;
	pos_const.header.frame_id = robot_reference_frame_id;
	pos_const.link_name = pivoting_link;
	pos_const.target_point_offset.x = 0.0;
	pos_const.target_point_offset.y = 0.0;
	pos_const.target_point_offset.z = 0.0;
	shape_msgs::SolidPrimitive box_primitive;
	box_primitive.type = shape_msgs::SolidPrimitive::BOX;
	box_primitive.dimensions.push_back(0.05);  // x
	box_primitive.dimensions.push_back(0.05);  // y
	box_primitive.dimensions.push_back(0.05);  // z
	pos_const.constraint_region.primitives.push_back(box_primitive);
	pos_const.constraint_region.primitive_poses.push_back(pivoting_link_pose.pose);
	pos_const.weight = 1.0;
	constraints.position_constraints.push_back(pos_const);

	// Orientation Constraint TODO
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
	tf2_buffer_->transform(pose, pose_world, vertical_reference_frame, ros::Duration(1.0));

	Eigen::Quaterniond pose_quaternion;

	tf2::fromMsg(pose_world.pose.orientation, pose_quaternion);

	Eigen::Vector3d z_pos = pose_quaternion.toRotationMatrix().col(2);
	Eigen::Vector3d z_world(0, 0, 1);

	return (fabs(z_pos.dot(z_world) - 1.0) < 0.01);
  }

  geometry_msgs::PoseStamped compute_after_pivoting_pose(const std::vector<std::string>& joint_names,
														 const std::vector<double>& initial_joint_position,
														 const std::vector<double>& fake_final_joint_position,
														 const std::string& gripper_grasp_frame_id,
														 const std::string& robot_reference_frame)
  {
	set_simulation_configuration(fake_final_joint_position, joint_names);

	geometry_msgs::PoseStamped pose_orientation_gripper;
	pose_orientation_gripper.pose.orientation.w = 1.0;
	pose_orientation_gripper.header.stamp = ros::Time::now();
	pose_orientation_gripper.header.frame_id = gripper_grasp_frame_id;
	pose_orientation_gripper =
		tf2_buffer_->transform(pose_orientation_gripper, robot_reference_frame, ros::Duration(1.0));

	set_simulation_configuration(initial_joint_position, joint_names);

	geometry_msgs::PoseStamped pose_position_gripper;
	pose_position_gripper.pose.orientation.w = 1.0;
	pose_position_gripper.header.stamp = ros::Time::now();
	pose_position_gripper.header.frame_id = gripper_grasp_frame_id;
	pose_position_gripper = tf2_buffer_->transform(pose_position_gripper, robot_reference_frame, ros::Duration(1.0));

	geometry_msgs::PoseStamped pose_gripper = pose_position_gripper;
	pose_gripper.pose.orientation = pose_orientation_gripper.pose.orientation;

	return pose_gripper;
  }

  geometry_msgs::PoseStamped compute_non_vertical_pivoting_pose(const std::vector<std::string>& joint_names,
																const std::vector<double>& initial_joint_position,
																const std::vector<double>& fake_final_joint_position,
																const std::string& gripper_grasp_frame_id,
																const std::string& robot_reference_frame,
																const std::string& pivoting_frame_id)
  {
	set_simulation_configuration(fake_final_joint_position, joint_names);

	geometry_msgs::PoseStamped pivoting_pose_grasp;
	pivoting_pose_grasp.pose.orientation.w = 1.0;
	pivoting_pose_grasp.header.stamp = ros::Time::now();
	pivoting_pose_grasp.header.frame_id = gripper_grasp_frame_id;
	pivoting_pose_grasp = tf2_buffer_->transform(pivoting_pose_grasp, pivoting_frame_id, ros::Duration(1.0));

	Eigen::Quaterniond pivoting_quaternion_grasp;
	tf2::fromMsg(pivoting_pose_grasp.pose.orientation, pivoting_quaternion_grasp);

	set_simulation_configuration(initial_joint_position, joint_names);

	geometry_msgs::PoseStamped pose_pivoting;
	pose_pivoting.pose.orientation.w = 1.0;
	pose_pivoting.header.stamp = ros::Time::now();
	pose_pivoting.header.frame_id = pivoting_frame_id;
	pose_pivoting = tf2_buffer_->transform(pose_pivoting, robot_reference_frame, ros::Duration(1.0));

	Eigen::Quaterniond quaternion_pivoting;
	tf2::fromMsg(pose_pivoting.pose.orientation, quaternion_pivoting);

	Eigen::Quaterniond quaternion_grasp = quaternion_pivoting * pivoting_quaternion_grasp;

	geometry_msgs::PoseStamped pose_grasp;
	pose_grasp.pose.orientation.w = 1.0;
	pose_grasp.header.stamp = ros::Time::now();
	pose_grasp.header.frame_id = gripper_grasp_frame_id;
	pose_grasp = tf2_buffer_->transform(pose_grasp, robot_reference_frame, ros::Duration(1.0));

	pose_grasp.pose.orientation = tf2::toMsg(quaternion_grasp);

	return pose_grasp;
  }

  void executePlanning(const sun_pivoting_planner_msgs::PivotingPlanGoalConstPtr& goal)
  {
// DBG
#ifdef DBG_BTN
	char ans;
	std::cout << "setto la configurazione iniziale [button]" << std::endl;
	std::cin >> ans;
#endif

	if(goal->plan_time > 0)
	{
		plan_time = goal->plan_time;
	}
	else
	{
		plan_time = 2.0;
	}
	

	/* Bring Simulated Robot to the initial configuration */
	set_simulation_configuration(goal->start_config, goal->start_config_joint_names);

// DBG
#ifdef DBG_BTN
	std::cout << "configurazione iniziale settata [button]" << std::endl;
	std::cin >> ans;
#endif

	moveit::planning_interface::MoveGroupInterface move_group_arm(goal->group_arm_name);

	/* Try the standard plan first */
	ROS_INFO("Try using standard planner");
	trajectory_msgs::JointTrajectory planned_traj;

// DBG
#ifdef DBG_BTN
	std::cout << "Pianifico standard [button]" << std::endl;
	std::cin >> ans;
#endif

	if (plan(move_group_arm, goal->end_effector_frame_id, goal->target_pose, goal->path_constraints, planned_traj,
			 true))
	{
	  set_simulation_configuration(planned_traj.points.back().positions, planned_traj.joint_names);
	  sun_pivoting_planner_msgs::PivotingPlanResult res;
	  res.planned_trajectories.push_back(planned_traj);
	  res.pivoting_mode.push_back(false);
	  ROS_INFO("Solution found in standard mode");
	  as_.setSucceeded(res);
	  return;
	}

// DBG
#ifdef DBG_BTN
	std::cout << "pianifico standard fallito [button]" << std::endl;
	std::cin >> ans;
#endif

	ROS_INFO("Solution NOT found in standard mode");

	if (!goal->activate_pivoting)
	{
	  ROS_INFO("Pivoting Mode not activated");
	  as_.setAborted();
	  return;
	}

	// ICRA2020 plan
	ROS_INFO("Pivoting Mode activated");

	moveit::planning_interface::MoveGroupInterface move_group_pivoting(goal->group_arm_pivoting_name);

	double piv_angle = simulate_gravity_pivoting_not_attached(
		goal->attached_object_id + "/" + goal->cog_subframe, move_group_pivoting.getJointNames().back(),
		move_group_pivoting.getCurrentJointValues().back(), goal->attached_object_id,
		move_group_pivoting.getLinkNames().back());

	ROS_INFO("Simple Pivoting");
	if ((piv_angle != 0.0) && plan(move_group_arm, goal->end_effector_frame_id, goal->target_pose,
								   goal->path_constraints, planned_traj, true))
	{
	  set_simulation_configuration(planned_traj.points.back().positions, planned_traj.joint_names);
	  sun_pivoting_planner_msgs::PivotingPlanResult res;
	  res.planned_trajectories.push_back(trajectory_msgs::JointTrajectory());  // empty traj = no move
	  res.pivoting_mode.push_back(true);
	  res.planned_trajectories.push_back(planned_traj);
	  res.pivoting_mode.push_back(false);
	  ROS_INFO("Solution found in simple pivoting mode");
	  as_.setSucceeded(res);
	  return;
	}

	ROS_INFO("Simple pivoting fail!");

// DBG
#ifdef DBG_BTN
	std::cout << "change attached obj state [button]" << std::endl;
	std::cin >> ans;
#endif

	// ATTACH object to the pivoting joint
	std::string link_was_attached =
		detachCollisionObject(planning_scene_monitor_, goal->attached_object_id, "robot_description");
	attachCollisionObject(goal->attached_object_id, move_group_pivoting.getLinkNames().back());

// DBG
#ifdef DBG_BTN
	print_collision_obj_state(planning_scene_monitor_);
	std::cout << "attached obj state changed [button]" << std::endl;
	std::cin >> ans;
	std::cout << "simulate pivoting [button]" << std::endl;
	std::cin >> ans;
#endif

// DBG
#ifdef DBG_BTN
	std::cout << "pivoting simulated [button]" << std::endl;
	std::cin >> ans;
#endif

// DBG
#ifdef DBG_BTN
	std::cout << "plan fake pivoting motion [button]" << std::endl;
	std::cin >> ans;
#endif

	if (!plan(move_group_pivoting, goal->end_effector_frame_id, goal->target_pose, goal->path_constraints, planned_traj,
			  true))
	{
	  ROS_INFO("Fail to plan the fake pivoting trajectory");
	  as_.setAborted();
	  return;
	}

// DBG
#ifdef DBG_BTN
	std::cout << "fake pivoting planned [button]" << std::endl;
	std::cin >> ans;
#endif

	std::vector<std::string> fake_traj_joint_names = planned_traj.joint_names;
	std::vector<double> post_pivoting_joint_position = planned_traj.points.front().positions;
	std::vector<double> fake_traj_final_joint_position = planned_traj.points.back().positions;

	sun_pivoting_planner_msgs::PivotingPlanResult res;

	moveit_msgs::Constraints pivoting_fixed_position_constraint = goal->path_constraints;
	add_pivoting_constraints(pivoting_fixed_position_constraint, move_group_arm.getLinkNames().back(),
							 move_group_arm.getPoseReferenceFrame());

// DBG
#ifdef DBG_BTN
	std::cout << "Constraints: " << pivoting_fixed_position_constraint << std::endl;
#endif

	res.pivoting_mode.push_back(true);
	res.pivoting_mode.push_back(false);

	geometry_msgs::PoseStamped p_s = compute_non_vertical_pivoting_pose(
		fake_traj_joint_names, post_pivoting_joint_position, fake_traj_final_joint_position,
		move_group_arm.getLinkNames().back(), move_group_pivoting.getPoseReferenceFrame(),
		move_group_pivoting.getLinkNames().back());

	// TODO, is it possible to use move_group_pivoting here? In order to simulate the actual real robot motion
	if (!plan(move_group_arm, move_group_arm.getLinkNames().back(), p_s, pivoting_fixed_position_constraint,
			  planned_traj, false, true))
	{
	  ROS_INFO("Fail to plan in pivoting mode");
	  as_.setAborted();
	  return;
	}

	res.planned_trajectories.push_back(planned_traj);
	set_simulation_configuration(planned_traj.points.back().positions, planned_traj.joint_names);

	// At this point the object should be vertical
	simulate_gravity_pivoting_not_attached(goal->attached_object_id + "/" + goal->cog_subframe,
										   move_group_pivoting.getJointNames().back(),
										   move_group_pivoting.getCurrentJointValues().back(), goal->attached_object_id,
										   move_group_pivoting.getLinkNames().back());

	post_pivoting_joint_position = move_group_pivoting.getCurrentJointValues();

#ifdef DBG_BTN
	std::cout << "computing p_p [button]" << std::endl;
	std::cin >> ans;
#endif

	// ATTACH object to the standard link
	detachCollisionObject(planning_scene_monitor_, goal->attached_object_id, "robot_description");
	attachCollisionObject(goal->attached_object_id, link_was_attached);

// DBG
#ifdef DBG_BTN
	print_collision_obj_state(planning_scene_monitor_);
	std::cout << "attach grasp state changed [button]" << std::endl;
	std::cin >> ans;
#endif

// DBG
#ifdef DBG_BTN
	std::cout << "final standard plan [button]" << std::endl;
	std::cin >> ans;
#endif

	if (!plan(move_group_arm, goal->end_effector_frame_id, goal->target_pose, goal->path_constraints, planned_traj,
			  true))
	{
	  ROS_INFO("Fail to plan in pivoting mode");
	  as_.setAborted();
	  return;
	}

// DBG
#ifdef DBG_BTN
	std::cout << "final standard plan done [button]" << std::endl;
	std::cin >> ans;
#endif

	res.planned_trajectories.push_back(planned_traj);
	set_simulation_configuration(planned_traj.points.back().positions, planned_traj.joint_names);

	ROS_INFO("Pivoting plan END!");

	as_.setSucceeded(res);
  }
};

}  // namespace sun

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pivoting_planner_server");
  sun::PivotingPlannerActionServer pivoting_planner("pivoting_plan_action");
  ros::spin();

  return 0;
}
