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
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_msgs/Float64MultiArray.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

namespace
{
  bool isStateValid(	const planning_scene::PlanningScene *planning_scene,
                  	const kinematic_constraints::KinematicConstraintSet *constraint_set,
                  	robot_state::RobotState *state,
                  	const robot_state::JointModelGroup *group, 
		  	const double *ik_solution)
  {
  	state->setJointGroupPositions(group, ik_solution);
  	state->update();
  	return (!planning_scene || !planning_scene->isStateColliding(*state, group->getName())) &&
    	(!constraint_set || constraint_set->decide(*state).satisfied);
  }
}


class PivotingPlannerActionServer
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sun_pivoting_planner_msgs::PivotingPlanAction> as_; // NodeHandle instance must be created before this line.
  std::string action_name_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

public:

  PivotingPlannerActionServer(std::string name) :
    as_(nh_, name, boost::bind(&PivotingPlannerActionServer::executePlanning, this, _1), false),
    action_name_(name),
	tf2_(buffer_)
  {
    as_.start();
	pub_fake_joint = nh_.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states",1);
	//joint_conf_constraint.joint_names.push_back("iiwa_joint_1");
	joint_conf_constraint.joint_names.push_back("iiwa_joint_2");
	joint_conf_constraint.joint_names.push_back("iiwa_joint_3");
	joint_conf_constraint.joint_names.push_back("iiwa_joint_4");
	joint_conf_constraint.move_range.push_back((2.0*120.0)/4.0 *M_PI/180.0);	
	joint_conf_constraint.move_range.push_back((2.0*170.0)/4.0 *M_PI/180.0);	
	joint_conf_constraint.move_range.push_back((2.0*120.0)/4.0 *M_PI/180.0);	
	// joint_conf_constraint.move_range.push_back((2.0*120.0)/1.0 *M_PI/180.0);	
	// joint_conf_constraint.move_range.push_back((2.0*170.0)/1.0 *M_PI/180.0);	
	// joint_conf_constraint.move_range.push_back((2.0*120.0)/1.0 *M_PI/180.0);
  }

  ~PivotingPlannerActionServer(void)
  {
  } 

  ros::Publisher pub_fake_joint;
  void set_initial_planning_configuration(
	  const std::vector<double>& joint_position, 
	  const std::vector<string>& joint_names
	)
  {
	sensor_msgs::JointState msg;
	msg.name = joint_names;
	msg.position = joint_position;
	msg.velocity.resize(joint_names.size());
	msg.effort.resize(joint_names.size());
	pub_fake_joint.publish(msg);
	ros::Duration(1.0).sleep();
	// cout << " sett..." << endl;
	// moveit::planning_interface::MoveGroupInterface arm_group(group_name);
	// robot_state::RobotState current_state(*arm_group.getCurrentState());
	// const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(group_name);
	// current_state.setJointGroupPositions(joint_model_group, joint_position);
	// cout << " sett..." << endl;
  }

  double compute_traj_length(const trajectory_msgs::JointTrajectory& traj)
  {
	  double length = 0.0;
	  for( int i = 1; i< traj.points.size(); i++ )
	  {
		  double dist_local_square = 0.0;
		  for(int j=0; j<traj.joint_names.size()-1; j++)
		  {
			  dist_local_square += pow(traj.points[i-1].positions[j] - traj.points[i].positions[j], 2);
		  }
		  length += sqrt(dist_local_square);
	  }
	  return length;
  }

  struct Joint_Conf_Constraint
  {
	std::vector<string> joint_names;
	std::vector<double> move_range;
  };

  Joint_Conf_Constraint joint_conf_constraint;

  void add_joint_configuration_constraints(
	  moveit::planning_interface::MoveGroupInterface& move_group,
	  const moveit_msgs::Constraints& path_constraints_in,
	  moveit_msgs::Constraints& path_constraints_out
	  )
  {

	  path_constraints_out = path_constraints_in;

	  std::vector<double> current_joint = move_group.getCurrentJointValues();

	  for(int j=0; j<joint_conf_constraint.joint_names.size(); j++)
	  {

		std::string j_name = joint_conf_constraint.joint_names[j];
		double j_move_range = joint_conf_constraint.move_range[j];

		moveit_msgs::JointConstraint joint_constraint;
		joint_constraint.weight = 1.0;
		joint_constraint.joint_name = j_name;

		move_group.getJointNames();
		int j_index = -1;
		for( int i=0; i<move_group.getJointNames().size(); i++ )
		{
			if(move_group.getJointNames()[i] == j_name)
			{
				j_index = i;
				break;
			}
		}
		if(j_index == -1)
		{
			continue;

		}
		joint_constraint.position = current_joint[j_index];


		joint_constraint.tolerance_below = j_move_range/2.0;
		joint_constraint.tolerance_above = j_move_range/2.0;

		if((joint_constraint.position-joint_constraint.tolerance_below)<move_group.getRobotModel()->getVariableBounds(j_name).min_position_)
		{
			joint_constraint.tolerance_below = fabs(joint_constraint.position - move_group.getRobotModel()->getVariableBounds(j_name).min_position_);
			joint_constraint.tolerance_above = fabs(move_group.getRobotModel()->getVariableBounds(j_name).min_position_ + j_move_range - joint_constraint.position);
		}

		if((joint_constraint.position+joint_constraint.tolerance_above)>move_group.getRobotModel()->getVariableBounds(j_name).max_position_)
		{
			joint_constraint.tolerance_above = fabs(joint_constraint.position - move_group.getRobotModel()->getVariableBounds(j_name).max_position_);
			joint_constraint.tolerance_below = fabs(move_group.getRobotModel()->getVariableBounds(j_name).max_position_ - j_move_range - joint_constraint.position);
		}

		std::cout << "Joint: " << j_name 
		<< "\n" << joint_constraint;

		path_constraints_out.joint_constraints.push_back(joint_constraint);
	  }
  }

  bool plan(
	  moveit::planning_interface::MoveGroupInterface& move_group, 
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

	  cout << "Init plan" << endl;

	  bool success = false;
	  double traj_length = INFINITY;

	  int num_attempts=0;

	  ros::Time time0 = ros::Time::now();

	  moveit_msgs::Constraints path_constraints_ = path_constraints;
	  if(use_configuration_constraints)
		add_joint_configuration_constraints(
			move_group,
			path_constraints,
			path_constraints_
		);

	  
	//   while((traj_length > 2.0) && ros::ok())
	//  {

	  cout << "Attempt: " << ++num_attempts << endl;
	  cout << "Elapsed Time: " << (ros::Time::now() - time0).toSec() / 60.0 << endl;

	  move_group.clearPathConstraints();
	  move_group.clearPoseTargets();

	  ROS_INFO_NAMED("------->", "Reference frame: %s", move_group.getPlanningFrame().c_str());

      move_group.setPathConstraints(path_constraints_);

	  std::cout << "target_pose: " << endl << target_pose << endl;

	  move_group.setPoseTarget(target_pose);

	  move_group.setPlannerId("PRMstar");

	  cout << "Planner ID: " << move_group.getPlannerId() << endl;

	  double plan_time = 5.0;
	  move_group.setPlanningTime(plan_time);
	  move_group.setNumPlanningAttempts(4);
	  cout << "setPlanningTime: " << plan_time << endl;
	  cout << "Constraints: " << move_group.getPathConstraints() << endl;

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      ROS_INFO("PLANNING...");
	  moveit::planning_interface::MoveItErrorCode result = move_group.plan(my_plan);
	  ROS_INFO("PLANNED");
      success = (result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	  cout << "RESULT CODE: " << result << endl;
	  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

	  planned_traj = my_plan.trajectory_.joint_trajectory;

	  if(success)
	  {
		  traj_length = compute_traj_length(planned_traj);
		  cout << "New Length: " << traj_length << endl;
	  }

	//  } // end while

	  if(success)
	  {
		  double length = compute_traj_length(planned_traj);
		  cout << "TRAJ LENGTH: " << length << endl;
	  }

	  return success;

  }

  void simulate_gravity_pivoting(
	  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor____,
	  const std::string& object_cog_frame_id,
	  const std::string& pivoting_joint_name,
	  double current_pivoting_angle
	  )
  {
	auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
	  cout << "1" << endl;
	planning_scene_monitor->requestPlanningSceneState(); 
	cout << "2" << endl;
	planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);
cout << "3" << endl;
	if(!planning_scene->knowsFrameTransform(object_cog_frame_id))
	{
		throw "Unknown frame transform";
	}
	cout << "4" << endl;
	// This transform is from the model frame to object_cog_frame_id
	// model frame is ASSUMED to be world, i.e. z axis oriented as -g
	auto transform = planning_scene->getFrameTransform(object_cog_frame_id);
cout << "5" << endl;
	Eigen::Vector3d z_cog = transform.rotation().col(2);
	cout << "z_cog: " << z_cog << endl;
	Eigen::Vector3d z_world(0,0,1);
	cout << "z_world: " << z_world << endl;

	double angle = acos( z_cog.dot(z_world) / ( z_cog.norm() * z_world.norm() ) );
	
	cout << "simulate_gravity_pivoting: angle: " << angle << endl;

	std::vector<string> piv_j_name; piv_j_name.push_back(pivoting_joint_name);
	std::vector<double> piv_j_pos; piv_j_pos.push_back( current_pivoting_angle  + angle);
	set_initial_planning_configuration(
	  piv_j_pos, 
	  piv_j_name
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
	const std::vector<string>& joint_names,
	const std::vector<double>& initial_joint_position,
	const std::vector<double>& fake_final_joint_position,
	const std::string& gripper_grasp_frame_id,
	const std::string& robot_reference_frame
  )
  {

	  set_initial_planning_configuration(fake_final_joint_position, joint_names);
	  
	  geometry_msgs::PoseStamped pose_orientation_gripper;
	  pose_orientation_gripper.pose.orientation.w = 1.0;
	  pose_orientation_gripper.header.stamp = ros::Time::now();
	  pose_orientation_gripper.header.frame_id = gripper_grasp_frame_id;
	  pose_orientation_gripper = buffer_.transform(pose_orientation_gripper, robot_reference_frame, ros::Duration(1.0));

	  set_initial_planning_configuration(initial_joint_position, joint_names);

	  geometry_msgs::PoseStamped pose_position_gripper;
	  pose_position_gripper.pose.orientation.w = 1.0;
	  pose_position_gripper.header.stamp = ros::Time::now();
	  pose_position_gripper.header.frame_id = gripper_grasp_frame_id;
	  pose_position_gripper = buffer_.transform(pose_position_gripper, robot_reference_frame, ros::Duration(1.0));

	  geometry_msgs::PoseStamped pose_gripper = pose_position_gripper;
	  pose_gripper.pose.orientation = pose_orientation_gripper.pose.orientation;

	  return pose_gripper;
  }

  void print_collision_state()
  {
	auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
	planning_scene_monitor->requestPlanningSceneState();

	// Prepare structure for grasp attached collision object and pivoting attached one
	planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);

	std::vector<moveit_msgs::AttachedCollisionObject> att_col_objs;
	planning_scene->getAttachedCollisionObjectMsgs(att_col_objs);
	std::vector<moveit_msgs::CollisionObject> col_objs;
	planning_scene->getCollisionObjectMsgs(col_objs);

	cout << "========PRINT DBG=========" << endl;
	cout << "_____Attached Collision OBJs____"<< endl;
	for(const auto& obj : att_col_objs)
		cout << obj << endl << "------------------" << endl;
	cout << "_______Collision OBJs________"<< endl;
	for(const auto& obj : col_objs)
		cout << obj << endl << "------------------" << endl;
	cout << "========PRINT DBG END=========" << endl;

  }

  void detachCollisionObject(const std::string& attached_object_id)
  {
	auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
	planning_scene_monitor->requestPlanningSceneState();
	planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);

	moveit_msgs::AttachedCollisionObject attached_obj;
	if(!planning_scene->getAttachedCollisionObjectMsg(attached_obj, attached_object_id))
	{
		ROS_ERROR("detachCollisionObject unable tu find attached object id");
	}

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//Remove from attached
	attached_obj.object.operation = moveit_msgs::AttachedCollisionObject::_object_type::REMOVE;
	planning_scene_interface.applyAttachedCollisionObject(attached_obj);

	planning_scene_monitor->requestPlanningSceneState();
	moveit_msgs::CollisionObject collision_obj;
	if(!planning_scene->getCollisionObjectMsg(collision_obj, attached_object_id))
	{
		ROS_ERROR("detachCollisionObject unable tu find attached object id");
	}

	moveit_msgs::CollisionObject new_obj = attached_obj.object;
	new_obj.primitive_poses = collision_obj.primitive_poses;

	for( const geometry_msgs::Pose& sub_pose : attached_obj.subframe_poses )
	{
		//TRANSFORM!!
	}

	// Change frame id!!

	planning_scene_interface.applyCollisionObject(attached_obj.object);



  }

  void changeObjectAttachState(
	  moveit_msgs::AttachedCollisionObject obj_to_detach, 
	  moveit_msgs::AttachedCollisionObject obj_to_attach
	  )
  {
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	char ans;
	cout << "changeObjectAttachState init [button]" << endl; cin >> ans;

	print_collision_state();

	cout << "changeObjectAttachState remove init [button]" << endl; cin >> ans;

	obj_to_detach.object.operation = moveit_msgs::AttachedCollisionObject::_object_type::REMOVE;
	planning_scene_interface.applyAttachedCollisionObject(obj_to_detach);

	print_collision_state();

	cout << "changeObjectAttachState remove done [button]" << endl; cin >> ans;

	cout << "changeObjectAttachState attach init [button]" << endl; cin >> ans;

	obj_to_attach.object.operation = moveit_msgs::AttachedCollisionObject::_object_type::ADD;
	planning_scene_interface.applyAttachedCollisionObject(obj_to_attach);

	print_collision_state();

	cout << "changeObjectAttachState attach done [button]" << endl; cin >> ans;

  }

  void executePlanning(const sun_pivoting_planner_msgs::PivotingPlanGoalConstPtr &goal)
  {

	/* Bring Simulated Robot to the initial configuration */
	char ans;
	cout << "setto la configurazione iniziale [button]" << endl; cin >> ans;
	set_initial_planning_configuration(goal->start_config, goal->start_config_joint_names);
	cout << "configurazione iniziale settata [button]" << endl; cin >> ans;

	moveit::planning_interface::MoveGroupInterface move_group_arm(goal->group_arm_name);

	/* Try the standard plan first */
	ROS_INFO("Try using standard planner");
	trajectory_msgs::JointTrajectory planned_traj;

	cout << "Pianifico standard [button]" << endl; cin >> ans;
    move_group_arm.setEndEffectorLink(goal->end_effector_frame_id);
	if(plan(move_group_arm, goal->target_pose, goal->path_constraints, planned_traj, false))
	{
		set_initial_planning_configuration(planned_traj.points.back().positions, planned_traj.joint_names);
		sun_pivoting_planner_msgs::PivotingPlanResult res;
		res.planned_trajectories.push_back(planned_traj);
		res.pivoting_mode.push_back(false);
		ROS_INFO("Solution found in standard mode");
		as_.setSucceeded(res);
		return;
	}
	cout << "pianifico standard fallito [button]" << endl; cin >> ans;

	ROS_INFO("Solution NOT found in standard mode");

	if(!goal->activate_pivoting)
	{
		ROS_INFO("Pivoting Mode not activated");
		as_.setAborted();
		return;
	}

	// ICRA2020 plan
	ROS_INFO("Pivoting Mode activated");

	// char ans;
	// cout << "ICRA2020 plan [button]" << endl; cin >> ans;

	

	moveit::planning_interface::MoveGroupInterface move_group_pivoting(goal->group_arm_pivoting_name);

	// Fetch the current planning scene state once
	cout << "get attached obj [button]" << endl; cin >> ans;
	auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
	planning_scene_monitor->requestPlanningSceneState();

	// Prepare structure for grasp attached collision object and pivoting attached one
	planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);
	moveit_msgs::AttachedCollisionObject grasp_attached_collision_obj;
	if(!planning_scene->getAttachedCollisionObjectMsg(grasp_attached_collision_obj, goal->attached_object_id))
	{
		ROS_ERROR_STREAM("Unable to find attached object with id " << goal->attached_object_id);
		as_.setAborted();
		return;
	}
	moveit_msgs::AttachedCollisionObject pivoting_attached_collision_obj = grasp_attached_collision_obj;
	pivoting_attached_collision_obj.link_name = move_group_pivoting.getLinkNames().back();

	cout << endl << "grasp_attached_collision_obj: " << endl << grasp_attached_collision_obj
	<< "==========" << endl << "pivoting_attached_collision_obj: " << pivoting_attached_collision_obj << endl;
	cout << "attached obj preso [button]" << endl; cin >> ans;

	cout << "change attached obj state [button]" << endl; cin >> ans;
	changeObjectAttachState(
	  grasp_attached_collision_obj, 
	  pivoting_attached_collision_obj
	);
	cout << "attached obj state changed [button]" << endl; cin >> ans;

	cout << "simulate pivoting [button]" << endl; cin >> ans;
	simulate_gravity_pivoting(
	  planning_scene_monitor, 
	  goal->attached_object_id + "/" + goal->cog_subframe,
	  move_group_pivoting.getJointNames().back(),
	  move_group_pivoting.getCurrentJointValues().back()
	);
	cout << "pivoting simulated [button]" << endl; cin >> ans;

	cout << "plan fake pivoting motion [button]" << endl; cin >> ans;
    move_group_pivoting.setEndEffectorLink(goal->end_effector_frame_id);
	if(!plan(move_group_pivoting, goal->target_pose, goal->path_constraints, planned_traj))
	{
		ROS_INFO("Fail to plan the fake pivoting trajectory");
		as_.setAborted();
		return;
	}
	cout << "fake pivoting planned [button]" << endl; cin >> ans;

	std::vector<string> fake_traj_joint_names = planned_traj.joint_names;
	std::vector<double> post_pivoting_joint_position = planned_traj.points.front().positions;
	std::vector<double> fake_traj_final_joint_position = planned_traj.points.back().positions;

	sun_pivoting_planner_msgs::PivotingPlanResult res;

	moveit_msgs::Constraints pivoting_fixed_position_constraint = goal->path_constraints;
	add_pivoting_constraints(pivoting_fixed_position_constraint, move_group_pivoting.getLinkNames().back(), move_group_pivoting.getPoseReferenceFrame());

	cout << "Constraints: " << pivoting_fixed_position_constraint << endl;

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
		// set_initial_planning_configuration(planned_traj.points.back().positions, planned_traj.joint_names);
	}

	cout << "computing p_p [button]" << endl; cin >> ans;
	geometry_msgs::PoseStamped p_p = compute_after_pivoting_pose(
		fake_traj_joint_names,
		post_pivoting_joint_position,
		fake_traj_final_joint_position,
		move_group_arm.getLinkNames().back(),
		move_group_pivoting.getPoseReferenceFrame()
	);
cout << p_p << endl;
cout << "p_p computed [button]" << endl; cin >> ans;


	// TODO, is it possible to use move_group_pivoting here? In order to simulate the actual real robot motion

	cout << "change attach grasp state [button]" << endl; cin >> ans;
	changeObjectAttachState(
	  pivoting_attached_collision_obj,
	  grasp_attached_collision_obj
	);
	cout << "attach grasp state changed [button]" << endl; cin >> ans;

	cout << "plan p_p [button]" << endl; cin >> ans;
	move_group_arm.setEndEffectorLink(move_group_arm.getLinkNames().back());
	if(!plan(move_group_arm, p_p, pivoting_fixed_position_constraint, planned_traj, false))
	{
		ROS_INFO("Fail to plan in pivoting mode");
		as_.setAborted();
		return;
	}
	cout << "p_p planned [button]" << endl; cin >> ans;
	
	res.planned_trajectories.push_back(planned_traj);
	set_initial_planning_configuration(planned_traj.points.back().positions, planned_traj.joint_names);

	cout << "simulate pivoting [button]" << endl; cin >> ans;
	simulate_gravity_pivoting(
	  planning_scene_monitor, 
	  goal->attached_object_id + "/" + goal->cog_subframe,
	  move_group_pivoting.getJointNames().back(),
	  move_group_pivoting.getCurrentJointValues().back()
	);
	cout << "pivoting simulated [button]" << endl; cin >> ans;

	cout << "final standard plan [button]" << endl; cin >> ans;
    move_group_arm.setEndEffectorLink(goal->end_effector_frame_id);
	if(!plan(move_group_arm, goal->target_pose, goal->path_constraints, planned_traj, false))
	{
		ROS_INFO("Fail to plan in pivoting mode");
		as_.setAborted();
		return;
	}
	cout << "final standard plan done [button]" << endl; cin >> ans;
	res.planned_trajectories.push_back(planned_traj);
	set_initial_planning_configuration(planned_traj.points.back().positions, planned_traj.joint_names);


	cout << "END!" << endl;
	as_.setSucceeded();
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pivoting_planner_server");

  PivotingPlannerActionServer pivoting_planner("pivoting_plan_action");
  ros::spin();

  return 0;
}
