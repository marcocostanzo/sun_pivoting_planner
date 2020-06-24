#ifndef SUN_PIVOTING_PLANNER_UTILITY
#define SUN_PIVOTING_PLANNER_UTILITY

#include "trajectory_msgs/JointTrajectory.h"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "sun_pivoting_planner/exceptions.h"


namespace sun{

struct Joint_Conf_Constraint
  {
	std::vector<std::string> joint_names;
	std::vector<double> move_range;
  };

// DEBUG
  void print_collision_obj_state(
	  planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	  const std::string& robot_description_id = "robot_description"
	  )
  {
	planning_scene_monitor->requestPlanningSceneState();

	// Prepare structure for grasp attached collision object and pivoting attached one
	planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);

	std::vector<moveit_msgs::AttachedCollisionObject> att_col_objs;
	planning_scene_ro->getAttachedCollisionObjectMsgs(att_col_objs);
	std::vector<moveit_msgs::CollisionObject> col_objs;
	planning_scene_ro->getCollisionObjectMsgs(col_objs);

	 std::cout << "========PRINT DBG=========" << std::endl;
	 std::cout << "_____Attached Collision OBJs____"<< std::endl;
	for(const auto& obj : att_col_objs)
		 std::cout << obj << std::endl << "------------------" << std::endl;
	 std::cout << "_______Collision OBJs________"<< std::endl;
	for(const auto& obj : col_objs)
		 std::cout << obj << std::endl << "------------------" << std::endl;
	 std::cout << "========PRINT DBG END=========" << std::endl;

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

  void add_joint_configuration_constraints(
	  const Joint_Conf_Constraint& joint_conf_constraint,
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

  // Return the link to which the object was attached
  std::string detachCollisionObject(
	  planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	  const std::string& attached_object_id, 
	  const std::shared_ptr<tf2_ros::Buffer>& tf2_buffer,
	  const std::string& robot_description_id = "robot_description")
  {

	  //DBG
	  char ans;
	  std::cout << "detachCollisionObject init [button]" << std::endl;std::cin >> ans;
	  std::cout << "detachCollisionObject print state: [button]" << std::endl;std::cin >> ans;
	  print_collision_obj_state(planning_scene_monitor);
	  std::cout << "detachCollisionObject state printed [button]" << std::endl;std::cin >> ans;

	//DBG
	std::cout << "detachCollisionObject requestPlanningSceneState [button]" << std::endl;std::cin >> ans;

	  planning_scene_monitor->requestPlanningSceneState();

	//DBG
	std::cout << "detachCollisionObject requestPlanningSceneState DONE [button]" << std::endl;std::cin >> ans;

		moveit_msgs::AttachedCollisionObject attached_obj;
	  std::string link_was_attached;

	  { //Scope LockedPlanningSceneRO
	  planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);

	//DBG
	std::cout << "detachCollisionObject request AttachedCollisionObject [button]" << std::endl;std::cin >> ans;

	if(!planning_scene_ro->getAttachedCollisionObjectMsg(attached_obj, attached_object_id))
	{
		ROS_ERROR("detachCollisionObject unable to find attached object id");
		throw attached_collision_object_not_found("detachCollisionObject unable to find attached object id");
	}
	} //END Scope LockedPlanningSceneRO

	link_was_attached = attached_obj.link_name;

	//DBG
	std::cout << "detachCollisionObject link_was_attached: " << link_was_attached << std::endl;
	std::cout << "detachCollisionObject requested AttachedCollisionObject [button]" << std::endl;std::cin >> ans;

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//DBG
	std::cout << "detachCollisionObject removing from attached [button]" << std::endl;std::cin >> ans;

	//Remove from attached
	attached_obj.object.operation = moveit_msgs::AttachedCollisionObject::_object_type::REMOVE;
	planning_scene_interface.applyAttachedCollisionObject(attached_obj);

	//DBG
	print_collision_obj_state(planning_scene_monitor);
	std::cout << "detachCollisionObject removed from attached [button]" << std::endl;std::cin >> ans;

	//DBG
	std::cout << "detachCollisionObject getting Collision obj [button]" << std::endl;std::cin >> ans;

	//DBG
	std::cout << "detachCollisionObject requestPlanningSceneState [button]" << std::endl;std::cin >> ans;

	planning_scene_monitor->requestPlanningSceneState();

	//DBG
	std::cout << "detachCollisionObject requestPlanningSceneState done [button]" << std::endl;std::cin >> ans;

	moveit_msgs::CollisionObject collision_obj;
	{ //Scope LockedPlanningSceneRO
	planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);
	if(!planning_scene_ro->getCollisionObjectMsg(collision_obj, attached_object_id))
	{
		ROS_ERROR("detachCollisionObject unable to find collision object id");
		throw collision_object_not_found("detachCollisionObject unable to find collision object id just detached");
	}
	} //END Scope LockedPlanningSceneRO

	//DBG
	std::cout << "detachCollisionObject collision obj get done [button]" << std::endl;std::cin >> ans;

	//DBG
	std::cout << "detachCollisionObject transforming subframes [button]" << std::endl;std::cin >> ans;

	moveit_msgs::CollisionObject new_obj = attached_obj.object;
	new_obj.primitive_poses = collision_obj.primitive_poses;

	ros::Time time_now = ros::Time::now();
	for( geometry_msgs::Pose& sub_pose : new_obj.subframe_poses )
	{
		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.pose = sub_pose;
		pose_stamped.header.frame_id = new_obj.header.frame_id;
		pose_stamped.header.stamp = time_now;
		tf2_buffer->transform(pose_stamped, pose_stamped, collision_obj.header.frame_id, ros::Duration(1.0));
		sub_pose = pose_stamped.pose;
	}

	// Change frame id!!
	new_obj.header.frame_id = collision_obj.header.frame_id;

	//DBG
	std::cout << "detachCollisionObject transform done [button]" << std::endl;std::cin >> ans;

	//DBG
	std::cout << "detachCollisionObject applying collision obj [button]" << std::endl;std::cin >> ans;

	new_obj.operation = moveit_msgs::AttachedCollisionObject::_object_type::ADD;
	planning_scene_interface.applyCollisionObject(new_obj);

	//DBG
	print_collision_obj_state(planning_scene_monitor);
	std::cout << "detachCollisionObject collision obj apply done [button]" << std::endl;std::cin >> ans;

	return link_was_attached;

  }

  void attachCollisionObject(
	  const std::string& object_id, 
	  const std::string& link_name
	)
	{
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::AttachedCollisionObject att_coll_object;
    att_coll_object.object.id = object_id;
    att_coll_object.link_name = link_name;
    att_coll_object.object.operation = att_coll_object.object.ADD;
    planning_scene_interface.applyAttachedCollisionObject(att_coll_object);
	}

} // namespace sun

#endif