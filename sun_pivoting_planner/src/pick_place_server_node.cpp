#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <sun_pivoting_planner_msgs/PickPlacePlanAction.h>
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

#include "yaml-cpp/yaml.h"

namespace sun
{
class PickPlacePlannerActionServer
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sun_pivoting_planner_msgs::PickPlacePlanAction> as_;  // NodeHandle instance must be
																					  // created before this line.

  actionlib::SimpleActionClient<sun_pivoting_planner_msgs::PivotingPlanAction> ac_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  tf2_ros::TransformListener tf2_transform_listener_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

public:
  PickPlacePlannerActionServer(const std::string& name, const std::string& pivoting_plan_action_name,
							   const ros::NodeHandle& nh = ros::NodeHandle())
	: nh_(nh)
	, as_(nh_, name, boost::bind(&PickPlacePlannerActionServer::executePlanning, this, _1), false)
	, ac_(pivoting_plan_action_name, true)
	, tf2_buffer_(new tf2_ros::Buffer())
	, tf2_transform_listener_(*tf2_buffer_.get())
	, planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf2_buffer_))
  {
	ac_.waitForServer();
	as_.start();
  }

  ~PickPlacePlannerActionServer(void)
  {
  }

  void executePlanning(const sun_pivoting_planner_msgs::PickPlacePlanGoalConstPtr& goal)
  {

	// Get the collision object initial state  
	moveit_msgs::CollisionObject collision_obj_initial_state;
	{  // Scope LockedPlanningSceneRO
	  planning_scene_monitor_->requestPlanningSceneState();
	  planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor_);
	  if (!planning_scene_ro->getCollisionObjectMsg(collision_obj_initial_state, goal->object_id))
	  {
		ROS_ERROR_STREAM("executePlanning pick_place unable to find collision object id " << goal->object_id);
		as_.setAborted();
		return;
	  }
	}  // END Scope LockedPlanningSceneRO

	std::string db_folder = collision_obj_initial_state.type.db;
	std::string obj_type = collision_obj_initial_state.type.key;

	YAML::Node obj_yaml;
	try
	{
	  obj_yaml = YAML::LoadFile(db_folder + obj_type + ".yaml");
	}
	catch (YAML::BadFile err)
	{
	  ROS_ERROR_STREAM("" << err.what());
	  ROS_ERROR("Invalid object_type");
	  as_.setAborted();
	  return;
	}

	std::vector<std::string> grasp_frames_ids = obj_yaml["grasp_frames_ids"].as<std::vector<std::string>>();
	std::vector<std::string> pre_grasp_frames_ids = obj_yaml["pregrasp_frames_ids"].as<std::vector<std::string>>();
	std::string cog_frame_id = obj_yaml["cog_frame_id"].as<std::string>();

	// Test all possible grasps
	int i;
	for (i = 0; i < grasp_frames_ids.size(); i++)
	{
	  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	  planning_scene_interface.applyCollisionObject(collision_obj_initial_state);

	  ROS_INFO_STREAM("Planning using grasp #" << i+1 << "/" << grasp_frames_ids.size());
	  ROS_INFO_STREAM(grasp_frames_ids[i] << " " << pre_grasp_frames_ids[i]);

	  sun_pivoting_planner_msgs::PickPlacePlanResult result;

	  // Generic Goal
	  sun_pivoting_planner_msgs::PivotingPlanGoal plannerGoal;
	  plannerGoal.group_arm_name = goal->group_arm_name;
	  plannerGoal.path_constraints = goal->path_constraints;
	  plannerGoal.attached_object_id = goal->object_id;
	  plannerGoal.cog_subframe = cog_frame_id;
	  plannerGoal.group_arm_pivoting_name = goal->group_arm_pivoting_name;

	  //* Pre Grasp *//
	  ROS_INFO_STREAM("Pre Grasp");

	  plannerGoal.start_config = goal->start_config;
	  plannerGoal.start_config_joint_names = goal->start_config_joint_names;
	  plannerGoal.target_pose = geometry_msgs::PoseStamped();
	  plannerGoal.target_pose.header.frame_id = goal->object_id + "/" + pre_grasp_frames_ids[i];
	  plannerGoal.target_pose.pose.orientation.w = 1.0;
	  plannerGoal.end_effector_frame_id = goal->grasp_link;
	  plannerGoal.activate_pivoting = false;

	  ac_.sendGoalAndWait(plannerGoal);

	  if (!(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
	  {
		ROS_WARN_STREAM("Plan #" << i+1 << "/" << grasp_frames_ids.size() << " PreGrasp Fail");
		continue;
	  }

	  result.pre_grasp_traj = ac_.getResult()->planned_trajectories[0];

	  //* Grasp (avoid it?) *//
	  ROS_INFO_STREAM("Grasp");

	  plannerGoal.start_config = result.pre_grasp_traj.points.back().positions;
	  plannerGoal.start_config_joint_names = result.pre_grasp_traj.joint_names;
	  plannerGoal.target_pose = geometry_msgs::PoseStamped();
	  plannerGoal.target_pose.header.frame_id = goal->object_id + "/" + grasp_frames_ids[i];
	  plannerGoal.target_pose.pose.orientation.w = 1.0;
	  plannerGoal.end_effector_frame_id = goal->grasp_link;
	  plannerGoal.activate_pivoting = false;

	  ac_.sendGoalAndWait(plannerGoal);

	  if (!(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
	  {
		ROS_WARN_STREAM("Plan #" << i+1 << "/" << grasp_frames_ids.size() << " Grasp Fail");
		continue;
	  }

	  //* Attach the object *//
	  ROS_INFO_STREAM("Attach");
	  // First teleport the object in the fingers
	  geometry_msgs::PoseStamped pose_move;
	  pose_move.header.frame_id = goal->grasp_link;
	  pose_move.pose.orientation.w = 1.0;
	  moveCollisionObject(planning_scene_monitor_, goal->object_id, pose_move, grasp_frames_ids[i]);
	  // zeroPivotingLink(); //TODO
	  attachCollisionObject(goal->object_id, goal->grasp_link);

	  if (goal->pre_place_endeffector != "")
	  {
		//* Pre Place *//
		ROS_INFO_STREAM("Pre Place");
		plannerGoal.start_config = result.pre_grasp_traj.points.back().positions;
		plannerGoal.start_config_joint_names = result.pre_grasp_traj.joint_names;
		plannerGoal.target_pose = goal->pre_place_pose;
		plannerGoal.end_effector_frame_id = goal->pre_place_endeffector;
		plannerGoal.activate_pivoting = goal->activate_pivoting;

		ac_.sendGoalAndWait(plannerGoal);

		if (!(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
		{
		  detachCollisionObject(planning_scene_monitor_, goal->object_id, "robot_description");
		  ROS_WARN_STREAM("Plan #" << i+1 << "/" << grasp_frames_ids.size() << " PrePlace Fail");
		  continue;
		}

		result.pre_place_traj = ac_.getResult()->planned_trajectories;
		result.pre_place_pivoting_mode = ac_.getResult()->pivoting_mode;

	  }
	  else
	  {
		ROS_WARN("Preplace not present in query, The planner will skip the pre-place");
	  }

	  //* Place *//
	  ROS_INFO_STREAM("Place");
	  if (goal->pre_place_endeffector != "")
	  {
		plannerGoal.start_config = result.pre_place_traj.back().points.back().positions;
		plannerGoal.start_config_joint_names = result.pre_place_traj.back().joint_names;
	  }
	  else
	  {
		plannerGoal.start_config = result.pre_grasp_traj.points.back().positions;
		plannerGoal.start_config_joint_names = result.pre_grasp_traj.joint_names;
	  }
	  plannerGoal.target_pose = goal->place_pose;
	  plannerGoal.end_effector_frame_id = goal->place_endeffector;
	  plannerGoal.activate_pivoting = goal->activate_pivoting;

	  ac_.sendGoalAndWait(plannerGoal);

	  if (!(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
	  {
		detachCollisionObject(planning_scene_monitor_, goal->object_id, "robot_description");
		ROS_WARN_STREAM("Plan #" << i+1 << "/" << grasp_frames_ids.size() << " Place Fail");
		continue;
	  }

	  result.place_traj = ac_.getResult()->planned_trajectories;
	  result.place_pivoting_mode = ac_.getResult()->pivoting_mode;

	  ROS_INFO_STREAM("Trajectory found at #" << i+1 << "/" << grasp_frames_ids.size());

	  detachCollisionObject(planning_scene_monitor_, goal->object_id, "robot_description");
	  as_.setSucceeded(result);
	  return;
	}

	ROS_ERROR("Unable to find a pick-place solution");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	planning_scene_interface.applyCollisionObject(collision_obj_initial_state);

	as_.setAborted();
  }
};

}  // namespace sun

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pivoting_pick_place_planner_server");
  sun::PickPlacePlannerActionServer pivoting_planner("pivoting_pick_place_plan_action", "pivoting_plan_action");
  ros::spin();

  return 0;
}
