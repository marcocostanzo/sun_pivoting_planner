#ifndef SUN_PIVOTING_PLANNER_UTILITY
#define SUN_PIVOTING_PLANNER_UTILITY

#include "trajectory_msgs/JointTrajectory.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "sun_pivoting_planner/exceptions.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_eigen/tf2_eigen.h"

#include "yaml-cpp/yaml.h"

namespace sun
{
struct Joint_Conf_Constraint
{
  std::vector<std::string> joint_names;
  std::vector<double> move_range;
};

// DEBUG
void print_collision_obj_state(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                               const std::string& robot_description_id);

double compute_traj_length(const trajectory_msgs::JointTrajectory& traj);

void add_joint_configuration_constraints(const Joint_Conf_Constraint& joint_conf_constraint,
                                         moveit::planning_interface::MoveGroupInterface& move_group,
                                         const moveit_msgs::Constraints& path_constraints_in,
                                         moveit_msgs::Constraints& path_constraints_out, double move_range_relax);

moveit_msgs::CollisionObject
getMoveitCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                         const std::string& object_id, const std::string& error_append = "");

moveit_msgs::AttachedCollisionObject
getMoveitAttachedCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                 const std::string& object_id, const std::string& error_append = "");

moveit_msgs::CollisionObject
getMoveitPossiblyAttachedCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                         const std::string& object_id, const std::string& error_append = "");

void applyMoveitCollisionObject(moveit_msgs::CollisionObject obj);

// Return the link to which the object was attached
std::string detachCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                  const std::string& attached_object_id, const std::string& robot_description_id);

void attachCollisionObject(const std::string& object_id, const std::string& link_name);

geometry_msgs::PoseStamped getCollisionObjectSubframePose(const moveit_msgs::CollisionObject& collision_obj,
                                                          const std::string& subframe_name);

geometry_msgs::PoseStamped
getCollisionObjectSubframePose(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                               const std::string& object_id, const std::string& subframe_name);

void moveCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                         const std::string& object_id, geometry_msgs::PoseStamped desired_pose,
                         const std::string& ref_subframe_name);

void removeCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                           const std::string& obj_id);

void removeAllCollisionObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

}  // namespace sun

#endif