#ifndef SUN_PIVOTING_PLANNER_UTILITY
#define SUN_PIVOTING_PLANNER_UTILITY

#include "trajectory_msgs/JointTrajectory.h"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "sun_pivoting_planner/exceptions.h"

#include "tf2_eigen/tf2_eigen.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
                               const std::string& robot_description_id = "robot_description");

double compute_traj_length(const trajectory_msgs::JointTrajectory& traj);

void add_joint_configuration_constraints(const Joint_Conf_Constraint& joint_conf_constraint,
                                         moveit::planning_interface::MoveGroupInterface& move_group,
                                         const moveit_msgs::Constraints& path_constraints_in,
                                         moveit_msgs::Constraints& path_constraints_out, double move_range_relax = 1.0);

// Return the link to which the object was attached
std::string detachCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                  const std::string& attached_object_id,
                                  const std::string& robot_description_id = "robot_description");

void attachCollisionObject(const std::string& object_id, const std::string& link_name);

void moveCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                         const std::string& object_id, geometry_msgs::PoseStamped desired_pose,
                         const std::string& ref_subframe_name);

// Spawn a new collision object such that the object subframe 'ref_subframe_name' is located in 'pose'
void spawnCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                          const std::string& object_id, const std::string& object_type, const std::string& db,
                          const geometry_msgs::PoseStamped& pose, const std::string& ref_subframe_name);

}  // namespace sun

#endif