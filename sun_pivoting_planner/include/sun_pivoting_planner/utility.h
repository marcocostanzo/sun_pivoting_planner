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

struct SceneObject
{
  std::string db;                                // database path
  std::string type;                              // object type
  std::string id;                                // object id
  geometry_msgs::PoseStamped ref_subframe_pose;  // ref_subframe pose
  std::string ref_subframe;
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

geometry_msgs::PoseStamped
getCollisionObjectSubframePose(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                               const std::string& object_id, const std::string& subframe_name);

void moveCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                         const std::string& object_id, geometry_msgs::PoseStamped desired_pose,
                         const std::string& ref_subframe_name);

void removeCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                           const std::string& obj_id);

void removeAllCollisionObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

geometry_msgs::Pose getPoseFromYAMLNode(const YAML::Node& yaml);

// Spawn a new collision object such that the object subframe 'ref_subframe_name' is located in 'pose'
void spawnCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                          const SceneObject& obj);

// Same as spawnCollisionObject but for multiple objects
void spawnCollisionObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                           const std::vector<SceneObject>& objs);

void changeObjectRefSubframe(SceneObject& obj, const std::string& new_ref_subframe);

}  // namespace sun

#endif