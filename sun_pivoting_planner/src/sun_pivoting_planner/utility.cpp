
#include "sun_pivoting_planner/utility.h"

namespace sun
{
// DEBUG
void print_collision_obj_state(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                               const std::string& robot_description_id)
{
  planning_scene_monitor->requestPlanningSceneState();

  // Prepare structure for grasp attached collision object and pivoting attached one
  planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);

  std::vector<moveit_msgs::AttachedCollisionObject> att_col_objs;
  planning_scene_ro->getAttachedCollisionObjectMsgs(att_col_objs);
  std::vector<moveit_msgs::CollisionObject> col_objs;
  planning_scene_ro->getCollisionObjectMsgs(col_objs);

  std::cout << "========PRINT DBG=========" << std::endl;
  std::cout << "_____Attached Collision OBJs____" << std::endl;
  for (const auto& obj : att_col_objs)
    std::cout << obj << std::endl << "------------------" << std::endl;
  std::cout << "_______Collision OBJs________" << std::endl;
  for (const auto& obj : col_objs)
    std::cout << obj << std::endl << "------------------" << std::endl;
  std::cout << "========PRINT DBG END=========" << std::endl;
}

double compute_traj_length(const trajectory_msgs::JointTrajectory& traj)
{
  double length = 0.0;
  for (int i = 1; i < traj.points.size(); i++)
  {
    double dist_local_square = 0.0;
    for (int j = 0; j < traj.joint_names.size() - 1; j++)
    {
      dist_local_square += pow(traj.points[i - 1].positions[j] - traj.points[i].positions[j], 2);
    }
    length += sqrt(dist_local_square);
  }
  return length;
}

void add_joint_configuration_constraints(const Joint_Conf_Constraint& joint_conf_constraint,
                                         moveit::planning_interface::MoveGroupInterface& move_group,
                                         const moveit_msgs::Constraints& path_constraints_in,
                                         moveit_msgs::Constraints& path_constraints_out, double move_range_relax)
{
  path_constraints_out = path_constraints_in;

  std::vector<double> current_joint = move_group.getCurrentJointValues();

  for (int j = 0; j < joint_conf_constraint.joint_names.size(); j++)
  {
    std::string j_name = joint_conf_constraint.joint_names[j];
    double j_move_range = move_range_relax * joint_conf_constraint.move_range[j];

    moveit_msgs::JointConstraint joint_constraint;
    joint_constraint.weight = 1.0;
    joint_constraint.joint_name = j_name;

    move_group.getJointNames();
    int j_index = -1;
    for (int i = 0; i < move_group.getJointNames().size(); i++)
    {
      if (move_group.getJointNames()[i] == j_name)
      {
        j_index = i;
        break;
      }
    }
    if (j_index == -1)
    {
      continue;
    }
    joint_constraint.position = current_joint[j_index];

    joint_constraint.tolerance_below = j_move_range / 2.0;
    joint_constraint.tolerance_above = j_move_range / 2.0;

    if ((joint_constraint.position - joint_constraint.tolerance_below) <
        move_group.getRobotModel()->getVariableBounds(j_name).min_position_)
    {
      joint_constraint.tolerance_below =
          fabs(joint_constraint.position - move_group.getRobotModel()->getVariableBounds(j_name).min_position_);
      joint_constraint.tolerance_above = fabs(move_group.getRobotModel()->getVariableBounds(j_name).min_position_ +
                                              j_move_range - joint_constraint.position);
    }

    if ((joint_constraint.position + joint_constraint.tolerance_above) >
        move_group.getRobotModel()->getVariableBounds(j_name).max_position_)
    {
      joint_constraint.tolerance_above =
          fabs(joint_constraint.position - move_group.getRobotModel()->getVariableBounds(j_name).max_position_);
      joint_constraint.tolerance_below = fabs(move_group.getRobotModel()->getVariableBounds(j_name).max_position_ -
                                              j_move_range - joint_constraint.position);
    }

#ifdef DBG_MSG
    std::cout << "Joint: " << j_name << "\n" << joint_constraint;
#endif

    path_constraints_out.joint_constraints.push_back(joint_constraint);
  }
}

moveit_msgs::CollisionObject
getMoveitCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                         const std::string& object_id, const std::string& error_append)
{
  planning_scene_monitor->requestPlanningSceneState();
  moveit_msgs::CollisionObject collision_obj;
  {  // Scope LockedPlanningSceneRO
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);
    if (!planning_scene_ro->getCollisionObjectMsg(collision_obj, object_id))
    {
      ROS_ERROR_STREAM("unable to find collision object id " << object_id << error_append);
      throw collision_object_not_found("unable to find collision object id " + object_id + error_append);
    }
  }  // END Scope LockedPlanningSceneRO
  return collision_obj;
}

moveit_msgs::AttachedCollisionObject
getMoveitAttachedCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                 const std::string& object_id, const std::string& error_append)
{
  planning_scene_monitor->requestPlanningSceneState();
  moveit_msgs::AttachedCollisionObject att_collision_obj;
  {  // Scope LockedPlanningSceneRO
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);
    if (!planning_scene_ro->getAttachedCollisionObjectMsg(att_collision_obj, object_id))
    {
      ROS_ERROR_STREAM("unable to find attached collision object id " << object_id << error_append);
      throw attached_collision_object_not_found("unable to find attached collision object id " + object_id +
                                                error_append);
    }
  }  // END Scope LockedPlanningSceneRO
  return att_collision_obj;
}

moveit_msgs::CollisionObject
getMoveitPossiblyAttachedCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                         const std::string& object_id, const std::string& error_append)
{
  try
  {
    return getMoveitCollisionObject(planning_scene_monitor, object_id, error_append);
  }
  catch (const collision_object_not_found& e)
  {
    moveit_msgs::AttachedCollisionObject att_coll_obj =
        getMoveitAttachedCollisionObject(planning_scene_monitor, object_id, error_append);
    return att_coll_obj.object;
  }
}

void applyMoveitCollisionObject(moveit_msgs::CollisionObject obj)
{
  obj.operation = moveit_msgs::CollisionObject::ADD;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(obj);
}

// Return the link to which the object was attached
std::string detachCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                  const std::string& attached_object_id, const std::string& robot_description_id)
{
  moveit_msgs::AttachedCollisionObject attached_obj =
      getMoveitAttachedCollisionObject(planning_scene_monitor, attached_object_id, " - in detachCollisionObject");
  std::string link_was_attached = attached_obj.link_name;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Remove from attached
  attached_obj.object.operation = moveit_msgs::AttachedCollisionObject::_object_type::REMOVE;
  planning_scene_interface.applyAttachedCollisionObject(attached_obj);

  planning_scene_monitor->requestPlanningSceneState();

  moveit_msgs::CollisionObject collision_obj = getMoveitCollisionObject(planning_scene_monitor, attached_object_id,
                                                                        " - in detachCollisionObject - unable to find "
                                                                        "collision object id just detached");

  moveit_msgs::CollisionObject new_obj = attached_obj.object;
  new_obj.primitive_poses = collision_obj.primitive_poses;

  ros::Time time_now = ros::Time::now();
  for (geometry_msgs::Pose& sub_pose : new_obj.subframe_poses)
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = sub_pose;
    pose_stamped.header.frame_id = new_obj.header.frame_id;
    pose_stamped.header.stamp = time_now;
    planning_scene_monitor->getTFClient()->transform(pose_stamped, pose_stamped, collision_obj.header.frame_id,
                                                     ros::Duration(1.0));
    sub_pose = pose_stamped.pose;
  }

  // Change frame id!!
  new_obj.header.frame_id = collision_obj.header.frame_id;

  new_obj.operation = moveit_msgs::AttachedCollisionObject::_object_type::ADD;
  planning_scene_interface.applyCollisionObject(new_obj);

  return link_was_attached;
}

void attachCollisionObject(const std::string& object_id, const std::string& link_name)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::AttachedCollisionObject att_coll_object;
  att_coll_object.object.id = object_id;
  att_coll_object.link_name = link_name;
  att_coll_object.object.operation = att_coll_object.object.ADD;
  planning_scene_interface.applyAttachedCollisionObject(att_coll_object);
}

geometry_msgs::PoseStamped getCollisionObjectSubframePose(const moveit_msgs::CollisionObject& collision_obj,
                                                          const std::string& subframe_name)
{
  for (int i = 0; i < collision_obj.subframe_names.size(); i++)
  {
    if (collision_obj.subframe_names[i] == subframe_name)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = collision_obj.header.frame_id;
      pose.pose = collision_obj.subframe_poses[i];
      return pose;
    }
  }

  throw subframe_not_found("getCollisionObjectSubframePose, subframe " + subframe_name + " not found");
}

geometry_msgs::PoseStamped
getCollisionObjectSubframePose(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                               const std::string& object_id, const std::string& subframe_name)
{
  moveit_msgs::CollisionObject collision_obj = getMoveitPossiblyAttachedCollisionObject(
      planning_scene_monitor, object_id, " - in getCollisionObjectSubframePose");

  return getCollisionObjectSubframePose(collision_obj, subframe_name);
}

void moveCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                         const std::string& object_id, geometry_msgs::PoseStamped desired_pose,
                         const std::string& ref_subframe_name)
{
  moveit_msgs::CollisionObject collision_obj =
      getMoveitCollisionObject(planning_scene_monitor, object_id, " - in moveCollisionObject");

  // Find the subframe
  geometry_msgs::PoseStamped ref_subframe = getCollisionObjectSubframePose(collision_obj, ref_subframe_name);

  // represent the desired pose in the object frame id
  desired_pose.header.stamp = ros::Time::now();
  planning_scene_monitor->getTFClient()->transform(desired_pose, desired_pose, collision_obj.header.frame_id,
                                                   ros::Duration(1.0));

  // To Eigen
  Eigen::Affine3d w_T_osf_i;  // 'world' T 'object subframe' _ 'initial'
  tf2::fromMsg(ref_subframe.pose, w_T_osf_i);
  Eigen::Affine3d w_T_osf_f;  // 'world' T 'object subframe' _ 'final'
  tf2::fromMsg(desired_pose.pose, w_T_osf_f);

  // Delta T, from initial to final
  Eigen::Affine3d f_T_i = w_T_osf_f * w_T_osf_i.inverse();

  // Convert all poses
  for (auto& primitive_pose : collision_obj.primitive_poses)
  {
    Eigen::Affine3d pose_i;
    tf2::fromMsg(primitive_pose, pose_i);
    Eigen::Affine3d pose_f = f_T_i * pose_i;
    primitive_pose = tf2::toMsg(pose_f);
  }
  // Convert all subframes
  for (geometry_msgs::Pose& primitive_pose : collision_obj.subframe_poses)
  {
    Eigen::Affine3d pose_i;
    tf2::fromMsg(primitive_pose, pose_i);
    Eigen::Affine3d pose_f = f_T_i * pose_i;
    primitive_pose = tf2::toMsg(pose_f);
  }

  // Update object
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_obj);
}

void removeCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                           const std::string& obj_id)
{
  moveit_msgs::CollisionObject collision_obj =
      getMoveitCollisionObject(planning_scene_monitor, obj_id, " - in removeCollisionObject");
  collision_obj.operation = moveit_msgs::CollisionObject::REMOVE;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_obj);
}

void removeAllCollisionObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  // Get Attached collision objects
  planning_scene_monitor->requestPlanningSceneState();
  std::vector<moveit_msgs::AttachedCollisionObject> att_col_objs;
  {
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);
    planning_scene_ro->getAttachedCollisionObjectMsgs(att_col_objs);
  }

  // Detach all
  for (auto& att_col_obj : att_col_objs)
  {
    att_col_obj.object.operation = moveit_msgs::AttachedCollisionObject::_object_type::REMOVE;
  }
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyAttachedCollisionObjects(att_col_objs);
  }

  // Get Collision objects
  planning_scene_monitor->requestPlanningSceneState();
  std::vector<moveit_msgs::CollisionObject> col_objs;
  {
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);
    planning_scene_ro->getCollisionObjectMsgs(col_objs);
  }
  // Remove all
  for (auto& col_obj : col_objs)
  {
    col_obj.operation = moveit_msgs::AttachedCollisionObject::_object_type::REMOVE;
  }
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects(col_objs);
  }
}

}  // namespace sun
