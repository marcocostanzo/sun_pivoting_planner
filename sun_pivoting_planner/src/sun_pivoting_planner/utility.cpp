
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

// Return the link to which the object was attached
std::string detachCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                  const std::string& attached_object_id, const std::string& robot_description_id)
{
// DBG
#ifdef DBG_BTN
  char ans;
  std::cout << "detachCollisionObject init [button]" << std::endl;
  std::cin >> ans;
  std::cout << "detachCollisionObject print state: [button]" << std::endl;
  std::cin >> ans;
  print_collision_obj_state(planning_scene_monitor);
  std::cout << "detachCollisionObject state printed [button]" << std::endl;
  std::cin >> ans;
#endif

// DBG
#ifdef DBG_BTN
  std::cout << "detachCollisionObject requestPlanningSceneState [button]" << std::endl;
  std::cin >> ans;
#endif

  planning_scene_monitor->requestPlanningSceneState();

// DBG
#ifdef DBG_BTN
  std::cout << "detachCollisionObject requestPlanningSceneState DONE [button]" << std::endl;
  std::cin >> ans;
#endif

  moveit_msgs::AttachedCollisionObject attached_obj;
  std::string link_was_attached;

  {  // Scope LockedPlanningSceneRO
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);

// DBG
#ifdef DBG_BTN
    std::cout << "detachCollisionObject request AttachedCollisionObject [button]" << std::endl;
    std::cin >> ans;
#endif

    if (!planning_scene_ro->getAttachedCollisionObjectMsg(attached_obj, attached_object_id))
    {
      ROS_ERROR("detachCollisionObject unable to find attached object id");
      throw attached_collision_object_not_found("detachCollisionObject unable to find attached object id");
    }
  }  // END Scope LockedPlanningSceneRO

  link_was_attached = attached_obj.link_name;

// DBG
#ifdef DBG_BTN
  std::cout << "detachCollisionObject link_was_attached: " << link_was_attached << std::endl;
  std::cout << "detachCollisionObject requested AttachedCollisionObject [button]" << std::endl;
  std::cin >> ans;
#endif

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

// DBG
#ifdef DBG_BTN
  std::cout << "detachCollisionObject removing from attached [button]" << std::endl;
  std::cin >> ans;
#endif

  // Remove from attached
  attached_obj.object.operation = moveit_msgs::AttachedCollisionObject::_object_type::REMOVE;
  planning_scene_interface.applyAttachedCollisionObject(attached_obj);

// DBG
#ifdef DBG_BTN
  print_collision_obj_state(planning_scene_monitor);
  std::cout << "detachCollisionObject removed from attached [button]" << std::endl;
  std::cin >> ans;

  // DBG
  std::cout << "detachCollisionObject getting Collision obj [button]" << std::endl;
  std::cin >> ans;

  // DBG
  std::cout << "detachCollisionObject requestPlanningSceneState [button]" << std::endl;
  std::cin >> ans;
#endif

  planning_scene_monitor->requestPlanningSceneState();

// DBG
#ifdef DBG_BTN
  std::cout << "detachCollisionObject requestPlanningSceneState done [button]" << std::endl;
  std::cin >> ans;
#endif

  moveit_msgs::CollisionObject collision_obj;
  {  // Scope LockedPlanningSceneRO
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);
    if (!planning_scene_ro->getCollisionObjectMsg(collision_obj, attached_object_id))
    {
      ROS_ERROR("detachCollisionObject unable to find collision object id");
      throw collision_object_not_found("detachCollisionObject unable to find collision object id just detached");
    }
  }  // END Scope LockedPlanningSceneRO

// DBG
#ifdef DBG_BTN
  std::cout << "detachCollisionObject collision obj get done [button]" << std::endl;
  std::cin >> ans;

  // DBG
  std::cout << "detachCollisionObject transforming subframes [button]" << std::endl;
  std::cin >> ans;
#endif

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

// DBG
#ifdef DBG_BTN
  std::cout << "detachCollisionObject transform done [button]" << std::endl;
  std::cin >> ans;

  // DBG
  std::cout << "detachCollisionObject applying collision obj [button]" << std::endl;
  std::cin >> ans;
#endif

  new_obj.operation = moveit_msgs::AttachedCollisionObject::_object_type::ADD;
  planning_scene_interface.applyCollisionObject(new_obj);

// DBG
#ifdef DBG_BTN
  print_collision_obj_state(planning_scene_monitor);
  std::cout << "detachCollisionObject collision obj apply done [button]" << std::endl;
  std::cin >> ans;
#endif

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

geometry_msgs::PoseStamped
getCollisionObjectSubframePose(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                               const std::string& object_id, const std::string& subframe_name)
{
  planning_scene_monitor->requestPlanningSceneState();
  moveit_msgs::CollisionObject collision_obj;
  {  // Scope LockedPlanningSceneRO
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);
    if (!planning_scene_ro->getCollisionObjectMsg(collision_obj, object_id))
    {
      moveit_msgs::AttachedCollisionObject att_collision_obj;
      if (!planning_scene_ro->getAttachedCollisionObjectMsg(att_collision_obj, object_id))
      {
        ROS_ERROR_STREAM("getCollisionObjectSubframePose unable to find collision object id" << object_id);
        throw collision_object_not_found("getCollisionObjectSubframePose unable to find collision object id"
                                         << object_id);
      }
      collision_obj = att_collision_obj.object;
    }
  }  // END Scope LockedPlanningSceneRO

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

void moveCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                         const std::string& object_id, geometry_msgs::PoseStamped desired_pose,
                         const std::string& ref_subframe_name)
{
// DBG
#ifdef DBG_BTN
  std::cout << "moveCollisionObject Initial state" << std::endl;
  print_collision_obj_state(planning_scene_monitor);
#endif

  planning_scene_monitor->requestPlanningSceneState();

  moveit_msgs::CollisionObject collision_obj;
  {  // Scope LockedPlanningSceneRO
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);
    if (!planning_scene_ro->getCollisionObjectMsg(collision_obj, object_id))
    {
      ROS_ERROR("moveCollisionObject unable to find collision object id");
      throw collision_object_not_found("moveCollisionObject unable to find collision object id");
    }
  }  // END Scope LockedPlanningSceneRO

  // Find the subframe
  geometry_msgs::PoseStamped ref_subframe;
  // for(const std::string& subframe_name : collision_obj.subframe_names)
  for (int i = 0; i < collision_obj.subframe_names.size(); i++)
  {
    if (collision_obj.subframe_names[i] == ref_subframe_name)
    {
      ref_subframe.header = collision_obj.header;
      ref_subframe.pose = collision_obj.subframe_poses[i];
      break;
    }
  }
  if (ref_subframe.header.frame_id == "")
  {
    throw subframe_not_found("moveCollisionObject unable to find the subframe" + ref_subframe_name);
  }

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

// DBG
#ifdef DBG_BTN
  std::cout << "moveCollisionObject Final" << std::endl;
  print_collision_obj_state(planning_scene_monitor);
#endif
}

void removeCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                           const std::string& obj_id)
{
  moveit_msgs::CollisionObject collision_obj;
  {  // Scope LockedPlanningSceneRO
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_ro(planning_scene_monitor);
    if (!planning_scene_ro->getCollisionObjectMsg(collision_obj, obj_id))
    {
      ROS_ERROR_STREAM("removeCollisionObject unable to find collision object id" << obj_id);
      throw collision_object_not_found("removeCollisionObject unable to find collision object id" + obj_id);
    }
  }  // END Scope LockedPlanningSceneRO
  collision_obj.operation = moveit_msgs::CollisionObject::REMOVE;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_obj);
}

void removeAllCollisionObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  planning_scene_monitor->requestPlanningSceneState();

  // Get Attached collision objects
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

geometry_msgs::Pose getPoseFromYAMLNode(const YAML::Node& yaml)
{
  geometry_msgs::Pose pose;
  pose.position.x = yaml["position"].as<std::vector<double>>()[0];
  pose.position.y = yaml["position"].as<std::vector<double>>()[1];
  pose.position.z = yaml["position"].as<std::vector<double>>()[2];
  pose.orientation.w = yaml["orientation"]["scalar"].as<double>();
  pose.orientation.x = yaml["orientation"]["vector"].as<std::vector<double>>()[0];
  pose.orientation.y = yaml["orientation"]["vector"].as<std::vector<double>>()[1];
  pose.orientation.z = yaml["orientation"]["vector"].as<std::vector<double>>()[2];
}

// Spawn a new collision object such that the object subframe 'ref_subframe_name' is located in 'pose'
void spawnCollisionObject(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                          const SceneObject& obj)
{
  moveit_msgs::CollisionObject collision_obj;

  collision_obj.header.frame_id = obj.ref_subframe_pose.header.frame_id;
  collision_obj.id = obj.id;
  collision_obj.type.key = obj.type;
  collision_obj.type.db = obj.db;

  YAML::Node obj_yaml = YAML::LoadFile(obj.db + obj.type + ".yaml");

  // Geometry
  const YAML::Node& geometry_primitives = obj_yaml["geometry_primitives"];
  for (YAML::const_iterator it = geometry_primitives.begin(); it != geometry_primitives.end(); ++it)
  {
    const YAML::Node& geometry_primitive = *it;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = geometry_primitive["type"].as<int>();
    primitive.dimensions = geometry_primitive["dimensions"].as<std::vector<double>>();

    geometry_msgs::Pose primitive_pose = getPoseFromYAMLNode(geometry_primitive);

    collision_obj.primitives.push_back(primitive);
    collision_obj.primitive_poses.push_back(primitive_pose);
  }

  // Subframes
  collision_obj.subframe_names.push_back(obj_yaml["base_frame_id"].as<std::string>());
  collision_obj.subframe_poses.resize(1);
  collision_obj.subframe_poses[0].orientation.w = 1.0;
  if (obj_yaml["subframes"])
  {
    const YAML::Node& subframes = obj_yaml["subframes"];
    for (YAML::const_iterator it = subframes.begin(); it != subframes.end(); ++it)
    {
      geometry_msgs::Pose subframe_pose = getPoseFromYAMLNode(it->second);

      collision_obj.subframe_names.push_back(it->first.as<std::string>());
      collision_obj.subframe_poses.push_back(subframe_pose);
    }
  }

  // Add to the scene
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_obj);

  // Move to the desired location
  moveCollisionObject(planning_scene_monitor, obj.id, obj.ref_subframe_pose, obj.ref_subframe);
}

void spawnCollisionObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                           const std::vector<SceneObject>& objs)
{
  for (const auto& obj : objs)
  {
    spawnCollisionObject(planning_scene_monitor, obj);
  }
}

void changeObjectRefSubframe(SceneObject& obj, const std::string& new_ref_subframe)
{
  YAML::Node obj_yaml = YAML::LoadFile(obj.db + obj.type + ".yaml");

  Eigen::Affine3d w_T_i;
  tf2::fromMsg(obj.ref_subframe_pose.pose, w_T_i);

  Eigen::Affine3d b_T_i;
  if (obj.ref_subframe == obj_yaml["base_frame_id"].as<std::string>())
  {
    b_T_i = Eigen::Affine3d::Identity();
  }
  else
  {
    if (!obj_yaml["subframes"][obj.ref_subframe])
    {
      throw subframe_not_found("changeObjectRefSubframe, subframe " + obj.ref_subframe + " not found in yaml file");
    }
    tf2::fromMsg(getPoseFromYAMLNode(obj_yaml["subframes"][obj.ref_subframe]), b_T_i);

    // b_T_i = Eigen::Affine3d(
    //     Eigen::Translation3d(obj_yaml["subframes"][obj.ref_subframe]["position"].as<std::vector<double>>()[0],
    //                          obj_yaml["subframes"][obj.ref_subframe]["position"].as<std::vector<double>>()[1],
    //                          obj_yaml["subframes"][obj.ref_subframe]["position"].as<std::vector<double>>()[2]) *
    //     Eigen::Quaterniond(
    //         obj_yaml["subframes"][obj.ref_subframe]["orientation"]["scalar"].as<double>(),
    //         obj_yaml["subframes"][obj.ref_subframe]["orientation"]["vector"].as<std::vector<double>>()[0],
    //         obj_yaml["subframes"][obj.ref_subframe]["orientation"]["vector"].as<std::vector<double>>()[1],
    //         obj_yaml["subframes"][obj.ref_subframe]["orientation"]["vector"].as<std::vector<double>>()[2]));
  }

  Eigen::Affine3d b_T_f;
  if (new_ref_subframe == obj_yaml["base_frame_id"].as<std::string>())
  {
    b_T_f = Eigen::Affine3d::Identity();
  }
  else
  {
    if (!obj_yaml["subframes"][new_ref_subframe])
    {
      throw subframe_not_found("changeObjectRefSubframe, subframe " + new_ref_subframe + " not found in yaml file");
    }
    tf2::fromMsg(getPoseFromYAMLNode(obj_yaml["subframes"][new_ref_subframe]), b_T_f);
  }

  Eigen::Affine3d w_T_f = w_T_i * b_T_i.inverse() * b_T_f;

  obj.ref_subframe_pose.pose = tf2::toMsg(w_T_f);
  obj.ref_subframe = new_ref_subframe;
}

}  // namespace sun
