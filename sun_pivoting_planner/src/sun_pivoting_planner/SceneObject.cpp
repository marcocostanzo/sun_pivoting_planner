
#include "sun_pivoting_planner/SceneObject.h"

namespace sun
{
std::string SceneObject::getBaseFrameID()
{
  return YAML::LoadFile(db + type + ".yaml")["base_frame_id"].as<std::string>();
}

void SceneObject::updateFromPlanningScene(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  fromPlanningScene(planning_scene_monitor, id);
}

void SceneObject::fromPlanningScene(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                    const std::string& object_id)
{
  moveit_msgs::CollisionObject collision_obj =
      getMoveitPossiblyAttachedCollisionObject(planning_scene_monitor, object_id, " - in getSceneObjectFromScene");

  db = collision_obj.type.db;
  id = collision_obj.id;
  type = collision_obj.type.key;
  db = collision_obj.type.db;
  pose = getCollisionObjectSubframePose(planning_scene_monitor, id, getBaseFrameID());
}

std::string SceneObject::getBaseFrameID() const
{
  return YAML::LoadFile(db + type + ".yaml")["base_frame_id"].as<std::string>();
}

geometry_msgs::PoseStamped SceneObject::getSubframePose(
    planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& subframe_name) const
{
  return getCollisionObjectSubframePose(planning_scene_monitor, id, subframe_name);
}

//! Spawn a new collision object such that the object subframe 'ref_subframe_name' is located in 'ref_subframe_pose'
void SceneObject::spawn(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor) const
{
  moveit_msgs::CollisionObject collision_obj;

  collision_obj.header.frame_id = pose.header.frame_id;
  collision_obj.id = id;
  collision_obj.type.key = type;
  collision_obj.type.db = db;

  YAML::Node obj_yaml = YAML::LoadFile(db + type + ".yaml");

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
  moveCollisionObject(planning_scene_monitor, id, pose, obj_yaml["base_frame_id"].as<std::string>());
}

// Remove the object from the planning scene
void SceneObject::remove(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  removeCollisionObject(planning_scene_monitor, id);
}

//! move the object such that the base frame goes to the stored pose
void SceneObject::move(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  moveCollisionObject(planning_scene_monitor, id, pose, getBaseFrameID());
}

//! move the object such that the base frame goes to the new_pose
void SceneObject::move(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                       const geometry_msgs::PoseStamped& new_pose)
{
  pose = new_pose;
  move(planning_scene_monitor);
}

//! move the object such that the frame subframe_name goes to the new_pose
void SceneObject::move(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                       const geometry_msgs::PoseStamped& new_pose, const std::string& subframe_name)
{
  setPose(new_pose, subframe_name);
  move(planning_scene_monitor);
}

// set the pose of the base frame, N.B. this does NOT update the planning scene
void SceneObject::setPose(const geometry_msgs::PoseStamped& new_pose)
{
  pose = new_pose;
}

// set the pose of the base_frame, the input is the pose of the subframe_name frame, N.B. this does NOT update the
// planning scene
void SceneObject::setPose(const geometry_msgs::PoseStamped& new_pose, const std::string& subframe_name)
{
  Eigen::Affine3d b_T_s;  // subframe w.r.t. base_frame
  tf2::fromMsg(getSubframeRelativePose(subframe_name), b_T_s);

  Eigen::Affine3d w_T_sf;  // desired subframe w.r.t. a world frame
  tf2::fromMsg(new_pose.pose, w_T_sf);

  // desired base_frame w.r.t. the same world frame
  Eigen::Affine3d w_T_bf = w_T_sf * (b_T_s.inverse());

  geometry_msgs::PoseStamped pose_bf;
  pose_bf.pose = tf2::toMsg(w_T_bf);
  pose_bf.header.frame_id = new_pose.header.frame_id;
  pose = pose_bf;
}

//! Get the subframe pose relative to the base_frame_id
geometry_msgs::Pose SceneObject::getSubframeRelativePose(const std::string& subframe_name)
{
  geometry_msgs::Pose pose;
  YAML::Node obj_yaml = YAML::LoadFile(db + type + ".yaml");
  if (subframe_name == obj_yaml["base_frame_id"].as<std::string>())
  {
    pose.orientation.w = 1.0;
    return pose;
  }
  if (!obj_yaml["subframes"][subframe_name])
  {
    throw subframe_not_found("getSubframeRelativePose, subframe " + subframe_name + " not found in yaml file");
  }

  return getPoseFromYAMLNode(obj_yaml["subframes"][subframe_name]);
}

Eigen::Affine3d SceneObject::getRelativeSubframeAffine3d(const std::string& source_subframe,
                                                         const std::string& target_subframe)
{
  geometry_msgs::Pose base_pose_source = getSubframeRelativePose(source_subframe);
  geometry_msgs::Pose base_pose_target = getSubframeRelativePose(target_subframe);

  Eigen::Affine3d base_T_source;
  tf2::fromMsg(base_pose_source, base_T_source);
  Eigen::Affine3d base_T_target;
  tf2::fromMsg(base_pose_target, base_T_target);

  Eigen::Affine3d source_T_target = base_T_source.inverse() * base_T_target;

  return source_T_target;
}

geometry_msgs::PoseStamped
SceneObject::getSubframePoseFromKnownSubframe(const std::string& subframe_id, const std::string& known_subframe_id,
                                              const geometry_msgs::PoseStamped& known_subframe_pose)
{
  Eigen::Affine3d known_T_target = getRelativeSubframeAffine3d(known_subframe_id, subframe_id);
  Eigen::Affine3d w_T_known;
  tf2::fromMsg(known_subframe_pose.pose, w_T_known);

  Eigen::Affine3d w_T_target = w_T_known * known_T_target;

  geometry_msgs::PoseStamped target_pose;
  target_pose.pose = tf2::toMsg(w_T_target);

  target_pose.header = known_subframe_pose.header;

  return target_pose;
}

geometry_msgs::Point getPositionFromYAMLNode(const YAML::Node& yaml)
{
  geometry_msgs::Point position;
  position.x = yaml["position"].as<std::vector<double>>()[0];
  position.y = yaml["position"].as<std::vector<double>>()[1];
  position.z = yaml["position"].as<std::vector<double>>()[2];
  return position;
}

geometry_msgs::Quaternion getOrientationFromYAMLNode(const YAML::Node& yaml)
{
  geometry_msgs::Quaternion orientation;
  orientation.w = yaml["orientation"]["scalar"].as<double>();
  orientation.x = yaml["orientation"]["vector"].as<std::vector<double>>()[0];
  orientation.y = yaml["orientation"]["vector"].as<std::vector<double>>()[1];
  orientation.z = yaml["orientation"]["vector"].as<std::vector<double>>()[2];
  return orientation;
}

geometry_msgs::Pose getPoseFromYAMLNode(const YAML::Node& yaml)
{
  geometry_msgs::Pose pose;
  pose.position = getPositionFromYAMLNode(yaml);
  pose.orientation = getOrientationFromYAMLNode(yaml);
  return pose;
}

void spawnSceneObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                       const std::vector<SceneObject>& objs)
{
  for (const auto& obj : objs)
  {
    obj.spawn(planning_scene_monitor);
  }
}

void removeSceneObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                        std::vector<SceneObject>& objs)
{
  for (auto& obj : objs)
  {
    obj.remove(planning_scene_monitor);
  }
}

const SceneObject& getSceneObjectFromVector(const std::vector<SceneObject>& objs, const std::string& object_id,
                                            int& index)
{
  for (index = 0; index < objs.size(); index++)
  {
    const auto& obj = objs[index];
    if (obj.id == object_id)
    {
      return obj;
    }
  }
  throw collision_object_not_found("getSceneObject object_id " + object_id + " not found");
}

const SceneObject& getSceneObjectFromVector(const std::vector<SceneObject>& objs, const std::string& object_id)
{
  int index;
  return getSceneObjectFromVector(objs, object_id, index);
}

const SceneObject removeSceneObjectFromVector(std::vector<SceneObject>& objs, const std::string& object_id, int& index)
{
  for (index = 0; index < objs.size(); index++)
  {
    if (objs[index].id == object_id)
    {
      SceneObject obj = objs[index];
      objs.erase(objs.begin() + index);
      return obj;
    }
  }
  throw collision_object_not_found("getSceneObject object_id " + object_id + " not found");
}

const SceneObject removeSceneObjectFromVector(std::vector<SceneObject>& objs, const std::string& object_id)
{
  int index;
  return removeSceneObjectFromVector(objs, object_id, index);
}

}  // namespace sun
