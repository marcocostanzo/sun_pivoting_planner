#ifndef SUN_PIVOTING_PLANNER_SCENEOBJECT
#define SUN_PIVOTING_PLANNER_SCENEOBJECT

#include "sun_pivoting_planner/utility.h"

namespace sun
{
class SceneObject
{
private:
  geometry_msgs::PoseStamped pose;  // this is the pose of base_frame_id in the yaml file

public:
  std::string db;    // database path
  std::string type;  // object type
  std::string id;    // object id

public:
  SceneObject()
  {
  }
  ~SceneObject() = default;

  std::string getBaseFrameID();

  void updateFromPlanningScene(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  void fromPlanningScene(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                         const std::string& object_id);

  std::string getBaseFrameID() const;

  geometry_msgs::PoseStamped getSubframePose(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                             const std::string& subframe_name) const;

  //! Spawn a new collision object such that the object subframe 'ref_subframe_name' is located in 'ref_subframe_pose'
  void spawn(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor) const;

  // Remove the object from the planning scene
  void remove(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  //! move the object such that the base frame goes to the stored pose
  void move(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  //! move the object such that the base frame goes to the new_pose
  void move(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
            const geometry_msgs::PoseStamped& new_pose);

  //! move the object such that the frame subframe_name goes to the new_pose
  void move(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
            const geometry_msgs::PoseStamped& new_pose, const std::string& subframe_name);

  // set the pose of the base frame, N.B. this does NOT update the planning scene
  void setPose(const geometry_msgs::PoseStamped& new_pose);

  // set the pose of the base_frame, the input is the pose of the subframe_name frame, N.B. this does NOT update the
  // planning scene
  void setPose(const geometry_msgs::PoseStamped& new_pose, const std::string& subframe_name);

  //! get Affine transform of subframe target_subframe w.r.t. source_subframe
  Eigen::Affine3d getRelativeSubframeAffine3d(const std::string& source_subframe, const std::string& target_subframe);

  geometry_msgs::PoseStamped getSubframePoseFromKnownSubframe(const std::string& subframe_id,
                                                              const std::string& known_subframe_id,
                                                              const geometry_msgs::PoseStamped& known_subframe_pose);

  //! Get the subframe pose relative to the base_frame_id
  geometry_msgs::Pose getSubframeRelativePose(const std::string& subframe_name);
};

geometry_msgs::Point getPositionFromYAMLNode(const YAML::Node& yaml);

geometry_msgs::Quaternion getOrientationFromYAMLNode(const YAML::Node& yaml);

geometry_msgs::Pose getPoseFromYAMLNode(const YAML::Node& yaml);

void spawnSceneObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                       const std::vector<SceneObject>& objs);

void removeSceneObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                        std::vector<SceneObject>& objs);

const SceneObject& getSceneObjectFromVector(const std::vector<SceneObject>& objs, const std::string& object_id,
                                            int& index);

const SceneObject& getSceneObjectFromVector(const std::vector<SceneObject>& objs, const std::string& object_id);

const SceneObject removeSceneObjectFromVector(std::vector<SceneObject>& objs, const std::string& object_id, int& index);

const SceneObject removeSceneObjectFromVector(std::vector<SceneObject>& objs, const std::string& object_id);

}  // namespace sun

#endif