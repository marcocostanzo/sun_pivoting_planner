#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sun_pivoting_planner_msgs/PivotingPlanAction.h>
#include <trajectory_msgs/JointTrajectory.h>

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
  // create messages that are used to published feedback/result
  sun_pivoting_planner_msgs::PivotingPlanFeedback feedback_;
  sun_pivoting_planner_msgs::PivotingPlanResult result_;
  std::vector<double> start_config;
  std::vector<double> computed_config; 
  std::vector<double> min_place_joint_conf;
  geometry_msgs::Pose pivoting_pose;
  geometry_msgs::Pose current_pose;
  geometry_msgs::Pose place_pose;

public:

  PivotingPlannerActionServer(std::string name) :
    as_(nh_, name, boost::bind(&PivotingPlannerActionServer::executePlanning, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~PivotingPlannerActionServer(void)
  {
  }

  bool IsConfigColliding(std::string planing_group_name,
			 std::vector<double>& joint2check)
  {
  	ROS_INFO("Check if the computed configuration is colliding with the planning scene");
  	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model)); 

  	robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  	const robot_model::JointModelGroup* joint_model_group = robot_state.getJointModelGroup(planing_group_name);
  	robot_state.setJointGroupPositions(joint_model_group, joint2check);
	bool is_colliding = planning_scene->isStateColliding( robot_state );

	if( !is_colliding )
  		ROS_INFO("COLLISION CHECK: joint2check NON E' IN COLLISIONE! :)");
	else
  		ROS_INFO("COLLISION CHECK: joint2check E' IN COLLISIONE! :(");
	return is_colliding;
  }

  double vecNorm(std::vector<double> const& v)
  {
    double temp = 0.0;
    for (int i = 0; i < v.size(); ++i) {
        temp += v[i] * v[i];
    }
    return sqrt(temp);
  }

  bool GetCorrectIK(std::string planing_group_name,
		    int num_joints,
  		    geometry_msgs::Pose target_pose, 
		    moveit_msgs::Constraints kinConstraints, 
		    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_,
		    bool PIVOTING_MODE_,
		    std::string link_ee_name,
		    std::string link_dummy_name)
  {

  	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  	planning_scene::PlanningScene* planning_scene(new planning_scene::PlanningScene(robot_model)); 

  	robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  	const robot_model::JointModelGroup* joint_model_group = robot_state.getJointModelGroup(planing_group_name);
  	robot_state::RobotState goal_state(robot_model);
  	goal_state.setJointGroupPositions(joint_model_group, start_config);
	goal_state.copyJointGroupPositions(joint_model_group, start_config);
        ROS_INFO("START JOINT CONFIGURATION ---> start_config.size() %d = num_joints %d", start_config.size(), num_joints);
        for(int s = 0; s < num_joints; s++)
		ROS_INFO("%f", 	start_config[s]);
	//soluzione CON collision-check e CON contraints
	robot_state::GroupStateValidityCallbackFn constraint_fn;
        planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
        planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
 	boost::scoped_ptr<kinematic_constraints::KinematicConstraintSet> kset;
	kset.reset(new kinematic_constraints::KinematicConstraintSet((ps)->getRobotModel()));
	kset->add(kinConstraints, (ps)->getTransforms());	
 	constraint_fn = boost::bind(&isStateValid, planning_scene, kset.get(), _1, _2, _3);
	
	bool found_ik = false;
	int maximum_iteration = 5;
	int count_iterations = 0;
	bool solution_found = true;

	if(PIVOTING_MODE_ == false)
	{
	  	while((int)found_ik != 1)
		{	
			found_ik = goal_state.setFromIK(joint_model_group, target_pose, 20, 0.05, constraint_fn);
			ROS_INFO("Solution found_ik (1:true - 0:false) = %d", (int)found_ik);
			count_iterations++;
			if (count_iterations >= maximum_iteration)
			{	
				solution_found = false;
				break;
			}
	 	}
		goal_state.copyJointGroupPositions(joint_model_group, computed_config);
		ROS_INFO("IK SOLUTION ---> computed_config.size() %d = num_joints %d", computed_config.size(), num_joints);
        	for(int s = 0; s < num_joints; s++)
			ROS_INFO("%f", 	computed_config[s]);	
	  	bool confIsColl = IsConfigColliding(planing_group_name, computed_config);
		if (confIsColl == true)
			solution_found = false;
	}
	else
	{
		double norm = 100.0;
		int NUM_ITER_ = 30;
		std::vector<double> diff;
		std::vector<double> computed_conf_tmp;

		diff.resize(num_joints-1);
		computed_conf_tmp.resize(num_joints);


		/*for(int i = 1; i < NUM_ITER_; i++)
		{
	  		goal_state.setJointGroupPositions(joint_model_group, start_config);
			found_ik = false;
	  		while((int)found_ik != 1)
			{
				found_ik = goal_state.setFromIK(joint_model_group, target_pose, 20, 0.05, constraint_fn);
				count_iterations++;
				if (count_iterations >= maximum_iteration)
				{	
					solution_found = false;
					break;
				}
		 	}
			goal_state.copyJointGroupPositions(joint_model_group, computed_conf_tmp);
			for(int n = 0; n < num_joints-1; n++)  //Computing norm difference (excluding the last augmented joint)...
				diff[n] = computed_conf_tmp[n] - start_config[n];
			double temp_norm = vecNorm(diff);
			ROS_INFO("---> |qi-qd|* {temp_norm1}: %f", temp_norm); 

			if (temp_norm < norm )
			{
				norm = temp_norm;
				computed_config = computed_conf_tmp;
				min_place_joint_conf = computed_conf_tmp;
			}
	   	}*/


		double temp_norm = 1000000;
	  	while(temp_norm >= 1.5)
		{	
	  		goal_state.setJointGroupPositions(joint_model_group, start_config);
			found_ik = false;
	  		while((int)found_ik != 1)
			{
				found_ik = goal_state.setFromIK(joint_model_group, target_pose, 20, 0.05, constraint_fn);
				count_iterations++;
				if (count_iterations >= maximum_iteration)
				{	
					solution_found = false;
					break;
				}
		 	}
			goal_state.copyJointGroupPositions(joint_model_group, computed_conf_tmp);
			for(int n = 0; n < num_joints-1; n++)  //Computing norm difference (excluding the last augmented joint)...
				diff[n] = computed_conf_tmp[n] - start_config[n];
			temp_norm = vecNorm(diff);
			ROS_INFO("---> |qi-qd|* {temp_norm1}: %f", temp_norm); 
			
	   	}
		norm = temp_norm;
		computed_config = computed_conf_tmp;
		min_place_joint_conf = computed_conf_tmp;

		ROS_INFO("Computed min_place_joint_conf. Min norm diff: %f. Joint values :", norm);
        	for(int s = 0; s < min_place_joint_conf.size(); s++)
			ROS_INFO("%f", 	min_place_joint_conf[s]);
	  	bool checkColl_min = IsConfigColliding(planing_group_name, min_place_joint_conf);

		//Compute the pivoting pose:
	  	goal_state.setJointGroupPositions(joint_model_group, min_place_joint_conf);

		// Print iiwa_link_ee pose. Remember that this is in the model frame
		const Eigen::Affine3d &end_effector_state = goal_state.getGlobalLinkTransform(link_ee_name);
		ROS_INFO_STREAM("Translation (end-effector): ");
		ROS_INFO_STREAM(end_effector_state.translation());
		ROS_INFO_STREAM("Rotation (end-effector): ");
		ROS_INFO_STREAM(end_effector_state.rotation());

		// Print iiwa_link_dummy pose. Remember that this is in the model frame
		const Eigen::Affine3d &dummy_state = goal_state.getGlobalLinkTransform(link_dummy_name);
		ROS_INFO_STREAM("Translation (dummy): ");
		ROS_INFO_STREAM(dummy_state.translation());
		ROS_INFO_STREAM("Rotation (dummy): ");
		ROS_INFO_STREAM(dummy_state.rotation());

		Eigen::Matrix3d rot = end_effector_state.rotation();
		Eigen::Quaterniond quat(rot);
		pivoting_pose.orientation.w = quat.w();
		pivoting_pose.orientation.x = quat.x();
		pivoting_pose.orientation.y = quat.y();
		pivoting_pose.orientation.z = quat.z();

		Eigen::Matrix3d place_rot = dummy_state.rotation();
		Eigen::Quaterniond place_quat(place_rot);
		place_pose.orientation.w = place_quat.w();
		place_pose.orientation.x = place_quat.x();
		place_pose.orientation.y = place_quat.y();
		place_pose.orientation.z = place_quat.z();
	  	place_pose.position.x = dummy_state.translation()[0];
	  	place_pose.position.y = dummy_state.translation()[1];
	  	place_pose.position.z = dummy_state.translation()[2];


	  	goal_state.setJointGroupPositions(joint_model_group, start_config);
		const Eigen::Affine3d &end_effector_current_state = goal_state.getGlobalLinkTransform(link_ee_name);
		ROS_INFO_STREAM("Position (end-effector CURRENT): ");
		ROS_INFO_STREAM(end_effector_current_state.translation());
	  	pivoting_pose.position.x = end_effector_current_state.translation()[0];
	  	pivoting_pose.position.y = end_effector_current_state.translation()[1];
	  	pivoting_pose.position.z = end_effector_current_state.translation()[2];

		Eigen::Matrix3d curr_rot = end_effector_current_state.rotation();
		Eigen::Quaterniond curr_quat(curr_rot);
		current_pose.orientation.w = curr_quat.w();
		current_pose.orientation.x = curr_quat.x();
		current_pose.orientation.y = curr_quat.y();
		current_pose.orientation.z = curr_quat.z();
	  	current_pose.position.x = end_effector_current_state.translation()[0];
	  	current_pose.position.y = end_effector_current_state.translation()[1];
	  	current_pose.position.z = end_effector_current_state.translation()[2];

		solution_found = true;
	 }

	return solution_found;
  }

  void executePlanning(const sun_pivoting_planner_msgs::PivotingPlanGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1000);
    bool success = true;

    // Init result action params
    result_.planning_code.val 				= 0;
    result_.execution_code.val 				= 0;
    result_.planned_trajectories.clear();
    result_.sequence.clear();

    // Init feedback action params
    feedback_.planning_success.val 			= 0;
    feedback_.planning_pivoting.val 			= 0;
    feedback_.execution_success.val 			= 0;
    feedback_.execution_pivoting.val 			= 0;

    // Obtaining the action-goal configuration parameters
    std::string group_name;
    int num_joints;
    geometry_msgs::Pose target_pose;
    moveit_msgs::Constraints path_constraints;
    bool activate_pivoting;
    std::string path_urdf_model;
    std::string path_urdf_augmented;
    std::string link_ee_name;
    std::string link_dummy_name;

    group_name 			= goal->group_name;
    num_joints 			= goal->num_joints;
    target_pose 		= goal->target_pose;
    path_constraints		= goal->path_constraints;
    activate_pivoting 		= goal->activate_pivoting;
    path_urdf_model 		= goal->path_urdf_model;
    path_urdf_augmented 	= goal->path_urdf_augmented;
    link_ee_name		= goal->link_ee_name;
    link_dummy_name		= goal->link_dummy_name;
    start_config.resize(goal->start_config.size());
    for(int p = 0; p < start_config.size(); p++)
	start_config[p] = goal->start_config[p];

    // Loading the robot URDF model and the augmented one
    std::ifstream f_model(path_urdf_model); 
    std::stringstream ss_model; 
    ss_model << f_model.rdbuf();
    std::ifstream f_augmented(path_urdf_augmented); 
    std::stringstream ss_augmented; 
    ss_augmented << f_augmented.rdbuf();

    min_place_joint_conf.resize(num_joints);
    for(int i = 0; i < num_joints; i++)
    {
	min_place_joint_conf[i] = 0.0;
    }

    //Preparing the planning scene to plan the path and the related planning_group
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_original(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    moveit::planning_interface::MoveGroupInterface group(group_name);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    group.setPlanningTime(60.0);
    ROS_INFO_NAMED("------->", "Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("------->", "End effector link: %s", group.getEndEffectorLink().c_str());

    // Publish info to the console for the user
    ROS_INFO("%s: Executing, planning with pivoting capabilities", action_name_.c_str());

    // Start executing the action
    robot_state::RobotState current_state(*group.getCurrentState());
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(group_name);
    std::vector<double> moveit_joint_conf; //conversion into MoveIt! msg:
    moveit_joint_conf.resize(num_joints);
    for(int i = 0; i < num_joints; i++)
	moveit_joint_conf[i] = start_config[i];
    ROS_INFO("setCurrentRobotState--->");
    for(int s = 0; s < num_joints; s++)
	ROS_INFO("%f", 	moveit_joint_conf[s]);
    current_state.setJointGroupPositions(joint_model_group, moveit_joint_conf);
    //group.setStartState(*group.getCurrentState());
    group.setStartState(current_state);

    int planning_mode = 0;
    bool ik_found = false;

  ros::Publisher pub_virtual_conf = nh_.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);	

    while(result_.execution_code.val != 1)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }


	switch (planning_mode)
	{
		case (0): // standard planning mode 
			ROS_INFO("CASE A - STANDARD planning mode ");
			nh_.setParam("robot_description", ss_model.str());
			ik_found = GetCorrectIK(group_name, num_joints, target_pose, path_constraints, planning_scene_monitor_original, false, link_ee_name, link_dummy_name);
			if (ik_found)
			{
				group.setJointValueTarget(computed_config); 
  				group.setPathConstraints(path_constraints);
  				group.setPlanningTime(60.0);
				feedback_.planning_success = group.plan(my_plan);
				ROS_INFO("STANDARD planning (1/0) = %d", feedback_.planning_success.val);
		  		if(feedback_.planning_success.val == 1)
				{
					result_.planning_code.val = 1;
		  			ROS_INFO("STANDARD execution on going ...");
		  			while(feedback_.execution_success.val != 1)
						feedback_.execution_success = group.execute(my_plan);
					ROS_INFO("STANDARD execution completed!");
    					result_.sequence.push_back(0);
    					result_.planned_trajectories.push_back(my_plan.trajectory_.joint_trajectory);
					result_.execution_code.val = 1;					
				}
			}
			else
			{
				planning_mode = 1;
			}
			break;

		case (1): // pivoting planning mode 

			ROS_INFO("CASE B - PIVOTING planning mode ");
    			nh_.setParam("robot_description", ss_augmented.str());
			ik_found = GetCorrectIK(group_name, num_joints, target_pose, path_constraints, planning_scene_monitor_original, true, link_ee_name, link_dummy_name);
			if(ik_found)
			{


				tf2::Quaternion w_Q_rf(0.70711, 0.0, -0.70711, 0.0); //<- denkmit object piv
						//tf2::Quaternion w_Q_rf(-0.5, -0.5, 0.5, 0.5); //<- no object piv
						tf2::Quaternion w_Q_target(goal->target_pose.orientation.x, goal->target_pose.orientation.y, goal->target_pose.orientation.z, goal->target_pose.orientation.w);

						tf2::Quaternion Q_unit = (w_Q_target.inverse() * w_Q_rf);
						double norm_vect = Q_unit[0] + Q_unit[1] + Q_unit[2];
                  norm_vect = 0.0;

						cout << "w_Q_target --> " << tf2::toMsg(w_Q_target) << endl;
						cout << "Q_unit --> " << tf2::toMsg(Q_unit) << endl;
						cout << "norm_vect --> " << norm_vect << endl;

bool gp_ik_found;
moveit::planning_interface::MoveGroupInterface::Plan gp_plan;
moveit_msgs::Constraints custom_constraints;
moveit_msgs::OrientationConstraint ocm;
moveit_msgs::PositionConstraint pcm;
shape_msgs::SolidPrimitive primitive;
moveit_msgs::BoundingVolume bv1;
						if( fabs(norm_vect) >= 0.01 )
						{
							goto PIVOTING2;
						}

				// Pivoting:
				
				//moveit::planning_interface::MoveGroupInterface::Plan gp_plan;
				//moveit_msgs::Constraints custom_constraints;
				ROS_INFO("PIVOTING_MODE -> Applico il constraint in orientamento sul dummy");
				//moveit_msgs::OrientationConstraint ocm;
				ocm.link_name = group.getEndEffectorLink().c_str();
				ocm.header.frame_id = group.getEndEffectorLink().c_str();
				ocm.orientation.w = 1.0;
				ocm.absolute_x_axis_tolerance = 0.0175;
				ocm.absolute_y_axis_tolerance = 0.0175;
				ocm.absolute_z_axis_tolerance = 2*M_PI;
				ocm.weight = 1.0;
				custom_constraints.orientation_constraints.push_back(ocm);
				ROS_INFO("PIVOTING_MODE -> Applico il constraint in posizione sul dummy");
				//moveit_msgs::PositionConstraint pcm;
				pcm.link_name = group.getEndEffectorLink().c_str();
				pcm.target_point_offset.x = 0.00001;
				pcm.target_point_offset.y = 0.00001;
				pcm.target_point_offset.z = 0.00001;
				//shape_msgs::SolidPrimitive primitive;
				primitive.type = shape_msgs::SolidPrimitive::BOX;
				primitive.dimensions.push_back(0.0005); // x
				primitive.dimensions.push_back(0.0005); // y
				primitive.dimensions.push_back(0.0005); // z
				//moveit_msgs::BoundingVolume bv1;
				bv1.primitives.push_back(primitive);
				bv1.primitive_poses.push_back(current_pose);
				pcm.constraint_region = bv1;
				pcm.weight = 1.0;
				custom_constraints.position_constraints.push_back(pcm);

				group.setPathConstraints(custom_constraints);
				group.setPlanningTime(60.0);

				group.setStartState(*group.getCurrentState());
				nh_.setParam("robot_description", ss_model.str());
				/*bool*/ gp_ik_found = GetCorrectIK(group_name, num_joints, pivoting_pose, custom_constraints, planning_scene_monitor_original, false, link_ee_name, link_dummy_name);

				if (gp_ik_found == true)
				{
					group.setJointValueTarget(computed_config); 
	  				group.setPathConstraints(custom_constraints);
	  				group.setPlanningTime(60.0);
					feedback_.planning_pivoting = group.plan(gp_plan);
					ROS_INFO("PIVOTING planning (1/0) = %d", feedback_.planning_pivoting.val);
			  		if(feedback_.planning_pivoting.val == 1)
					{
			  			ROS_INFO("PIVOTING execution on going ...");
			  			while(feedback_.execution_pivoting.val != 1)
							feedback_.execution_pivoting = group.execute(gp_plan);				
	    					result_.sequence.push_back(1);
    						result_.planned_trajectories.push_back(gp_plan.trajectory_.joint_trajectory);
						ROS_INFO("PIVOTING execution completed! 12456");
				/*		
						tf2::Quaternion w_Q_rf(0.70711, 0.0, -0.70711, 0.0); //<- denkmit object piv
						//tf2::Quaternion w_Q_rf(-0.5, -0.5, 0.5, 0.5); //<- no object piv
						tf2::Quaternion w_Q_target(goal->target_pose.orientation.x, goal->target_pose.orientation.y, goal->target_pose.orientation.z, goal->target_pose.orientation.w);

						tf2::Quaternion Q_unit = (w_Q_target.inverse() * w_Q_rf);
						double norm_vect = Q_unit[0] + Q_unit[1] + Q_unit[2];

						cout << "w_Q_target --> " << tf2::toMsg(w_Q_target) << endl;
						cout << "Q_unit --> " << tf2::toMsg(Q_unit) << endl;
						cout << "norm_vect --> " << norm_vect << endl;*/


						if( fabs(norm_vect) < 0.01 )
						{
						// Virtual rotation of the object
VIRTUAL_ROTATION:
						ros::WallDuration(1.0).sleep();
						nh_.setParam("robot_description", ss_augmented.str());
						ROS_INFO("Starting virtual rotation of the piece...");
						computed_config[7] = min_place_joint_conf[7];
						sensor_msgs::JointState joint_state;
						joint_state.name.push_back("iiwa_joint_1");
						joint_state.name.push_back("iiwa_joint_2");
						joint_state.name.push_back("iiwa_joint_3");
						joint_state.name.push_back("iiwa_joint_4");
						joint_state.name.push_back("iiwa_joint_5");
						joint_state.name.push_back("iiwa_joint_6");
						joint_state.name.push_back("iiwa_joint_7");
						joint_state.name.push_back("iiwa_joint_dummy");
						joint_state.position.push_back(computed_config[0]);
						joint_state.position.push_back(computed_config[1]);
						joint_state.position.push_back(computed_config[2]);
						joint_state.position.push_back(computed_config[3]);
						joint_state.position.push_back(computed_config[4]);
						joint_state.position.push_back(computed_config[5]);
						joint_state.position.push_back(computed_config[6]);
						joint_state.position.push_back(computed_config[7]);
						for(int i = 0; i < 4; i++)
						{
							pub_virtual_conf.publish(joint_state);
							sleep(0.5);
						}
						ROS_INFO("Virtual rotation of the piece completed!");
						ros::WallDuration(1.0).sleep();

						// Raggiungo la posa desiderata
						group.setStartState(*group.getCurrentState());
						group.setJointValueTarget(min_place_joint_conf);

        					//ROS_INFO("min_place_joint_conf:");
        					//for(int s = 0; s < num_joints; s++)
						//	ROS_INFO("%f", 	min_place_joint_conf[s]);

						group.clearPathConstraints();
		  				group.setPlanningTime(60.0);
						ROS_INFO("STANDARD place planning...");
      						moveit::planning_interface::MoveGroupInterface::Plan place_plan;
						while(feedback_.planning_success.val != 1)
						{
							feedback_.planning_success = group.plan(place_plan);
							ROS_INFO("STANDARD place planning (1/0) = %d", feedback_.planning_success.val);
							if(feedback_.planning_success.val == 1)
							{
								result_.planning_code.val = 1;
								ROS_INFO("STANDARD place execution on going ...");
								while(feedback_.execution_success.val != 1)
									feedback_.execution_success = group.execute(place_plan);
								ROS_INFO("STANDARD place execution completed");
	    							result_.sequence.push_back(0);
			    					result_.planned_trajectories.push_back(place_plan.trajectory_.joint_trajectory);
								result_.execution_code.val = 1;
							}
							else
							{
								ROS_INFO("TENTATIVO #2");
								group.setPoseTarget(place_pose);
  							}
  						}
					} else {

PIVOTING2:
							//PIVOTING INCLINATO
							ROS_INFO("PIVOTING INCLINATO planning...");
							//tf2::Quaternion w_Q_rf(0.70711, 0.0, -0.70711, 0.0);
							tf2::Quaternion w_Q_G_pl(pivoting_pose.orientation.x, pivoting_pose.orientation.y, pivoting_pose.orientation.z, pivoting_pose.orientation.w);
							tf2::Quaternion w_Q_O_pl(place_pose.orientation.x, place_pose.orientation.y, place_pose.orientation.z, place_pose.orientation.w);			
							tf2::Quaternion Q_diff = 	w_Q_O_pl * w_Q_rf.inverse();
							tf2::Quaternion Q_star = Q_diff * w_Q_G_pl;
							tf2::Quaternion Q_star_i = Q_diff.inverse() * w_Q_G_pl;

cout << "w_Q_rf ---> " << tf2::toMsg(w_Q_rf) << endl;
cout << "w_Q_G_pl ---> " << tf2::toMsg(w_Q_G_pl) << endl;
cout << "w_Q_O_pl ---> " << tf2::toMsg(w_Q_O_pl) << endl;
cout << "Q_diff ---> " << tf2::toMsg(Q_diff) << endl;
cout << "Q_star ---> " << tf2::toMsg(Q_star) << endl;
cout << "Q_star_i ---> " << tf2::toMsg(Q_star_i) << endl;

							{
							// Pivoting 2:
							moveit::planning_interface::MoveGroupInterface::Plan gp_plan;
							moveit_msgs::Constraints custom_constraints;
							ROS_INFO("PIVOTING_MODE -> Applico il constraint in orientamento sul dummy");
							moveit_msgs::OrientationConstraint ocm;
							ocm.link_name = group.getEndEffectorLink().c_str();
							ocm.header.frame_id = group.getEndEffectorLink().c_str();
							ocm.orientation.w = 1.0;
							ocm.absolute_x_axis_tolerance = 0.0175;
							ocm.absolute_y_axis_tolerance = 0.0175;
							ocm.absolute_z_axis_tolerance = 2*M_PI;
							ocm.weight = 1.0;
							custom_constraints.orientation_constraints.push_back(ocm);
							ROS_INFO("PIVOTING_MODE -> Applico il constraint in posizione sul dummy");
							moveit_msgs::PositionConstraint pcm;
							pcm.link_name = group.getEndEffectorLink().c_str();
							pcm.target_point_offset.x = 0.00001;
							pcm.target_point_offset.y = 0.00001;
							pcm.target_point_offset.z = 0.00001;
							shape_msgs::SolidPrimitive primitive;
							primitive.type = shape_msgs::SolidPrimitive::BOX;
							primitive.dimensions.push_back(0.0005); // x
							primitive.dimensions.push_back(0.0005); // y
							primitive.dimensions.push_back(0.0005); // z
							moveit_msgs::BoundingVolume bv1;
							bv1.primitives.push_back(primitive);
							bv1.primitive_poses.push_back(current_pose);
							pcm.constraint_region = bv1;
							pcm.weight = 1.0;
							custom_constraints.position_constraints.push_back(pcm);

							group.setPathConstraints(custom_constraints);
							group.setPlanningTime(60.0);

							group.setStartState(*group.getCurrentState());
							nh_.setParam("robot_description", ss_model.str());
							pivoting_pose.orientation = tf2::toMsg(Q_star_i);
							bool gp_ik_found = GetCorrectIK(group_name, num_joints, pivoting_pose, custom_constraints, planning_scene_monitor_original, false, link_ee_name, link_dummy_name);

							if(!gp_ik_found){
								cout << "ERRORE" << endl;
								exit(-1);
							}

							group.setJointValueTarget(computed_config); 
			  				group.setPathConstraints(custom_constraints);
			  				group.setPlanningTime(60.0);
							feedback_.planning_pivoting = group.plan(gp_plan);
							ROS_INFO("PIVOTING2 planning (1/0) = %d", feedback_.planning_pivoting.val);
					  		if(feedback_.planning_pivoting.val == 1)
							{
					  			ROS_INFO("PIVOTING2 execution on going ...");
								feedback_.execution_pivoting.val = 0;
					  			while(feedback_.execution_pivoting.val != 1)
									feedback_.execution_pivoting = group.execute(gp_plan);				
				 					result_.sequence.push_back(1);
			 						result_.planned_trajectories.push_back(gp_plan.trajectory_.joint_trajectory);
								ROS_INFO("PIVOTING2 execution completed!");
							}
							}

							{
							// Pivoting SA:
							moveit::planning_interface::MoveGroupInterface::Plan gp_plan;
							moveit_msgs::Constraints custom_constraints;
							ROS_INFO("PIVOTING_SA -> Applico il constraint in orientamento sul dummy");
							moveit_msgs::OrientationConstraint ocm;
							ocm.link_name = group.getEndEffectorLink().c_str();
							ocm.header.frame_id = group.getEndEffectorLink().c_str();
							ocm.orientation.w = 1.0;
							ocm.absolute_x_axis_tolerance = 0.0175;
							ocm.absolute_y_axis_tolerance = 0.0175;
							ocm.absolute_z_axis_tolerance = 2*M_PI;
							ocm.weight = 1.0;
							custom_constraints.orientation_constraints.push_back(ocm);
							ROS_INFO("PIVOTING_SA -> Applico il constraint in posizione sul dummy");
							moveit_msgs::PositionConstraint pcm;
							pcm.link_name = group.getEndEffectorLink().c_str();
							pcm.target_point_offset.x = 0.00001;
							pcm.target_point_offset.y = 0.00001;
							pcm.target_point_offset.z = 0.00001;
							shape_msgs::SolidPrimitive primitive;
							primitive.type = shape_msgs::SolidPrimitive::BOX;
							primitive.dimensions.push_back(0.0005); // x
							primitive.dimensions.push_back(0.0005); // y
							primitive.dimensions.push_back(0.0005); // z
							moveit_msgs::BoundingVolume bv1;
							bv1.primitives.push_back(primitive);
							bv1.primitive_poses.push_back(current_pose);
							pcm.constraint_region = bv1;
							pcm.weight = 1.0;
							custom_constraints.position_constraints.push_back(pcm);

							group.setPathConstraints(custom_constraints);
							group.setPlanningTime(60.0);

							group.setStartState(*group.getCurrentState());
							nh_.setParam("robot_description", ss_model.str());
							pivoting_pose.orientation = tf2::toMsg(w_Q_G_pl);
							bool gp_ik_found = GetCorrectIK(group_name, num_joints, pivoting_pose, custom_constraints, planning_scene_monitor_original, false, link_ee_name, link_dummy_name);

							if(!gp_ik_found){
								cout << "ERRORE" << endl;
								exit(-1);
							}

							group.setJointValueTarget(computed_config); 
			  				group.setPathConstraints(custom_constraints);
			  				group.setPlanningTime(60.0);
							feedback_.planning_pivoting = group.plan(gp_plan);
							ROS_INFO("PIVOTING_SA planning (1/0) = %d", feedback_.planning_pivoting.val);
					  		if(feedback_.planning_pivoting.val == 1)
							{
					  			ROS_INFO("PIVOTING_SA execution on going ...");
								feedback_.execution_pivoting.val = 0;
					  			while(feedback_.execution_pivoting.val != 1)
									feedback_.execution_pivoting = group.execute(gp_plan);				
				 					result_.sequence.push_back(0);
			 						result_.planned_trajectories.push_back(gp_plan.trajectory_.joint_trajectory);
								ROS_INFO("PIVOTING_SA execution completed!");
							}
							}
goto VIRTUAL_ROTATION;


						}					
					}
				}
				else
				{
					planning_mode = 2;
				}
			}
			else
			{
				planning_mode = 2;
			}
			break;

		case (2): 
			ROS_INFO("CASE C - Solution not found");
			sleep(10.0);
			break;
	}
   }

      as_.publishFeedback(feedback_); // publish the feedback
      r.sleep();  // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes

    if(success)
    {
      result_.planning_code = feedback_.planning_success;
      result_.execution_code = feedback_.execution_success;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      ROS_INFO("-------------------------------------------");
      as_.setSucceeded(result_);  // set the action state to succeeded
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pivoting_planner_server");

  PivotingPlannerActionServer pivoting_planner("pivoting_plan_action");
  ros::spin();

  return 0;
}
