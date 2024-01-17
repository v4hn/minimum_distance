#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <Eigen/Geometry>

#include <boost/format.hpp>
#include <utility>

int main(int argc, char** argv){
	ros::init(argc, argv, "test_minimum_distance");
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// load robot and monitor
	auto psm{ std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description") };
	auto planning_scene{ std::make_shared<planning_scene::PlanningScene>(psm->getRobotModel()) };
	psm->requestPlanningSceneState();
	psm->startStateMonitor();
	if(!psm->waitForCurrentRobotState(ros::Time::now())){
		return 1;
	}

	// get link_a and link_b
	auto link_a{ planning_scene->getRobotModel()->getLinkModel("l_gripper_l_finger_tip_link") };
	auto link_b{ planning_scene->getRobotModel()->getLinkModel("rh_forearm") };
	std::set<moveit::core::LinkModel const*> links{ link_a, link_b };
	auto const link_pair{ std::make_pair(link_a->getName(), link_b->getName()) };

	collision_detection::DistanceRequest request;
	collision_detection::DistanceResult result;
	request.type = collision_detection::DistanceRequestType::SINGLE;
	request.active_components_only = &links;
	request.enable_signed_distance = true;
	request.enable_nearest_points = true;
	request.compute_gradient = true;
	request.verbose = true;

	// publish points as visual spheres
	ros::NodeHandle nh;
	ros::Publisher pub{ nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true) };
	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_footprint";
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	markers.markers.push_back(marker);
	markers.markers.push_back(marker);

	ros::Rate rate{ 10 };

	while(ros::ok()){
		result.clear();
		{
			planning_scene_monitor::LockedPlanningSceneRO const scene{ psm };
			planning_scene = scene->clone(scene);
		}

		planning_scene->getCurrentStateNonConst().update();
		planning_scene->getCollisionEnv()->distanceSelf(request, result, planning_scene->getCurrentState());

		if(result.distances.find(link_pair) == result.distances.end()){
			std::cerr << "No distance results between link_a and link_b" << std::endl;
			return 1;
		}
		auto const pair_distances{ result.distances[link_pair] };
		if(pair_distances.empty()){
			std::cerr << "empty distance results between link_a and link_b" << std::endl;
			return 1;
		}

		collision_detection::DistanceResultsData const link_results{ pair_distances[0] };
		std::cout << "Nearst points:\n"
				<< " on Link " << link_results.link_names[0] << ": " << link_results.nearest_points[0].transpose() << "\n"
				<< " on Link " << link_results.link_names[1] << ": " << link_results.nearest_points[1].transpose() << "\n";
		markers.markers[0].ns = std::string{"test_minimum_distance - Link "} + link_results.link_names[0];
		markers.markers[0].pose.position.x = link_results.nearest_points[0].x();
		markers.markers[0].pose.position.y = link_results.nearest_points[0].y();
		markers.markers[0].pose.position.z = link_results.nearest_points[0].z();
		markers.markers[1].ns = std::string{"test_minimum_distance - Link "} + link_results.link_names[1];
		markers.markers[1].pose.position.x = link_results.nearest_points[1].x();
		markers.markers[1].pose.position.y = link_results.nearest_points[1].y();
		markers.markers[1].pose.position.z = link_results.nearest_points[1].z();

		pub.publish(markers);
		rate.sleep();
	}

	return 0;
}
