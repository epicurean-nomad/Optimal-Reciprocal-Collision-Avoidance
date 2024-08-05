/*
 * This file provides an API to interact with the RVO2 (ORCA) Library
 * which has been modified to work in ROS
 * 
 */

#include "./OrcaAgents.h"
#include <algorithm>
// #include "orca/GlobalPlanner.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
// #include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>
#include <fstream>
#include "../ORCA/RVO.h"
#include <vector>
#include <bits/stdc++.h>
#include <thread>

#include <rviz_visual_tools/rviz_visual_tools.h>


// using namespace std;

namespace rvt = rviz_visual_tools;

ros::ServiceClient get_model_state_client;
// rvt::RvizVisualToolsConstPtr visual_tools_;


gazebo_msgs::GetModelState get_state(std::string name){
            gazebo_msgs::GetModelState srv;
		    srv.request.model_name = name;  // Replace with your agent name

            if (get_model_state_client.call(srv)) {

		    	return srv;   
            }
            else{
                ROS_ERROR("Failed to call GetModelState service");
            }
}


void publishVelocityObstacles(RVO::RVOSimulator *sim, OrcaAgent* agent,ros::Publisher vo_pb, visualization_msgs::MarkerArray markerList){
	int agentNo = 0,neighbourID;
	double neighbourRadius, distanceToNeighbour, agentRadius = sim->getAgentRadius(agentNo), centreAngle, tgtAngle, slopeTgtRAngle, slopeTgtLAngle,agentTimeHorizon;
	Vector2 neighbourPosition, neighbourTimeAgentRelativePosition, neighbourVelocity, agentPosition = sim->getAgentPosition(agentNo);

	// visualization_msgs::Marker line_marker;
    // line_marker.header.frame_id = "map"; // Set the appropriate frame ID
    // line_marker.header.stamp = ros::Time::now();
    // line_marker.ns = "vo_lines";
    // line_marker.action = visualization_msgs::Marker::ADD;
    // line_marker.pose.orientation.w = 1.0; // Set orientation to identity
    // line_marker.id = 0; // Unique ID for each line
    // line_marker.type = visualization_msgs::Marker::LINE_LIST;
    // line_marker.scale.x = 0.01; // Set the line width
    // line_marker.color.g = 0.5;
	// line_marker.color.b = -0.5; 
    // line_marker.color.a = 1.0; // Set the line alpha (opaque)

	visualization_msgs::Marker tri_marker;
    tri_marker.header.frame_id = "map"; // Set the appropriate frame ID
    tri_marker.header.stamp = ros::Time::now();
    tri_marker.ns = "vo_lines";
    tri_marker.action = visualization_msgs::Marker::ADD;
    tri_marker.pose.orientation.w = 1.0; // Set orientation to identity
    tri_marker.id = -2; // Unique ID for each line
    tri_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    tri_marker.scale.x = 1; 
	tri_marker.scale.y = 1;
	tri_marker.scale.z = 1;
    tri_marker.color.g = 1; 
	tri_marker.color.b = 0;
	tri_marker.color.r = 0;
    tri_marker.color.a = 0.5; // Set the line alpha (opaque)


	

	for (int i = 0; i < sim->getAgentNumAgentNeighbors(agentNo); i++) {

		neighbourID = sim->getAgentAgentNeighbor(agentNo,i);
		neighbourRadius = sim->getAgentRadius(neighbourID);
		neighbourPosition = sim->getAgentPosition(neighbourID);
		neighbourVelocity = sim->getAgentVelocity(neighbourID);
		agentTimeHorizon = sim->getAgentTimeHorizon(agentNo);

		neighbourTimeAgentRelativePosition = agentPosition + (neighbourPosition-agentPosition)/agentTimeHorizon;



		double distanceToNeighbourT = sqrt(pow(neighbourTimeAgentRelativePosition.x()-agentPosition.x(),2) + pow(neighbourTimeAgentRelativePosition.y()-agentPosition.y(),2));
		distanceToNeighbour = sqrt(pow(neighbourPosition.x()-agentPosition.x(),2) + pow(neighbourPosition.y()-agentPosition.y(),2));
		
		centreAngle = atan2(neighbourTimeAgentRelativePosition.y()-agentPosition.y(),neighbourTimeAgentRelativePosition.x()-agentPosition.x());
		// double centreAngle1 = atan2(neighbourPosition.y()-agentPosition.y(),neighbourPosition.x()-agentPosition.x());

		// std::cout<<std::endl<<"Centre AngleT: "<<centreAngle<<std::endl<<"Centre Angle: "<<centreAngle1<<std::endl<<std::endl;
		double tgtAngleT = asin(std::min(1.0,std::max(-1.0,(neighbourRadius + agentRadius)/(agentTimeHorizon*distanceToNeighbourT)))) ;
		tgtAngle = asin(std::min(1.0,std::max(-1.0,(neighbourRadius + agentRadius)/distanceToNeighbour))) ;

		// std::cout<<std::endl<<"Tgt AngleT: "<<tgtAngleT<<std::endl<<"Tgt Angle: "<<tgtAngle<<std::endl<<std::endl;

		slopeTgtLAngle = centreAngle + tgtAngleT;
		slopeTgtRAngle = centreAngle - tgtAngleT;

		geometry_msgs::Point start_pointL, end_pointL, start_pointR, end_pointR, start_pointTransformedL, start_pointTransformedR, end_pointTransformedL, end_pointTransformedR;
    	// start_pointL.x = neighbourTimeAgentRelativePosition.x() + (neighbourRadius + agentRadius)*cos(slopeTgtLAngle*M_PI/180)/agentTimeHorizon;
    	// start_pointL.y = neighbourTimeAgentRelativePosition.y() + (neighbourRadius + agentRadius)*sin(slopeTgtLAngle*M_PI/180)/agentTimeHorizon;
    	
		// start_pointR.x = neighbourTimeAgentRelativePosition.x() + (neighbourRadius + agentRadius)*cos(slopeTgtRAngle*M_PI/180)/agentTimeHorizon;
    	// start_pointR.y = neighbourTimeAgentRelativePosition.y() + (neighbourRadius + agentRadius)*sin(slopeTgtRAngle*M_PI/180)/agentTimeHorizon;
		double mL = tan(slopeTgtLAngle);
		double mR = tan(slopeTgtRAngle);
		
		start_pointL.x = (pow(mL,2)*agentPosition.x() + mL*(neighbourTimeAgentRelativePosition.y() - agentPosition.y()) + neighbourTimeAgentRelativePosition.x())/ (1+pow(mL,2));
    	start_pointL.y = (pow(mL,2)*neighbourTimeAgentRelativePosition.y() + mL*(neighbourTimeAgentRelativePosition.x() - agentPosition.x()) + agentPosition.y())/ (1+pow(mL,2));
    	
		start_pointR.x = (pow(mR,2)*agentPosition.x() + mR*(neighbourTimeAgentRelativePosition.y() - agentPosition.y()) + neighbourTimeAgentRelativePosition.x())/ (1+pow(mR,2));
    	start_pointR.y = (pow(mR,2)*neighbourTimeAgentRelativePosition.y() + mR*(neighbourTimeAgentRelativePosition.x() - agentPosition.x()) + agentPosition.y())/ (1+pow(mR,2));

		start_pointTransformedL.x = start_pointL.x + neighbourVelocity.x();
    	start_pointTransformedL.y = start_pointL.y + neighbourVelocity.y();
    	
		start_pointTransformedR.x = start_pointR.x + neighbourVelocity.x();
    	start_pointTransformedR.y = start_pointR.y + neighbourVelocity.y();



		double lineLength = 1000.0; // Adjust this value as needed for visualization

		end_pointL.x = agentPosition.x() + lineLength * cos(slopeTgtLAngle);
		end_pointL.y = agentPosition.y() + lineLength * sin(slopeTgtLAngle);

		end_pointR.x = agentPosition.x() + lineLength * cos(slopeTgtRAngle);
		end_pointR.y = agentPosition.y() + lineLength * sin(slopeTgtRAngle);

		end_pointTransformedL.x = end_pointL.x + neighbourVelocity.x();
    	end_pointTransformedL.y = end_pointL.y + neighbourVelocity.y();
		end_pointTransformedR.x = end_pointR.x + neighbourVelocity.x();
    	end_pointTransformedR.y = end_pointR.y + neighbourVelocity.y();

    	
		tri_marker.points.push_back(start_pointTransformedL);
    	tri_marker.points.push_back(end_pointTransformedL);
    	tri_marker.points.push_back(end_pointTransformedR);
		
		tri_marker.points.push_back(start_pointTransformedR);
		tri_marker.points.push_back(end_pointTransformedL);
    	tri_marker.points.push_back(end_pointTransformedR);
		
		visualization_msgs::Marker obst_marker,obst_markerT;

		obst_markerT.header.frame_id = "map"; // Set the appropriate frame ID
    	obst_markerT.header.stamp = ros::Time::now();
    	obst_markerT.ns = "velocity_obstacles";
    	obst_markerT.action = visualization_msgs::Marker::ADD;
    	obst_markerT.pose.orientation.w = 1.0; // Set orientation to identity
    	obst_markerT.id = neighbourID; // Unique ID for each line
    	obst_markerT.type = visualization_msgs::Marker::CYLINDER;
    	obst_markerT.scale.x = 2*(neighbourRadius + agentRadius)/agentTimeHorizon; // Set the line width
		obst_markerT.scale.y = 2*(neighbourRadius + agentRadius)/agentTimeHorizon;
		obst_markerT.scale.z = 0.05;
    	obst_markerT.color.g = 0.5;
		obst_markerT.pose.position.x = neighbourTimeAgentRelativePosition.x() + neighbourVelocity.x();
		obst_markerT.pose.position.y = neighbourTimeAgentRelativePosition.y()+ neighbourVelocity.y();
		obst_markerT.pose.position.z = 0.025;
    	obst_markerT.color.a = 0.5; // Set the line alpha (opaque)


		obst_marker.header.frame_id = "map"; // Set the appropriate frame ID
    	obst_marker.header.stamp = ros::Time::now();
    	obst_marker.ns = "velocity_obstacles";
    	obst_marker.action = visualization_msgs::Marker::ADD;
    	obst_marker.pose.orientation.w = 1.0; // Set orientation to identity
    	obst_marker.id = neighbourID + 100; // Unique ID for each line
    	obst_marker.type = visualization_msgs::Marker::CYLINDER;
    	obst_marker.scale.x = 2*(neighbourRadius + agentRadius); // Set the line width
		obst_marker.scale.y = 2*(neighbourRadius + agentRadius);
		obst_marker.scale.z = 0.05;
    	obst_marker.color.g = 0.5;
		obst_marker.pose.position.x = neighbourPosition.x();
		obst_marker.pose.position.y = neighbourPosition.y();
		obst_marker.pose.position.z = 0.025;
    	obst_marker.color.a = 0.5;
		
		
		// markerList.markers.push_back(obst_marker);
		markerList.markers.push_back(obst_markerT);

	}
	markerList.markers.push_back(tri_marker);
	visualization_msgs::Marker deleteMarker;
	deleteMarker.id = -1;
	deleteMarker.action = visualization_msgs::Marker::DELETEALL;
	deleteMarker.ns = "";
	markerList.markers.push_back(deleteMarker);
	vo_pb.publish(markerList);
}



void publishOrcaLines(RVO::RVOSimulator *sim, OrcaAgent* agent,ros::Publisher line_pb,rvt::RvizVisualToolsPtr vt) {
    int agentNo = 0;

	int planeSize = 100;
	Vector2 planeCenter;
	RVO::Line orcaLine;
	Eigen::Isometry3d pose;
	geometry_msgs::Point start_point, end_point;
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "map"; // Set the appropriate frame ID
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "orca_lines";
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0; // Set orientation to identity
    line_marker.id = 0; // Unique ID for each line
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.scale.x = 0.1; // Set the line width
    line_marker.color.g = 0.5;
	line_marker.color.b = -0.5; 
    line_marker.color.a = 1.0; // Set the line alpha (opaque)


	vt->deleteAllMarkers();



    for (int i = 0; i < sim->getAgentNumORCALines(agentNo); i++) {
        RVO::Line orcaLine = sim->getAgentORCALine(agentNo, i);
    	geometry_msgs::Point start_point, end_point;
    	start_point.x = orcaLine.point.x() - 1000*orcaLine.direction.x();
    	start_point.y = orcaLine.point.y() - 1000*orcaLine.direction.y();
    	end_point.x = orcaLine.point.x() + 1000*orcaLine.direction.x();
    	end_point.y = orcaLine.point.y() + 1000*orcaLine.direction.y();
    	line_marker.points.push_back(start_point);
    	line_marker.points.push_back(end_point);

    	line_pb.publish(line_marker);

		Eigen::Isometry3d pose;
		pose = Eigen::AngleAxisd(atan2(orcaLine.direction.y(),orcaLine.direction.x()), Eigen::Vector3d::UnitZ());

		Vector2 planeCenter;
		planeCenter = orcaLine.point + normalize(Vector2(-orcaLine.direction.y(),orcaLine.direction.x()))*planeSize;

		pose.translation() = Eigen::Vector3d( planeCenter.x(), planeCenter.y(), 0 ); // translate x,y,z

		vt->publishXYPlane(pose, rvt::TRANSLUCENT_LIGHT, planeSize);
    }

	vt->trigger();
	line_pb.publish(line_marker);
	

}





void setPrefVelocities(RVO::RVOSimulator *sim, std::vector<OrcaAgent*> agents){

	for(int i=0; i<agents.size(); i++){
		agents[i]->updatePrefVel(sim);
	}

}


void publishNewVelocity(RVO::RVOSimulator *sim, OrcaAgent* agent, RVO::Vector2 new_vel, ros::Publisher new_vel_pb){
	visualization_msgs::Marker new_velocity_marker;
    new_velocity_marker.header.frame_id = "map"; // Set the appropriate frame ID
    new_velocity_marker.header.stamp = ros::Time::now();
    new_velocity_marker.ns = "Robot1";
    new_velocity_marker.action = visualization_msgs::Marker::ADD;
    new_velocity_marker.pose.orientation.w = 1.0; // Set orientation to identity
    new_velocity_marker.id = 0; // Unique ID for each line
    new_velocity_marker.type = visualization_msgs::Marker::ARROW;
    new_velocity_marker.scale.x = 0.1; 
	new_velocity_marker.scale.y = 0.25;
	new_velocity_marker.scale.z = 0.3;

	new_velocity_marker.color.r = 1.0;
	new_velocity_marker.color.b = 1.0;
    new_velocity_marker.color.a = 1.0; 


	// new_velocity_marker.pose.orientation = srv.response.pose.orientation;
	geometry_msgs::Point start, end;
	start.x = 0; start.y = 0, start.z = 0;
	end.x = new_vel.x(); end.y = new_vel.y(), end.z = 0;

	// std::cout<<"publishing new velocity"<<std::endl;

	new_velocity_marker.points.push_back(start);
	new_velocity_marker.points.push_back(end);
	

	
	
	

	new_vel_pb.publish(new_velocity_marker);


}

void getOrcaVelocities(RVO::RVOSimulator *sim, std::vector<OrcaAgent*> agents,ros::Publisher new_vel_pb){
	sim->doStep();

	for(int i=0;i<sim->getNumAgents();i++){
        // std::cout<<"Number of neighbours for "<<agents[i]->getName()<<": "<<sim->getAgentNumAgentNeighbors(i)<<std::endl;
		// std::cout<<"Number of ORCALines for "<<agents[i]->getName()<<": "<<sim->getAgentNumORCALines(i)<<std::endl<<std::endl;
		RVO::Vector2 new_vel = sim->getAgentVelocity(i);
		if(i == 0){
			publishNewVelocity(sim,agents[0],new_vel,new_vel_pb);
		}
		agents[i]->moveAgentOrca(new_vel);
	}
	
}

void updateORCA(RVO::RVOSimulator *sim, std::vector<OrcaAgent*> agents){

	for(int i=0;i<sim->getNumAgents();i++){
		gazebo_msgs::GetModelState srv = get_state(agents[i]->getName());

		double roll,pitch,yaw;
        tf::Quaternion q(
            srv.response.pose.orientation.x,
            srv.response.pose.orientation.y,
            srv.response.pose.orientation.z,
            srv.response.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

		sim->setAgentPosition(i,RVO::Vector2(srv.response.pose.position.x,srv.response.pose.position.y));
		sim->setAgentVelocity(i,RVO::Vector2(cos(yaw)*srv.response.twist.linear.x,sin(yaw)*srv.response.twist.linear.x));
		agents[i]->updateGoal();

	}
	

}


void setupScenario(RVO::RVOSimulator *sim, std::vector<OrcaAgent*> agents, double radius){

			/* Specify the global time step of the simulation. */

            
			sim->setTimeStep(0.001f);
            
			/* Specify the default parameters for agents that are subsequently added. */
			sim->setAgentDefaults(5*radius, 5, 5.0f, 5.0f, radius, 1.5);
		

			// Add agents, specifying their start position

			gazebo_msgs::GetModelState srv;

			
			for (int i=0; i<agents.size(); i++){

				srv = get_state(agents[i]->getName());
				std::cout<<"Added agent with ID: "<<sim->addAgent(RVO::Vector2(srv.response.pose.position.x , srv.response.pose.position.y))<<std::endl;
			
			}

}


void publishPose(RVO::RVOSimulator *sim, OrcaAgent* agent,ros::Publisher pose_pb){
	visualization_msgs::Marker pose_marker;
    pose_marker.header.frame_id = "map"; // Set the appropriate frame ID
    pose_marker.header.stamp = ros::Time::now();
    pose_marker.ns = "Robot1";
    pose_marker.action = visualization_msgs::Marker::ADD;
    pose_marker.pose.orientation.w = 1.0; // Set orientation to identity
    pose_marker.id = 0; // Unique ID for each line
    pose_marker.type = visualization_msgs::Marker::ARROW;
    // pose_marker.scale.x = 1.0;
	pose_marker.scale.y = 0.1;
	pose_marker.scale.z = 0.1;
    pose_marker.color.g = 1.0; // Set the line color (red)
    pose_marker.color.a = 1.0; // Set the line alpha (opaque)

	

	gazebo_msgs::GetModelState srv = get_state(agent->getName());

	double vx = srv.response.twist.linear.x;
    double vy = srv.response.twist.linear.y;
    
    // Calculate velocity magnitude
    double velocity_magnitude = sqrt(vx*vx + vy*vy);

	pose_marker.pose.orientation = srv.response.pose.orientation;
	pose_marker.pose.position = srv.response.pose.position;
	pose_marker.scale.x = velocity_magnitude;


	pose_pb.publish(pose_marker);

}






void publishGoal(RVO::RVOSimulator *sim, OrcaAgent* agent,ros::Publisher goal_pb){

	visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "map"; // Set the appropriate frame ID
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "Robot1";
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.orientation.w = 1.0; // Set orientation to identity
    goal_marker.id = 0; // Unique ID for each line
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.scale.x = 0.5; 
	goal_marker.scale.y = 0.5;
	goal_marker.scale.z = 0.5;

	goal_marker.color.r = 1.0; // 
    goal_marker.color.a = 1.0; // Set the line alpha (opaque)

	geometry_msgs::Point goal = agent->getGoal();

	gazebo_msgs::GetModelState srv = get_state(agent->getName());

	goal_marker.pose.orientation = srv.response.pose.orientation;
	goal_marker.pose.position = goal;

	goal_pb.publish(goal_marker);

}



int main(int argc, char**argv)
{		
	ros::init(argc, argv, "main");
	ros::NodeHandle nh;

	rvt::RvizVisualToolsPtr visual_tools_;

	visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map","/orcaPlanes"));

	visualization_msgs::MarkerArray markerList;


	std::vector<std::string> agent_names = {"Robot1","Robot2","Robot3","Robot4","Robot5","Robot6","Robot7","Robot8","Robot9","Robot10","Robot11","Robot12"};

	std::vector<OrcaAgent*> agents;

	get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	
	// ros::Subscriber sub = nh.subscribe("/Robot1/orcaLines", 1000, chatterCallback);

	ros::Publisher line_pb = nh.advertise<visualization_msgs::Marker>("/Robot1/orcaLines", 5);

	ros::Publisher pose_pb = nh.advertise<visualization_msgs::Marker>("/Robot1/pose", 5);

	ros::Publisher goal_pb = nh.advertise<visualization_msgs::Marker>("/Robot1/goal", 5);

	ros::Publisher new_vel_pb = nh.advertise<visualization_msgs::Marker>("/Robot1/new_vel", 5);

	ros::Publisher vo_pb = nh.advertise<visualization_msgs::MarkerArray>("/Robot1/vo", 5);


	for(int i=0; i<agent_names.size();i++){

        // std::cout<<agent_names[i]<<std::endl;
		
		OrcaAgent* agent = new OrcaAgent(nh, agent_names[i],2.0,i);

		agents.push_back(agent);
	}

	RVO::RVOSimulator *globalSim = new RVO::RVOSimulator();

    std::cout<<"Safe"<<std::endl;

	setupScenario(globalSim,agents,0.4);

	std::thread ros_thread([&]() {
        ros::spin();
    });
	
	do{
		setPrefVelocities(globalSim,agents);
		getOrcaVelocities(globalSim,agents,new_vel_pb);
		updateORCA(globalSim,agents);
		publishOrcaLines(globalSim,agents[0],line_pb,visual_tools_);
		publishPose(globalSim,agents[0],pose_pb);
		publishGoal(globalSim,agents[0],goal_pb);
		publishVelocityObstacles(globalSim,agents[0],vo_pb,markerList);
		// publishGoal(globalSim,agents[0],new_vel_pb);

	}
	while(true);

	


	ros_thread.join();

	

	return 0;
}











