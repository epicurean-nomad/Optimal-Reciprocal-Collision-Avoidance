
#ifndef TEST_SIMULATOR_1
#define TEST_SIMULATOR_1

#pragma once

#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#ifndef RVO_SEED_RANDOM_NUMBER_GENERATOR
#define RVO_SEED_RANDOM_NUMBER_GENERATOR 1
#endif


#include <cmath>
#include <cstdlib>
#include <vector>
#include <fstream>
// #include <opencv2/opencv.hpp>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#if RVO_SEED_RANDOM_NUMBER_GENERATOR
#include <ctime>
// #include <math>
#endif

#if _OPENMP
#include <omp.h>
#endif



/* Including the ros headers*/
#include <ros/ros.h>
// #include <gazebo/math.h>
// #include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/GetModelState.h>
#include <bits/stdc++.h>
// #include <sensor_msgs/LaserScan.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
// #include "orca_msgs/AgentState.h"
// #include "orca_msgs/DetectedEntity.h"

// #include <nav_msgs/OccupancyGrid.h>
// #include <eigen3/Eigen/Dense>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

/* Including the RVO headers*/
// #include "orca/Definitions.h"
#include "../ORCA/RVO.h"

#include <tf/tf.h>
#include <mutex>


/*Set CV FLAG*/
#define CV_FLAG true



using namespace RVO;


class OrcaAgent{
    
	
	private:

        std::string name;
		double prefVel;
        ros::Publisher velocity_pb;
		
        ros::Timer goal_timer;
        ros::Timer move_timer;
        ros::ServiceClient get_model_state_client;

        double radius = 0.5;
        double min_x = -7.0, max_x = 7.0, min_y = -7.0, max_y = 7.0;

        geometry_msgs::Point goal;


		int id;



    public:

		// ros::Publisher line_pb;

        OrcaAgent(ros::NodeHandle& nh, std::string name, double prefVel,int id){
            
			
			
			this->name = name;

			this->id = id;

            std::stringstream ss;
            ss<<"/" << name << "/cmd_vel";

			std::cout<<ss.str()<<std::endl;

            this->velocity_pb = nh.advertise<geometry_msgs::Twist>(ss.str(), 10);

			
			// this->line_pb = nh.advertise<geometry_msgs::Twist>("/Robot1/orcaLines", 10);

			// std::cout<<"here"<<std::endl;

			get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

			// move_timer = nh.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent& event) {this->moveAgent(event);});


			double rand_x = this->min_x + static_cast<double>(rand()) * (this->max_x - this->min_x)/ (static_cast<double>(RAND_MAX));
            double rand_y = this->min_y + static_cast<double>(rand()) * (this->max_x - this->min_x) / (static_cast<double>(RAND_MAX));

			this->goal.x = rand_x;
			this->goal.y = rand_y;

			// this->goal.x = -7;
			// this->goal.y = 1;

			std::cout<<this->name<<" New Goal: ("<<this->goal.x<<","<<this->goal.y<<")"<<std::endl;

			this->prefVel = prefVel;
            // goal_timer = nh.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent& event) { this->updateGoal(event); }, goal_timer);

  
        };


		geometry_msgs::Point getGoal(){
			return this->goal;
		}

        
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


		RVO::Vector2 updatePrefVel(RVO::RVOSimulator *sim){

			gazebo_msgs::GetModelState srv = this->get_state(this->name);

            double roll,pitch,yaw;

        	tf::Quaternion q(
        	    srv.response.pose.orientation.x,
        	    srv.response.pose.orientation.y,
        	    srv.response.pose.orientation.z,
        	    srv.response.pose.orientation.w
        	);

        	tf::Matrix3x3 m(q);
        	m.getRPY(roll, pitch, yaw);


        	double x = srv.response.pose.position.x;
        	double y = srv.response.pose.position.y;
        	// Calculate the distance and angle to the goal
        	double goal_x = this->goal.x;
        	double goal_y = this->goal.y;


        	double dx = goal_x - x;
        	double dy = goal_y - y;

			Vector2 prefVec(dx,dy);

			// std::cout

			double distance_to_goal = mag(prefVec);

			Vector2 normVec = normalize(prefVec);

            // std::cout<<"Preffered velocity for "<<this->name<<": ("<<normVec.x()<<","<<normVec.y()<<std::endl<<std::endl;

			
			sim->setAgentPrefVelocity(this->id, Vector2(normVec.x() , normVec.y()));
			sim->setAgentVelocity(this->id, Vector2(normVec.x() , normVec.y()));
			// std::cout<<"Updated Pref Velocity for Agent"<<this->id<<" in simulator to "<<sim->getAgentPrefVelocity(this->id)<<std::endl;
		}


		void moveAgentOrca(RVO::Vector2 new_vel){

            // std::cout<<"New velocity for "<<this->name<<": ("<<new_vel.x()<<","<<new_vel.y()<<")"<<std::endl<<std::endl;

			gazebo_msgs::GetModelState srv = this->get_state(this->name);
			
			double magnitude = mag(new_vel);

			double new_angle = atan2(new_vel.y(), new_vel.x());

			double roll,pitch,yaw;
        	tf::Quaternion q(
        	    srv.response.pose.orientation.x,
        	    srv.response.pose.orientation.y,
        	    srv.response.pose.orientation.z,
        	    srv.response.pose.orientation.w
        	);

        	tf::Matrix3x3 m(q);
        	m.getRPY(roll, pitch, yaw);


			geometry_msgs::Twist vel_msg;

			double max_angular_velocity = 5;

			vel_msg.linear.x = magnitude;
			vel_msg.linear.y = 0;
			vel_msg.linear.z = 0; 

			vel_msg.angular.z = -1*max_angular_velocity * (new_angle - yaw);
			vel_msg.angular.y = 0;
			vel_msg.angular.x = 0;


        	this->velocity_pb.publish(vel_msg);

		}

		std::string getName(){
			return this->name;
		}


		

        void moveAgent(const ros::TimerEvent& event){

            gazebo_msgs::GetModelState srv = this->get_state(this->name);



            double roll,pitch,yaw;
        	tf::Quaternion q(
        	    srv.response.pose.orientation.x,
        	    srv.response.pose.orientation.y,
        	    srv.response.pose.orientation.z,
        	    srv.response.pose.orientation.w
        	);

        	tf::Matrix3x3 m(q);
        	m.getRPY(roll, pitch, yaw);


        	double x = srv.response.pose.position.x;
        	double y = srv.response.pose.position.y;
        	// Calculate the distance and angle to the goal
        	double goal_x = this->goal.x;
        	double goal_y = this->goal.y;


        	double dx = goal_x - x;
        	double dy = goal_y - y;


        	double distance_to_goal = sqrt(dx * dx + dy * dy);

        	double angle_to_goal = atan2(dy, dx);

			// std::cout<<"Roll: "<<roll<<std::endl;
			// std::cout<<"Pitch: "<<pitch<<std::endl;
			// std::cout<<"Yaw: "<<yaw<<std::endl;
			// std::cout<<std::endl;
			// std::cout<<"Angle to goal: "<<angle_to_goal<<std::endl;
			// std::cout<<std::endl;
        	// Calculate the linear and angular velocity towards the goal
        	geometry_msgs::Twist vel_msg;
        	const double max_linear_velocity = this->prefVel;
        	const double max_angular_velocity = 7;
        	vel_msg.linear.x = abs(max_linear_velocity * cos(angle_to_goal - yaw));
			vel_msg.linear.y = 0;
			vel_msg.linear.z = 0; 

			// double tmp = -1*max_angular_velocity * (angle_to_goal - yaw);

			// if(abs(tmp) > abs(4*acos(0.0) - abs(tmp))){
			// 	vel_msg.angular.z = -(4*acos(0.0) - tmp);
			// }
			// else{
			// 	vel_msg.angular.z = tmp;
			// }
			vel_msg.angular.z = -1*max_angular_velocity * (angle_to_goal - yaw);
			vel_msg.angular.y = 0;
			vel_msg.angular.x = 0;
        	// Publish the velocity command
			// std::cout<<"here"<<std::endl;
        	this->velocity_pb.publish(vel_msg);

			// geometry_msgs::Twist vel_msg1;
			// vel_msg1.linear.x = 0;
			// vel_msg1.linear.y = 0;
			// vel_msg1.linear.z = 0; 
        	// vel_msg1.angular.z = 20;
			// vel_msg1.angular.y = 0;
			// vel_msg1.angular.x = 0;

			// geometry_msgs::Twist vel_msg2;
			// vel_msg2.linear.x = 0;
			// vel_msg2.linear.y = 0;
			// vel_msg2.linear.z = 0; 
        	// vel_msg2.angular.z = -20;
			// vel_msg2.angular.y = 0;
			// vel_msg2.angular.x = 0;

			// this->velocity_pb.publish(vel_msg1);
			// sleep(4);
			// this->velocity_pb.publish(vel_msg2);
			// std::cout<<"here"<<std::endl;

            //Takes agent state and gives it its optimal velocity

            // Also checks for collisions

            // could be replaced by ORCA function


        }
		void updateGoal(){
        // void updateGoal(const ros::TimerEvent& event){

            // std::cout<<"Getting executed"<<std::endl;
			
			if(this->reachedGoal()){

				std::cout<<this->name<<" reached goal"<<std::endl;

                double rand_x = this->min_x + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (this->max_x - this->min_x)));
                double rand_y = this->min_y + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (this->max_y - this->min_y)));

				this->goal.x = rand_x;
				this->goal.y = rand_y;

				std::cout<<this->name<<" New Goal: ("<<this->goal.x<<","<<this->goal.y<<")"<<std::endl;
            }
        }



        bool reachedGoal(){
            gazebo_msgs::GetModelState srv = this->get_state(this->name);

            double agent_x = srv.response.pose.position.x;
            double agent_y = srv.response.pose.position.y;

            if(sqrt(pow((agent_x - this->goal.x),2) + pow((agent_y - this->goal.y),2)) < this->radius){
                return true;
            }
            return false;
        }


};



#endif