#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <xarm_planner/pose_plan.h>
#include <xarm_planner/joint_plan.h>
#include <xarm_planner/exec_plan.h>
#include <xarm_planner/single_straight_plan.h>
#include <stdlib.h>
#include <geometry_msgs/Pose.h>
#include <vector>

geometry_msgs::Pose target;
float diff_x = 0;
float diff_y = 0;
float diff_z = 0;
float last_z = 0;


bool request_plan(ros::ServiceClient& client, xarm_planner::joint_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service joint_plan");
		return false;
	}
}

bool request_plan(ros::ServiceClient& client, xarm_planner::pose_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service pose_plan");
		return false;
	}
}

bool request_plan(ros::ServiceClient& client, xarm_planner::single_straight_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service single_straight_plan");
		return false;
	}
}

bool request_exec(ros::ServiceClient& client, xarm_planner::exec_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service exec_plan");
		return false;
	}
}

void cb_diff_input(const geometry_msgs::Pose::ConstPtr& msg){
	diff_x = msg->position.x;
  	diff_y = msg->position.y;
	diff_z = msg->position.z;
	//ROS_INFO("ERROR IN: x: %f, y: %f, z: %f", diff_x, diff_y, diff_z);
	//ROS_INFO("ENTRA CALLBACK");
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "xarm_simple_planner_client");
	ros::NodeHandle nh;
	
	ros::ServiceClient client = nh.serviceClient<xarm_planner::joint_plan>("xarm_joint_plan");
	ros::ServiceClient client_exec = nh.serviceClient<xarm_planner::exec_plan>("xarm_exec_plan");
	ros::ServiceClient client22 = nh.serviceClient<xarm_planner::pose_plan>("xarm_straight_plan");
	
	ros::Subscriber sub_diff = nh.subscribe("diff_pose", 1000, cb_diff_input);

	xarm_planner::joint_plan srv;
	xarm_planner::exec_plan srv_exec;
	ros::Rate loop_rate(200);
	xarm_planner::single_straight_plan srv22;

	target.position.x = 0.2;
	target.position.y = 0.05;
	target.position.z = 0.2;

	target.orientation.x = 1;
	target.orientation.y = 0;
	target.orientation.z = 0;
	target.orientation.w = 0;

	srv22.request.target = target;
	if(request_plan(client22, srv22))
	{
		ROS_INFO("INITIALIZE! Executing... ");
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}

	float epsilon = 50;
	float epsilon_z = 1000;

    while (ros::ok())
    {
		if(abs(diff_x)<epsilon && abs(diff_y)<epsilon){
			ROS_INFO("CENTRO");
			target.position.x = target.position.x;
			target.position.y = target.position.y;
		}
		else if(diff_x>0 && diff_y>0){
			ROS_INFO("Q1");
			target.position.x = target.position.x + 0.01;
			target.position.y = target.position.y - 0.01;
		}
		else if(diff_x<0 && diff_y>0){
			ROS_INFO("Q2");
			target.position.x = target.position.x + 0.01;
			target.position.y = target.position.y + 0.01;
		}
		else if(diff_x<0 && diff_y<0){
			ROS_INFO("Q3");
			target.position.x = target.position.x - 0.01;
			target.position.y = target.position.y + 0.01;
		}
		else if(diff_x>0 && diff_y<0){
			ROS_INFO("Q4");
			target.position.x = target.position.x - 0.01;
			target.position.y = target.position.y - 0.01;
		}	

		// if(diff_z > 42000){
		// 	target.position.z = target.position.z + 0.01;
		// }else if(last_z < diff_z && diff_z < 20000){
		// 	target.position.z = target.position.z - 0.01;
		// }
		// last_z = target.position.z;

		srv22.request.target = target;
		if(request_plan(client22, srv22))
		{
			ROS_INFO("Plan SUCCESS! Executing... ");
			srv_exec.request.exec = true;
			request_exec(client_exec, srv_exec);
		}
        ros::spinOnce();
        loop_rate.sleep();  
    }

	
}

