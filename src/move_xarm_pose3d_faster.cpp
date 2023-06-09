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
float diff_x, diff_y, diff_z;
float last_z = 0;


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
	ROS_INFO("ERROR IN: x: %f, y: %f, z: %f", diff_y, diff_y, diff_z);
	//ROS_INFO("ENTRA CALLBACK");
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "xarm_simple_planner_client");
	ros::NodeHandle nh;
	
	ros::ServiceClient client_pose = nh.serviceClient<xarm_planner::pose_plan>("xarm_pose_plan");
	ros::ServiceClient client_exec = nh.serviceClient<xarm_planner::exec_plan>("xarm_exec_plan");
	
	ros::Subscriber sub_diff = nh.subscribe("diff_pose", 1000, cb_diff_input);

	xarm_planner::exec_plan srv_exec;
	xarm_planner::pose_plan srv_pose;
	
	ros::Rate loop_rate(10);
	
	target.position.x = 0.3;
	target.position.y = 0.0;
	target.position.z = 0.0;
	
	target.orientation.x = 1;
	target.orientation.y = 0;
	target.orientation.z = 0;
	target.orientation.w = 0;

	srv_pose.request.target = target;
	if(request_plan(client_pose, srv_pose))
	{
		ROS_INFO("INITIALIZE! Executing... ");
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}

    while (ros::ok())
    {
		xarm_planner::exec_plan srv_exec;
		xarm_planner::pose_plan srv_pose;

		target.position.x = -diff_y+0.3;
		target.position.y = -diff_x;
		
		if (diff_z < 0){	
			target.position.z = 0.3;
		}else{
			target.position.z = diff_z;
		}
		

		srv_pose.request.target = target;
		if(request_plan(client_pose, srv_pose))
		{
			ROS_INFO("Planning done! Executing... ");
			srv_exec.request.exec = true;
			request_exec(client_exec, srv_exec);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
    }

	
}

