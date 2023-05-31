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
xarm_planner::single_straight_plan srv22;



float posx = 0.3;
float posy = 0.4;
float posz = 0.2;


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

void cb_input(const geometry_msgs::Pose::ConstPtr& msg)
{
  	posx = msg->position.x;
  	posy = msg->position.y;
  	posz = msg->position.z;
	
	ROS_INFO("PRINT 1 ---> x: %f, y: %f, z: %f", posx, posy, posz);
	
}


int main(int argc, char** argv)
{	
	ros::init(argc, argv, "xarm_simple_planner_client");
	ros::NodeHandle nh;
	
	ros::ServiceClient client = nh.serviceClient<xarm_planner::joint_plan>("xarm_joint_plan");
	ros::ServiceClient client_exec = nh.serviceClient<xarm_planner::exec_plan>("xarm_exec_plan");
	ros::ServiceClient client22 = nh.serviceClient<xarm_planner::pose_plan>("xarm_straight_plan");
	

	ros::Subscriber sub = nh.subscribe("pose", 1000, cb_input);

	xarm_planner::joint_plan srv;
	xarm_planner::exec_plan srv_exec;
	ros::Rate loop_rate(10);

    while (ros::ok())
    {
       	ROS_INFO("PRINT 2 ---> x: %f, y: %f, z: %f", posx, posy, posz);
		target.position.x = posx;
		target.position.y = posy;
		target.position.z = posz;

		target.orientation.x = 1;
		target.orientation.y = 0;
		target.orientation.z = 0;
		target.orientation.w = 0;

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

