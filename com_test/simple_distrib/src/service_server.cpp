#include "ros/ros.h"
#include "std_msgs/String.h"
#include "simple_distrib/TimeCount.h"
#include <fstream>
#include <vector>
#include <signal.h>
#include<numeric>
using namespace std;

ros::Publisher pub;

string node_name = ros::this_node::getName();


bool callback(simple_distrib::TimeCount::Request &req,
		simple_distrib::TimeCount::Response &res)
{
	res.content = req.content;
	res.header = req.header;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_server");
	ros::NodeHandle n;
	
	ros::ServiceServer service = n.advertiseService("time_count",callback);
	ROS_INFO("ready");
	ros::spin();
	
	return 0;
}
