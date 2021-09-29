#include "ros/ros.h"
#include "simple_distrib/TimeCount.h"
#include <cstdlib>
#include <fstream>

using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_client");
fstream writer;
	writer.open("./log.txt",ios::trunc|std::ios::out);
  ros::NodeHandle n;
  int64_t str_size = atoll(argv[1]);
int64_t	count_max = atoll(argv[2]);


  ros::ServiceClient client = n.serviceClient<simple_distrib::TimeCount>("time_count");
  simple_distrib::TimeCount srv;

  srv.request.content = std::string(str_size,'a');

int64_t send_count=0;
  while(send_count<count_max){
	  srv.request.header.stamp = ros::Time::now();
  if (client.call(srv))
  {
	  writer<<(ros::Time::now()-srv.response.header.stamp).toNSec()<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service time_count");
    return 1;
  }
  send_count++;
  }
  writer.close();
  return 0;
}
