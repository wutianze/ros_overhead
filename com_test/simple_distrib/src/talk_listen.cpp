#include "ros/ros.h"
#include "std_msgs/String.h"
#include "simple_distrib/Test.h"
#include <fstream>
#include <time.h>
using namespace std;

ros::Publisher publisher;
simple_distrib::Test msg;
string to_send;
int64_t count_max;
int64_t send_count=0;
fstream writer;
void timerCallback(const ros::TimerEvent& e){
	//msg.id = count_num;
	//msg.timestamp = ros::Time::now().toNSec();
	msg.header.stamp = ros::Time::now();
	//ROS_INFO("time send:%d",msg.header.stamp.nsec);
	publisher.publish(msg);
	//ROS_INFO("publisher one");
}

void chatterCallback1(const simple_distrib::Test::ConstPtr& tmpMsg)
{
	writer<<(ros::Time::now() - tmpMsg->header.stamp).toNSec()<<endl;
	send_count++;
	if(send_count>=count_max){
		ROS_INFO("finished\n");
		writer.close();
		return;
	}
	msg.header.stamp = ros::Time::now();
	//ROS_INFO("time send:%d",msg.header.stamp.nsec);
	publisher.publish(msg);
	//ROS_INFO("publisher one");

}
int main(int argc, char **argv)
{
	writer.open("./log.txt",ios::trunc|std::ios::out);
	ros::init(argc, argv, "talker_listen");
	ros::NodeHandle n;

	int64_t str_size = atoll(argv[1]);
	count_max = atoll(argv[2]);

	publisher = n.advertise<simple_distrib::Test>("msg", 1);
	to_send = std::string(str_size,'a');  //1024 8192 16384 65536 262144 524288 1048576 2097152 4194304 8388608
	msg.content = to_send;
	
	ros::Subscriber sub = n.subscribe("msg_back", 1, chatterCallback1);   //TCP 
//msg.header.stamp = ros::Time::now();
//	publisher.publish(msg);
	ros::Timer timer = n.createTimer(ros::Duration(1),timerCallback);

	ros::spin();
	return 0;
}
