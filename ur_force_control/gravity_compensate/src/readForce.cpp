#include<ros/ros.h>
#include<geometry_msgs/WrenchStamped.h>
#include<iostream>
#include<fstream>

using namespace std;

ofstream outfile;

void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg) {
    outfile<<msg.wrench.force.x<<" "<<msg.wrench.force.y<<" "<<msg.wrench.force.z<<" "<<msg.wrench.torque.x<<" "<<msg.wrench.torque.y<<" "<<msg.wrench.torque.z<<endl;
}

int main(int argc,char** argv) {
    outfile.open("/home/hxq/forcedata.txt",ios::out | ios::trunc);
    ros::init(argc,argv,"force_data");
    ros::NodeHandle nh;

    ros::Subscriber wrench_sub = nh.subscribe("/compensate_wrench_tool",1000,WrenchsubCallback);
    ros::spin();
    return 0;
}