#include <iostream>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <cmath>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>


const std::string PLANNING_GROUP = "manipulator";
const int pos_num=9;

class GravityIdentify
{
public:
    GravityIdentify():move_group(PLANNING_GROUP)
    {       
        ros::Duration(5).sleep();
        int exit_code=urMove();
        if(exit_code!=0){
            std::cout<<"move failed"<<std::endl;
            return;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber wrench_sub;


    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    tf::TransformListener listener;

    Eigen::Matrix<double,pos_num*3,1> F;
    Eigen::Matrix<double,pos_num*3,1> M;
    Eigen::Matrix<double,pos_num*3,3> R;

    std::vector<double> wrenchb_temp;
   

    Eigen::Matrix<double,6,1> p;
    Eigen::Matrix<double,6,1> G;

    bool flag;
    int sensor_point; 
    int index;
    void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg);
    void getSE3();

    Eigen::Matrix3d getAntisymmetric(Eigen::Matrix<double,3,1> v);
    Eigen::Matrix3d quaternion2Rotation(double x,double y,double z,double w);

    void calculateP();
    void calculateG();
    int urMove();
    void writeToYaml() const;
};

//利用Moveit控制机器人进行运动
int GravityIdentify::urMove()
{
    std::string pos="pose";

    // 修改目标容差值
    double new_tolerance = 0.01;  // 替换为你想设置的目标容差值


    // 设置新的目标容差
    move_group.setGoalJointTolerance(new_tolerance);
    for(int i=1;i<=pos_num;i++){
        std::cout<<"prepare to plan for point"<<i<<std::endl;
        move_group.setNamedTarget(pos+std::to_string(i));
        bool plan_success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!plan_success) return -1;
        bool execute_success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!execute_success) return -1;
        ros::Duration(5).sleep();
    }
    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "TestGravityCompensate");
    GravityIdentify gc;
    return 0;
}