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
        wrenchb_temp.resize(6,0);  
        index=0;
        sensor_point=0;

        flag=false;
        wrench_sub = nh.subscribe("/filtered_wrench", 1000, &GravityIdentify::WrenchsubCallback,this);
       
        ros::Duration(5).sleep();
        int exit_code=urMove();
        if(exit_code!=0){
            std::cout<<"move failed"<<std::endl;
            return;
        }
        calculateP();
        calculateG();
        writeToYaml();
        ros::shutdown();
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

//四元数转旋转矩阵
Eigen::Matrix3d GravityIdentify::quaternion2Rotation(double x,double y,double z,double w){
    Eigen::Matrix3d R;
    R(0,0)=1-2*y*y-2*z*z;
    R(0,1)=2*(x*y-z*w);
    R(0,2)=2*(x*z+y*w);
    R(1,0)=2*(x*y+z*w);
    R(1,1)=1-2*x*x-2*z*z;
    R(1,2)=2*(y*z-x*w);
    R(2,0)=2*(x*z-y*w);
    R(2,1)=2*(y*z+x*w);
    R(2,2)=1-2*x*x-2*y*y;
    return R;
}


void GravityIdentify::WrenchsubCallback(const geometry_msgs::WrenchStamped& msg){
    if(flag){
        sensor_point++;
        wrenchb_temp[0] += msg.wrench.force.x;
        wrenchb_temp[1] += msg.wrench.force.y;
        wrenchb_temp[2] += msg.wrench.force.z;
        wrenchb_temp[3] += msg.wrench.torque.x;
        wrenchb_temp[4] += msg.wrench.torque.y;
        wrenchb_temp[5] += msg.wrench.torque.z;
        if(sensor_point==5000){
            // calculate 100 times force average date
            for(int i=0;i<6;i++){
                wrenchb_temp[i]/=100;
                std::cout<<wrenchb_temp[i]<<" ";
            }
            std::cout<<std::endl;
            Eigen::Matrix<double,3,1> M_temp;
            Eigen::Matrix<double,3,1> F_temp;
            M_temp<<wrenchb_temp[3],wrenchb_temp[4],wrenchb_temp[5];
            F_temp<<wrenchb_temp[0],wrenchb_temp[1],wrenchb_temp[2];
            M.block(index*3,0,3,1)=M_temp;
            F.block(index*3,0,3,1)=F_temp;
            for(int i=0;i<6;i++){
                wrenchb_temp[i]=0;
            }           
            getSE3();
            sensor_point=0;
            flag=false;     
        }
    }
}


//利用tf_listener 查询机器人末端相对于基座的位姿
void GravityIdentify::getSE3(){
    tf::StampedTransform transform;
    try{
        listener.lookupTransform("base", "tool0",ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    double x=transform.getRotation().getX();
    double y=transform.getRotation().getY();
    double z=transform.getRotation().getZ();
    double w=transform.getRotation().getW();

    R.block(3*index,0,3,3)=quaternion2Rotation(x,y,z,w).transpose();
}

//获取向量对应的反对称矩阵
Eigen::Matrix3d GravityIdentify::getAntisymmetric(Eigen::Matrix<double,3,1> v){
    Eigen::Matrix3d temp;
    temp<<0,v(2,0),-v(1,0),
          -v(2,0),0,v(0,0),
          v(1,0),-v(0,0),0;
    return temp;
}

//计算负载的质心
void GravityIdentify::calculateP(){
    Eigen::Matrix<double,pos_num*3,6> E;
    Eigen::Matrix<double,3,3> I33=Eigen::Matrix3d::Identity();  
    for(int i=0;i<pos_num;i++){       
        E.block(i*3,0,3,3)=getAntisymmetric(F.block(i*3,0,3,1));
        E.block(i*3,3,3,3)=I33;
    }

    p=((E.transpose()*E).inverse())*E.transpose()*M;
    std::cout<<"the trans of gravity is\n"<<p<<std::endl;
}

//计算负载的质量以及力矩传感器的零点
void GravityIdentify::calculateG(){
    
    Eigen::Matrix<double,pos_num*3,6> E;
    Eigen::Matrix3d I33=Eigen::Matrix3d::Identity();
    E.block(0,0,pos_num*3,3)=R;
    for(int i=0;i<pos_num;i++) E.block(i*3,3,3,3)=I33;
    G=(E.transpose()*E).inverse()*E.transpose()*F;
    std::cout<<"the gravity and zero point of sensor is\n"<<G<<std::endl;
}

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
        flag=true;
        while(flag) continue;
        ros::Duration(5).sleep();
        index++;
    }
    return 0;
}

//将标识出的参数写入yaml文件
void GravityIdentify::writeToYaml() const
{
    std::string dir_package,dir_param_file;
    dir_package = ros::package::getPath("gravity_compensate");
    dir_param_file = dir_package + "/config/param.yaml";
    std::ofstream fout(dir_param_file);

    YAML::Emitter out(fout);
    out << YAML::BeginMap;
    out << YAML::Key << "pos";
    out << YAML::BeginSeq;
    out << YAML::Value << p(0,0);
    out << YAML::Value << p(1,0);
    out << YAML::Value << p(2,0);
    out << YAML::EndSeq;
    out << YAML::Key << "Gm";
    out << YAML::BeginSeq;
    out << YAML::Value << G(0,0);
    out << YAML::Value << G(1,0);
    out << YAML::Value << G(2,0);
    out << YAML::EndSeq;
    out << YAML::Key << "F0";
    out << YAML::BeginSeq;
    out << YAML::Value << G(3,0);
    out << YAML::Value << G(4,0);
    out << YAML::Value << G(5,0);
    out << YAML::Value << p(3,0)-G(4,0)*p(2,0)+G(5,0)*p(1,0);
    out << YAML::Value << p(4,0)-G(5,0)*p(0,0)+G(3,0)*p(2,0);
    out << YAML::Value << p(5,0)-G(3,0)*p(1,0)+G(4,0)*p(0,0);
    out << YAML::EndSeq;
    out << YAML::EndMap;
    fout.close();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "GravityIdentify");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    GravityIdentify gc;
    return 0;
}
