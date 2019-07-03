#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "control.h"


Robot::Robot(ros::NodeHandle& n)
{
    joint_pub[head] = n.advertise<std_msgs::Float64>("/yanshee/head_position_controller/command",100);
    joint_pub[l_arm1] = n.advertise<std_msgs::Float64>("/yanshee/left_arm1_position_controller/command",100);
    joint_pub[l_arm2] = n.advertise<std_msgs::Float64>("/yanshee/left_arm2_position_controller/command",100);
    joint_pub[l_arm3] = n.advertise<std_msgs::Float64>("/yanshee/left_arm3_position_controller/command",100);
    joint_pub[r_arm1] = n.advertise<std_msgs::Float64>("/yanshee/right_arm1_position_controller/command",100);
    joint_pub[r_arm2] = n.advertise<std_msgs::Float64>("/yanshee/right_arm2_position_controller/command",100);
    joint_pub[r_arm3] = n.advertise<std_msgs::Float64>("/yanshee/right_arm3_position_controller/command",100);

    min_rad[head] = -0.78;  max_rad[head] = 0.78;
    min_rad[l_arm1] = -1.58;max_rad[l_arm1] = 1.58;
    min_rad[l_arm2] = 0;max_rad[l_arm2] = 3.15;
    min_rad[l_arm3] = -1.58;max_rad[l_arm3] = 1.58;
    min_rad[r_arm1] = -1.58;max_rad[r_arm1] = 1.58;
    min_rad[r_arm2] = 0;max_rad[r_arm2] = 3.15;
    min_rad[r_arm3] = -1.58;max_rad[r_arm3] = 1.58;

    target_rad[head] = 0;
    target_rad[l_arm1] = 0;
    target_rad[l_arm2] = 1.5708;
    target_rad[l_arm3] = 0;
    target_rad[r_arm1] = 0;
    target_rad[r_arm2] = 1.5708;
    target_rad[r_arm3] = 0;

    left_current_position.x = 0;
    left_current_position.y = 0;
    left_current_position.z = a2+a3;
    right_current_position.x = 0;
    right_current_position.y = 0;
    right_current_position.z = a2+a3;

    stop_flag = false;

}

/************************************************************
Function:transform()
Description:由于urdf中机器人模型的坐标系不一定和我们期望的一致，所以在
这里将current_rad[i]和joint_msg[i].data做一次转换，如果坐标系一致则
current_rad[]和joint_msg[i].data相等。
*************************************************************/
void Robot::transform()
{
    joint_msg[head].data = current_rad[head];
    joint_msg[l_arm1].data = current_rad[l_arm1];
    joint_msg[l_arm2].data = current_rad[l_arm2];
    joint_msg[l_arm3].data = current_rad[l_arm3];
    joint_msg[r_arm1].data = current_rad[r_arm1];
    joint_msg[r_arm2].data = current_rad[r_arm2];
    joint_msg[r_arm3].data = current_rad[r_arm3];
}

/************************************************************
Function:robot_action()
Description:将target_rad[]限制在合适范围内，同步到current_rad[]中，经过
transform()转换给joint_msg[i].data并发布给gazebo中每个关节的话题
*************************************************************/
void Robot::robot_action()
{
    int wait_time = 0,temp = 0;

    if(stop_flag)
    {
        return;//如果无解时不再运行action
    }


    for(int i = 0;i<link_num;i++)
    {
        if(target_rad[i] < min_rad[i])
            current_rad[i] = min_rad[i];
        else if(target_rad[i] > max_rad[i])
            current_rad[i] = max_rad[i];
        else
        {
            current_rad[i] = target_rad[i];
        }
        transform();
        joint_pub[i].publish(joint_msg[i]);
    }

}

/************************************************************
Function:get_position(ARM_ID arm_id,Position& position)
Description:得到手臂末端的坐标
Input:
    arm_id:left_arm/right_arm  选择左臂或者右臂
    position:Position引用类型，通过此引用将坐标传出
Output:通过引用position将坐标传出
*************************************************************/
void Robot::get_position(ARM_ID arm_id,Position& position)
{
    float theta_1,theta_2,theta_3;
    if(arm_id == left_arm)
    {
        theta_1 = current_rad[l_arm1];
        theta_2 = current_rad[l_arm2];
        theta_3 = current_rad[l_arm3];
    }
    else if(arm_id == right_arm)
    {
        theta_1 = current_rad[r_arm1];
        theta_2 = current_rad[r_arm2];
        theta_3 = current_rad[r_arm3];
    }
    else
    {
        ROS_INFO("ARM_ID：%d error!",arm_id);
        return;
    }
    position.x = cos(theta_1)*((a2*cos(theta_2))+(a3*(cos(theta_2+theta_3))));
    position.y = sin(theta_1)*((a2*cos(theta_2))+(a3*(cos(theta_2+theta_3))));
    position.z = a2*sin(theta_2)+(a3*(sin(theta_2+theta_3)));
}


/************************************************************
Function:set_position(ARM_ID arm_id,Position& position)
Description:设置手臂末端的坐标
Input:
    arm_id:left_arm/right_arm  选择左臂或者右臂
    position:Position引用类型，通过此引得到位置输入
*************************************************************/
void Robot::set_position(ARM_ID arm_id,Position& position)
{
    float th1,  //第一关节角
          th2a, //第二关节角a解
          th3a, //第三关节角a解
          th2b, //第二关节角b解
          th3b, //第三关节角b解
          i_2,  //中间变量，坐标原点到末端的距离的平方
          i,    //中间变量，坐标原点到末端的距离
          th21, //求第二关节角的中间变量
          th22, //求第二关节角的中间变量
          offset_a,//a解距离当前关节位置的偏差
          offset_b;//b解距离当前关节位置的偏差

    stop_flag = false;

    i_2 = position.x*position.x+position.y*position.y+position.z*position.z;
    i = sqrt(i_2);

//计算相应的角度
    th1 = atan(position.y/position.x);
    th21 = acos(((a2*a2)-(a3*a3)+i_2)/(2*a2*i));  //没有多解的情况，但是会出现无解的情况
    if(th21 < 0 || th21 > pi)//无解
    {
      ROS_INFO("position error!");
      stop_flag = true;//无解则停止action
      return;
    }

    if(position.x >=0)  //th22的多解问题，通过判断坐标区域选择解，实际最终只有一个解
        th22 = asin(position.z/i);
    else
        th22 = pi-(asin(position.z/i));
//a，b两组解
    th2a = th21+th22;
    th3a = acos(((a2*a2)+(a3*a3)-i_2)/(2*a2*a3));
    th3a = th3a - pi;

    th2b = th22 - th21;
    th3b = -th3a;

//验证两组解的可行性

    if((th2a >= min_rad[arm_id+2])&&(th2a <= max_rad[arm_id+2])&&(th3a >= min_rad[arm_id+3])&&(th3a <= max_rad[arm_id+3]))
        offset_a = fabs(current_rad[arm_id+2]-th2a)+fabs(current_rad[arm_id+3]-th3a);
    else
        offset_a = 100;//置一个大的偏差使不在限制范围内的解不会被选到
    if((th2b >= min_rad[arm_id+2])&&(th2b <= max_rad[arm_id+2])&&(th3b >= min_rad[arm_id+3])&&(th3b <= max_rad[arm_id+3]))
        offset_b = fabs(current_rad[arm_id+2]-th2b)+fabs(current_rad[arm_id+3]-th3b);
    else
        offset_b = 100;//置一个大的偏差使不在限制范围内的解不会被选到
    if(offset_a == 100 && offset_b == 100)
    {
        ROS_INFO("position limit error!");
        stop_flag = true;//所有的解都不再限制范围内也停止action
        return;
    }
//选择可行且总旋转弧度小的那个解
    if(offset_a <= offset_b)
    {
        target_rad[arm_id+1] = th1;
        target_rad[arm_id+2] = th2a;
        target_rad[arm_id+3] = th3a;
    }else
    {
        target_rad[arm_id+1] = th1;
        target_rad[arm_id+2] = th2b;
        target_rad[arm_id+3] = th3b;
    }

    switch (arm_id)
    {
        case left_arm:
            left_current_position.x = position.x;
            left_current_position.y = position.y;
            left_current_position.z = position.z;
            break;
        case right_arm:
            right_current_position.x = position.x;
            right_current_position.y = position.y;
            right_current_position.z = position.z;
            break;
        default:
            break;
    }

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"control");
    ros::NodeHandle yanshee_node;
    ros::NodeHandle& n = yanshee_node;
    Robot yanshee(n);
    Position position = {100.0,-100,10};
    Position current_position;
    float time = 0;
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        yanshee.robot_action();
        yanshee.set_position(left_arm,position);
        yanshee.set_position(right_arm,position);
        yanshee.get_position(left_arm,current_position);
        ROS_INFO("position(%f,%f,%f)",current_position.x,current_position.y,current_position.z);
        time+=0.01;
        position.y = 90*(sin(time));
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
