#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#define pi 3.1416
#define a1 0
#define a2 59.2
#define a3 77.8


enum LINK_LIST
{
    head = 0,
    l_arm1,
    l_arm2,
    l_arm3,
    r_arm1,
    r_arm2,
    r_arm3,
    link_num,
};

enum ARM_ID
{
    left_arm = 0,
    right_arm = 3,
};

typedef struct
{
    float x;
    float y;
    float z;
}Position;

class Robot
{
private:
    bool stop_flag;//无解时置此标志，停止发布弧度消息
    bool left_last_singularity;//左臂处于奇异状态
    bool right_last_singularity;//右臂处于奇异状态
    ros::Publisher joint_pub[link_num];//每个关节对应的发布
    std_msgs::Float64 joint_msg[link_num];//每个关节对应的消息
    float target_rad[link_num];//目标弧度
    float current_rad[link_num];//当前弧度
    //float send_rad[link_num];//
    float min_rad[link_num];//最小弧度限制
    float max_rad[link_num];//最大弧度限制

    Position left_current_position;
    Position right_current_position;
public:
    Robot(ros::NodeHandle& n);
    void transform();
    void robot_action();
    void get_position(ARM_ID arm_id,Position& position);
    void set_position(ARM_ID arm_id,Position& position);
};

#endif

