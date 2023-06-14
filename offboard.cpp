//square fly --base

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/Image.h>
//#include <cv_bridge/cv_bridge.h>

#include "utils/TTPDrone.hpp"
#include "utils/TTPVisualServo.hpp"
#include "utils/TTPVisualization.hpp"

#include "demo/BoundingBox.h"
#include "demo/BoundingBoxes.h"
#include "demo/PositionCommand.h"
const double D = 1.5;//正方形边长
const double HZ = 30.0;
const double T_S = 1.0/(HZ*10.0); // 18秒1米
const double E_S = 10.0/HZ; // 1秒中10度

std::vector<Eigen::Vector3d> visual_path;
std::vector<Eigen::Vector3d> visual_keypoints;

ros::Publisher path_visualization_pub;
ros::Publisher keypoints_visualization_pub;

double ego_cmd_x, ego_cmd_y, ego_cmd_z, ego_cmd_yaw;
uint8_t ego_flag_int;
bool ego_flag_bool;

void vis_cb(const ros::TimerEvent& e)
{
    TTP::Visualization::displayMarkerList(path_visualization_pub, visual_path, 0.1, Eigen::Vector4d(0.0,0.9,0.0,1.0), 1);
    TTP::Visualization::displayMarkerListNotLine(keypoints_visualization_pub, visual_keypoints, 0.2, Eigen::Vector4d(0.9,0.0,0.0,1.0), 2);
}
 
void ego_cmd_cb(const demo::PositionCommand::ConstPtr& ptr)
{
    ego_cmd_x = ptr->position.x;
    ego_cmd_y = ptr->position.y;
    ego_cmd_z =ptr->position.z;
    ego_cmd_yaw = ptr->yaw;
    ego_flag_int = ptr->trajectory_flag;
    ego_flag_bool = true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle node("~");
    ros::Rate rate(HZ);

    ros::Subscriber ego_sub= node.subscribe<demo::PositionCommand>("/planner/pos_cmd",30,ego_cmd_cb);
    ros::Publisher ego_pub = node.advertise<geometry_msgs::PoseStamped>("/planner/goal",30);

    path_visualization_pub = node.advertise<visualization_msgs::Marker>("/drone_path_visualization",30);
    keypoints_visualization_pub = node.advertise<visualization_msgs::Marker>("/drone_keypoints_visualization",30);


    ros::Timer vis_timer = node.createTimer(ros::Duration(0.05), vis_cb);
    TTP::Drone::S_Ptr drone = TTP::Drone::get_instance(node);
    
    //init 5s
    uint32_t init_counter = HZ*5;
    while(!drone->get_state_ok() && init_counter--)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("drone state init.");
    }

    //take off
    const float takeoff_z = 0.6;
    //const float takeoff_z = 0.2;
    geometry_msgs::PoseStamped segment_start_pose, current_pose;
    geometry_msgs::Pose target_pose;
    drone->get_current_pose(segment_start_pose);
    drone->get_current_pose(current_pose);
    target_pose = segment_start_pose.pose;
    target_pose.position.z = takeoff_z;

    visual_keypoints.push_back(Eigen::Vector3d(segment_start_pose.pose.position.x, segment_start_pose.pose.position.y, segment_start_pose.pose.position.z));
    visual_keypoints.push_back(Eigen::Vector3d(target_pose.position.x, target_pose.position.y, target_pose.position.z));


    drone->get_current_pose(segment_start_pose);
    double vs_target_hit_yaw = drone->get_yaw(segment_start_pose.pose);
    geometry_msgs::Pose front_pose;
    Eigen::Vector3d front_start_vector;
    uint32_t time_index = 0;

    while(node.ok())
    {
        ros::spinOnce();
        rate.sleep();

        // update
        drone->get_current_pose(current_pose);

        // cmd
        drone->set_pose(target_pose);
        drone->show_pose_info("taking off", current_pose.pose);
        
        // break
        if(drone->reach_pose(target_pose))
        {
            break;
        }
    }

    //front to 1.5
    const double T_T_S_2 = T_S*1.8;
    drone->get_current_pose(segment_start_pose);
    time_index = 0.0;
    while(node.ok())
    {
        ros::spinOnce();
        rate.sleep();
        // update
        drone->get_current_pose(current_pose);
        //task
        const double t = time_index*T_T_S_2 >= 1.0 ? 1.0 : time_index*T_T_S_2;
        const double x = segment_start_pose.pose.position.x + 1.5 * t; //向前飞1.5米
        const double y = segment_start_pose.pose.position.y;
        const double z = segment_start_pose.pose.position.z;
         drone->set_pose(x,y,z, drone->get_roll(segment_start_pose.pose), drone->get_pitch(segment_start_pose.pose), drone->get_yaw(segment_start_pose.pose));
        drone->show_pose_info("front to D m", current_pose.pose);
        time_index++;
        if(t == 1.0){
            break;
        }
    }

    //yaw1

    const double T_T_S_3 = T_S*1.8;
    drone->get_current_pose(segment_start_pose);
    time_index = 0.0;
    while(node.ok())
    {
        ros::spinOnce();
        rate.sleep();
        // update
        drone->get_current_pose(current_pose);
        const double t = time_index*T_T_S_3 >= 1.0 ? 1.0 : time_index*T_T_S_3;
        //const double x = segment_start_pose.pose.position.x；
        //const double y = segment_start_pose.pose.position.y;
        //const double z = segment_start_pose.pose.position.z;
        double vs_target_hit_yaw = drone->get_yaw(segment_start_pose.pose);
        vs_target_hit_yaw += 90;
         drone->set_pose(segment_start_pose.pose.position.x, segment_start_pose.pose.position.y, segment_start_pose.pose.position.z, 0.0, 0.0, vs_target_hit_yaw);
        drone->show_pose_info("yaw1", current_pose.pose);
        time_index++;
        if(t == 1.0){
            break;
        }
    }


    //front to 1.5
    const double T_T_S_4 = T_S*1.8;
    drone->get_current_pose(segment_start_pose);
    time_index = 0.0;
    while(node.ok())
    {
        ros::spinOnce();
        rate.sleep();
        // update
        drone->get_current_pose(current_pose);
        //task
        const double t = time_index*T_T_S_4 >= 1.0 ? 1.0 : time_index*T_T_S_4;
        const double x = segment_start_pose.pose.position.x ; //向前飞1.5米
        const double y = segment_start_pose.pose.position.y+ 1.5 * t;
        const double z = segment_start_pose.pose.position.z;
         drone->set_pose(x,y,z, drone->get_roll(segment_start_pose.pose), drone->get_pitch(segment_start_pose.pose), drone->get_yaw(segment_start_pose.pose));
        drone->show_pose_info("front to D m", current_pose.pose);
        time_index++;
        if(t == 1.0){
            break;
        }
    }
    //yaw2
    const double T_T_S_5 = T_S*1.8;
    drone->get_current_pose(segment_start_pose);
    time_index = 0.0;
    while(node.ok())
    {
        ros::spinOnce();
        rate.sleep();
        // update
        drone->get_current_pose(current_pose);
        const double t = time_index*T_T_S_5 >= 1.0 ? 1.0 : time_index*T_T_S_5;
        //const double x = segment_start_pose.pose.position.x；
        //const double y = segment_start_pose.pose.position.y;
        //const double z = segment_start_pose.pose.position.z;
        double vs_target_hit_yaw = drone->get_yaw(segment_start_pose.pose);
        vs_target_hit_yaw += 90;
         drone->set_pose(segment_start_pose.pose.position.x, segment_start_pose.pose.position.y, segment_start_pose.pose.position.z, 0.0, 0.0, vs_target_hit_yaw);
        drone->show_pose_info("yaw1", current_pose.pose);
        time_index++;
        if(t == 1.0){
            break;
        }
    }


    //front to 1.5
    const double T_T_S_6 = T_S*1.8;
    drone->get_current_pose(segment_start_pose);
    time_index = 0.0;
    while(node.ok())
    {
        ros::spinOnce();
        rate.sleep();
        // update
        drone->get_current_pose(current_pose);
        //task
        const double t = time_index*T_T_S_6 >= 1.0 ? 1.0 : time_index*T_T_S_6;
        const double x = segment_start_pose.pose.position.x - 1.5 * t; //向前飞1.5米
        const double y = segment_start_pose.pose.position.y;
        const double z = segment_start_pose.pose.position.z;
         drone->set_pose(x,y,z, drone->get_roll(segment_start_pose.pose), drone->get_pitch(segment_start_pose.pose), drone->get_yaw(segment_start_pose.pose));
        drone->show_pose_info("front to D m", current_pose.pose);
        time_index++;
        if(t == 1.0){
            break;
        }
    }
    //yaw3
    const double T_T_S_7 = T_S*1.8;
    drone->get_current_pose(segment_start_pose);
    time_index = 0.0;
    while(node.ok())
    {
        ros::spinOnce();
        rate.sleep();
        // update
        drone->get_current_pose(current_pose);
        const double t = time_index*T_T_S_7 >= 1.0 ? 1.0 : time_index*T_T_S_7;
        //const double x = segment_start_pose.pose.position.x；
        //const double y = segment_start_pose.pose.position.y;
        //const double z = segment_start_pose.pose.position.z;
        double vs_target_hit_yaw = drone->get_yaw(segment_start_pose.pose);
        vs_target_hit_yaw += 90;
         drone->set_pose(segment_start_pose.pose.position.x, segment_start_pose.pose.position.y, segment_start_pose.pose.position.z, 0.0, 0.0, vs_target_hit_yaw);
        drone->show_pose_info("yaw1", current_pose.pose);
        time_index++;
        if(t == 1.0){
            break;
        }
    }

    //front to 1.5
    const double T_T_S_8 = T_S*1.8;
    drone->get_current_pose(segment_start_pose);
    time_index = 0.0;
    while(node.ok())
    {
        ros::spinOnce();
        rate.sleep();
        // update
        drone->get_current_pose(current_pose);
        //task
        const double t = time_index*T_T_S_8 >= 1.0 ? 1.0 : time_index*T_T_S_8;
        const double x = segment_start_pose.pose.position.x; //向前飞1.5米
        const double y = segment_start_pose.pose.position.y- 1.5 * t;
        const double z = segment_start_pose.pose.position.z;
         drone->set_pose(x,y,z, drone->get_roll(segment_start_pose.pose), drone->get_pitch(segment_start_pose.pose), drone->get_yaw(segment_start_pose.pose));
        drone->show_pose_info("front to D m", current_pose.pose);
        time_index++;
        if(t == 1.0){
            break;
        }
    }
    //yaw3
    const double T_T_S_9 = T_S*1.8;
    drone->get_current_pose(segment_start_pose);
    time_index = 0.0;
    while(node.ok())
    {
        ros::spinOnce();
        rate.sleep();
        // update
        drone->get_current_pose(current_pose);
        const double t = time_index*T_T_S_9 >= 1.0 ? 1.0 : time_index*T_T_S_9;
        //const double x = segment_start_pose.pose.position.x；
        //const double y = segment_start_pose.pose.position.y;
        //const double z = segment_start_pose.pose.position.z;
        double vs_target_hit_yaw = drone->get_yaw(segment_start_pose.pose);
        vs_target_hit_yaw += 90;
         drone->set_pose(segment_start_pose.pose.position.x, segment_start_pose.pose.position.y, segment_start_pose.pose.position.z, 0.0, 0.0, vs_target_hit_yaw);
        drone->show_pose_info("yaw1", current_pose.pose);
        time_index++;
        if(t == 1.0){
            break;
        }
    }

    //down to 1.2
    const double T_T_S_10 = T_S*1.8;
    drone->get_current_pose(segment_start_pose);
    time_index = 0.0;
    while(node.ok())
    {
        ros::spinOnce();
        rate.sleep();
        // update
        drone->get_current_pose(current_pose);
        //task
        const double t = time_index*T_T_S_10 >= 1.0 ? 1.0 : time_index*T_T_S_10;
        const double x = segment_start_pose.pose.position.x;
        const double y = segment_start_pose.pose.position.y;
        //const double z = segment_start_pose.pose.position.z+(0.6 - segment_start_pose.pose.position.z)*t;//jiang
        const double z = segment_start_pose.pose.position.z*(1-t);
         drone->set_pose(x,y,z, drone->get_roll(segment_start_pose.pose), drone->get_pitch(segment_start_pose.pose), drone->get_yaw(segment_start_pose.pose));
        drone->show_pose_info("down to 1.2 m", current_pose.pose);
        time_index++;
        if(t == 1.0){
            break;
        }
    }
    ros::spin();
    return 0;
}
