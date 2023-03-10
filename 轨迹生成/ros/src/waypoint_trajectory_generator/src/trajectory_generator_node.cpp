#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>

// Useful customized headers
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

//  全局变量
    double _vis_traj_width;
    double _Vel, _Acc;
    int    _dev_order, _min_order;
    OsqpEigen::Solver slover;   // 这玩意只要初始化后就需要全生命周期存在，不然就会报错，坑爹

// ros 相关变量
    ros::Subscriber _way_pts_sub;
    ros::Publisher  _wp_traj_vis_pub, _wp_path_vis_pub;

// 规划要用的变量
    int _poly_num1D;
    MatrixXd _polyCoeff;    // 系数矩阵
    VectorXd _polyTime;     //每段时间
    Vector3d _startPos = Vector3d::Zero();  //起始坐标
    Vector3d _startVel = Vector3d::Zero();  //起始速度

// 声明
    void visWayPointTraj( MatrixXd polyCoeff, VectorXd time);
    void visWayPointPath(MatrixXd path);
    Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t );
    VectorXd timeAllocation( MatrixXd Path);
    void trajGeneration(Eigen::MatrixXd path);
    void rcvWaypointsCallBack(const nav_msgs::Path & wp);

// 获取路径点
void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list; //路径点列表
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        // 遇到z坐标小于0则抛弃后面的点
        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }

    // 导航点列表
    MatrixXd waypoints(wp_list.size() + 1, 3);
    waypoints.row(0) = _startPos;
    
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k+1) = wp_list[k];

    //轨迹生成，使用minimum snap trajectory generation方法
    //路径点是路径搜索生成的，这里先用手动指定来设置
    trajGeneration(waypoints);
}

// 轨迹生成算法
void trajGeneration(Eigen::MatrixXd path)
{
    TrajectoryGeneratorWaypoint  trajectoryGeneratorWaypoint;
    
    // 起点及终点的速度和加速度
    MatrixXd vel = MatrixXd::Zero(2, 3); 
    MatrixXd acc = MatrixXd::Zero(2, 3);

    //赋值初始速度
    vel.row(0) = _startVel;

    // 时间分配，简单点就可以每个都分配1拉倒
    _polyTime  = timeAllocation(path);

    // 生成mini-snap的多项式轨迹
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

    //可视化，直线路径
    visWayPointPath(path);

    //可视化规划轨迹
    visWayPointTraj( _polyCoeff, _polyTime);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    //初始化系数？我猜应该是vel，acc这些的最大值，不要超过这些值
    nh.param("planning/vel",   _Vel,   1.0 );
    nh.param("planning/acc",   _Acc,   1.0 );
    nh.param("planning/dev_order", _dev_order,  3 );
    nh.param("planning/min_order", _min_order,  3 );
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);

    //_poly_numID 多项式系数的个数，阶数为个数-1
    _poly_num1D = 2 * _dev_order;

    //初始状态
    _startPos(0)  = 0;
    _startPos(1)  = 0;
    _startPos(2)  = 0;    

    _startVel(0)  = 0;
    _startVel(1)  = 0;
    _startVel(2)  = 0;
    
    //获取手动路径点
    _way_pts_sub     = nh.subscribe( "waypoints", 1, rcvWaypointsCallBack );

    //可视化方面的？
    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();  
        status = ros::ok();  
        rate.sleep();
    }
    return 0;
}

void visWayPointTraj( MatrixXd polyCoeff, VectorXd time)
{        
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "/map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;


    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }

    _wp_traj_vis_pub.publish(_traj_vis);
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "/map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);

      if( i < (path.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = path(i+1, 0);
          p_line.y = path(i+1, 1); 
          p_line.z = path(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}

// 时间分配函数
// 可以简单的分配1为时间
// 一般是以梯形来计算时间，先加速，中匀速，后减速，面积为路径长度。
VectorXd timeAllocation( MatrixXd Path)
{ 
    VectorXd time(Path.rows() - 1);
    
    // 加速段时间
    double t_scope = _Vel / _Acc;
    // 加速段距离(同时考虑加速和减速段）
    double distance_acc = 1.0 / 2.0 * _Acc * t_scope * t_scope * 2.0;

    for (int k = 0; k < Path.rows() - 1; ++k) {
        //计算路径直线距离
        Vector3d delta = Path.row(k) - Path.row(k + 1);
        double d = std::sqrt(delta.dot(delta));

        if (d <= distance_acc) {
            time(k) = std::sqrt(d / _Acc);
        }
        else {
            time(k) = 2 * t_scope + (d - distance_acc) / _Vel;
        }
    }
    return time;
}