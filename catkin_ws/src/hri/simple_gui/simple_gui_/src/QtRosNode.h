#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include "navig_msgs/CalculatePath.h"
#include "navig_msgs/SmoothPath.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher        pubCmdVel;
    ros::Publisher        pubFollowPath;
    ros::ServiceClient    cltBFS;
    ros::ServiceClient    cltDFS;
    ros::ServiceClient    cltDijkstra;
    ros::ServiceClient    cltAStar;
    ros::ServiceClient    cltGetMap;
    ros::ServiceClient    cltSmoothPath;
    tf::TransformListener tf_listener;
    
    geometry_msgs::Twist cmd_vel;
    bool publishing_cmd_vel;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void stop_publishing_cmd_vel();

    bool call_static_map          (nav_msgs::OccupancyGrid& map);
    bool call_breadth_first_search(float start_x, float start_y, float goal_x, float goal_y, nav_msgs::OccupancyGrid& map, nav_msgs::Path& path);
    bool call_depth_first_search  (float start_x, float start_y, float goal_x, float goal_y, nav_msgs::OccupancyGrid& map, nav_msgs::Path& path);
    bool call_dijkstra_search     (float start_x, float start_y, float goal_x, float goal_y, nav_msgs::OccupancyGrid& map, nav_msgs::Path& path);
    bool call_a_star_search       (float start_x, float start_y, float goal_x, float goal_y, nav_msgs::OccupancyGrid& map, nav_msgs::Path& path);
    bool call_smooth_path         (nav_msgs::Path& path, nav_msgs::Path& smooth_path);

    void publish_goal_path(nav_msgs::Path path);

    void set_param_control_type(std::string control_type);

    void set_param_inflation_radius(float inflation_radius);
    void set_param_nearness_radius(float nearness_radius);
    void set_param_smoothing_alpha(float smoothing_alpha);
    void set_param_smoothing_beta(float  smoothing_beta);

    void get_robot_pose(float& robot_x, float& robot_y, float& robot_a);
    
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
