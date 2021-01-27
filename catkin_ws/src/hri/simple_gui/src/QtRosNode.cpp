#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    publishing_cmd_vel = false;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(30);
    pubCmdVel     =n->advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 10);
    pubFollowPath =n->advertise<nav_msgs::Path>("/navigation/simple_move/follow_path", 10);
    pubPfGoalPoint=n->advertise<geometry_msgs::PoseStamped>("/navigation/obs_avoidance/pot_fields_goal", 10);
    cltBFS        =n->serviceClient<navig_msgs::CalculatePath>("/navigation/path_planning/breadth_first_search");
    cltDFS        =n->serviceClient<navig_msgs::CalculatePath>("/navigation/path_planning/depth_first_search");
    cltDijkstra   =n->serviceClient<navig_msgs::CalculatePath>("/navigation/path_planning/dijkstra_search");
    cltAStar      =n->serviceClient<navig_msgs::CalculatePath>("/navigation/path_planning/a_star_search");
    cltSmoothPath =n->serviceClient<navig_msgs::SmoothPath   >("/navigation/path_planning/smooth_path");
    cltGetMap     =n->serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");

    int pub_zero_counter = 5;
    while(ros::ok() && !this->gui_closed)
    {
        if(publishing_cmd_vel)
        {
            pubCmdVel.publish(cmd_vel);
            pub_zero_counter = 5;
        }
        else if(--pub_zero_counter > 0)
        {
            if(pub_zero_counter <= 0)
                pub_zero_counter = 0;
            pubCmdVel.publish(cmd_vel);
        }
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::publish_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    pubCmdVel.publish(cmd_vel);
}

void QtRosNode::start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    publishing_cmd_vel = true;
}

void QtRosNode::stop_publishing_cmd_vel()
{
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    publishing_cmd_vel = false;
}

bool QtRosNode::call_static_map(nav_msgs::OccupancyGrid& map)
{
    nav_msgs::GetMap  srv;
    if(!cltGetMap.call(srv))
    {
        std::cout << "QtRosNode.-> Cannot get static map." << std::endl;
        return false;
    }
    map = srv.response.map;
    return true;
}

bool QtRosNode::call_breadth_first_search(float start_x, float start_y, float goal_x, float goal_y,
                                          nav_msgs::OccupancyGrid& map, nav_msgs::Path& path)
{
    navig_msgs::CalculatePath srv;
    srv.request.map = map;
    srv.request.start.pose.position.x = start_x;
    srv.request.start.pose.position.y = start_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    if(!cltBFS.call(srv))
    {
        std::cout << "QtRosNode.->Cannot calculate path by BFS" << std::endl;
        return false;
    }
    path = srv.response.path;
    return true;
}

bool QtRosNode::call_depth_first_search(float start_x, float start_y, float goal_x, float goal_y,
                                        nav_msgs::OccupancyGrid& map, nav_msgs::Path& path)
{
    navig_msgs::CalculatePath srv;
    srv.request.map = map;
    srv.request.start.pose.position.x = start_x;
    srv.request.start.pose.position.y = start_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    if(!cltDFS.call(srv))
    {
        std::cout << "QtRosNode.->Cannot calculate path by DFS" << std::endl;
        return false;
    }
    path = srv.response.path;
    return true;
}

bool QtRosNode::call_dijkstra_search(float start_x, float start_y, float goal_x, float goal_y,
                                     nav_msgs::OccupancyGrid& map, nav_msgs::Path& path)
{
    navig_msgs::CalculatePath srv;
    srv.request.map = map;
    srv.request.start.pose.position.x = start_x;
    srv.request.start.pose.position.y = start_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    if(!cltDijkstra.call(srv))
    {
        std::cout << "QtRosNode.->Cannot calculate path by Dijkstra" << std::endl;
        return false;
    }
    path = srv.response.path;
    return true;
}

bool QtRosNode::call_a_star_search(float start_x, float start_y, float goal_x, float goal_y,
                                   nav_msgs::OccupancyGrid& map, nav_msgs::Path& path)
{
    navig_msgs::CalculatePath srv;
    srv.request.map = map;
    srv.request.start.pose.position.x = start_x;
    srv.request.start.pose.position.y = start_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    if(!cltAStar.call(srv))
    {
        std::cout << "QtRosNode.->Cannot calculate path by A-star" << std::endl;
        return false;
    }
    path = srv.response.path;
    return true;
}

bool QtRosNode::call_smooth_path(nav_msgs::Path& path, nav_msgs::Path& smooth_path)
{
    navig_msgs::SmoothPath srv;
    srv.request.path = path;
    if(!cltSmoothPath.call(srv))
    {
        smooth_path = path;
        std::cout << "QtRosNode.->Cannot smooth path" << std::endl;
        return false;
    }
    smooth_path = srv.response.smooth_path;
    return true;
}

void QtRosNode::publish_goal_path(nav_msgs::Path path)
{
    pubFollowPath.publish(path);
}

void QtRosNode::publish_pf_goal_point(float goal_x, float goal_y)
{
    geometry_msgs::PoseStamped p;
    p.pose.position.x = goal_x;
    p.pose.position.y = goal_y;
    p.pose.position.z = 0;
    pubPfGoalPoint.publish(p);
}

void QtRosNode::set_param_control_type(std::string control_type)
{
    n->setParam("/navigation/control_type/", control_type);
}

void QtRosNode::set_param_inflation_radius(float inflation_radius)
{
    n->setParam("/navigation/path_planning/inflation_radius", inflation_radius);
}

void QtRosNode::set_param_nearness_radius(float nearness_radius)
{
    n->setParam("/navigation/path_planning/nearness_radius",  nearness_radius);
}

void QtRosNode::set_param_smoothing_alpha(float smoothing_alpha)
{
    n->setParam("/navigation/path_planning/smoothing_alpha",  smoothing_alpha);
}
  
void QtRosNode::set_param_smoothing_beta(float  smoothing_beta)
{
    n->setParam("/navigation/path_planning/smoothing_beta" ,  smoothing_beta);
}

void QtRosNode::get_robot_pose(float& robot_x, float& robot_y, float& robot_a)
{
    tf::StampedTransform t;
    tf::Quaternion q;
    tf_listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
    tf_listener.lookupTransform("map", "base_link", ros::Time(0), t);
    robot_x = t.getOrigin().x();
    robot_y = t.getOrigin().y();
    q = t.getRotation();
    robot_a = atan2(q.z(), q.w())*2;
}
