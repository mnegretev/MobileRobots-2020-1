/*
 * AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
 * PRACTICE 8 - LOCALIZATION BY PARTICLE FILTERS - PART II
 *
 * Instructions:
 * Complete the indicated functions to implement localization by particle filters. 
 */

#include <iostream>   
#include <string>  
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/String.h"
#include "random_numbers/random_numbers.h"
#include "occupancy_grid_utils/ray_tracer.h"
#include "tf/transform_listener.h"

#define NOMBRE "APELLIDO_PATERNO_APELLIDO_MATERNO"

#define LASER_DOWNSAMPLING 10
#define SENSOR_NOISE       0.5
#define RESAMPLING_NOISE_POSITION 0.2
#define RESAMPLING_NOISE_ANGLE    0.1
#define MOVEMENT_NOISE_POSITION   0.1
#define MOVEMENT_NOISE_ANGLE      0.05
#define THRESHOLD_DISTANCE  0.2
#define THRESHOLD_ANGLE     0.2


std::vector<geometry_msgs::Pose> get_initial_distribution(int N, float min_x, float min_y, float max_x, float max_y)
{
    std::vector<geometry_msgs::Pose> particles;
    particles.resize(N);
    random_numbers::RandomNumberGenerator rnd;

    /*
     * TODO:
     * Generate a set of N particles (each particle represented by a Pose structure)
     * with positions uniformly distributed within the given bounding box.
     */
    for(size_t i=0; i < particles.size(); i++)
    {
        particles[i].position.x  = rnd.uniformReal(min_x, max_x);
        particles[i].position.y  = rnd.uniformReal(min_y, max_y);
        float a = rnd.uniformReal(-M_PI, M_PI);
        particles[i].orientation.w = cos(a/2);
        particles[i].orientation.z = sin(a/2);
    }
    return particles;
}

void simulate_particle_scans(std::vector<geometry_msgs::Pose>& particles, nav_msgs::OccupancyGrid& map,
                             sensor_msgs::LaserScan& scan_info, std::vector<sensor_msgs::LaserScan>& simulated_scans)
{
    /*
     * TODO:
     * Simulate a laser scan for each particle given a set of particles, a map and a scan info.
     * Store the simulated scans in 'simulated_scans'.
     * Check online documentation
     * http://docs.ros.org/groovy/api/occupancy_grid_utils/html/namespaceoccupancy__grid__utils.html
     */
    for(size_t i=0; i < particles.size(); i++)
        simulated_scans[i] = *occupancy_grid_utils::simulateRangeScan(map, particles[i], scan_info);
}

void calculate_particle_weights(std::vector<sensor_msgs::LaserScan>& simulated_scans, sensor_msgs::LaserScan& real_scan,
                                std::vector<float>& weights, float& max_weight, float& weights_sum)
{
    /*
     * TODO:
     * For each particle, calculate a weight in [0,1) indicating the similarity between
     * its simulated scan and a real scan.
     * Determine also the sum of all weights and maximum of all weights.
     * Store results in the corresponding variables
     */
    weights_sum = 0;
    max_weight = 0;
    for(size_t i=0; i < simulated_scans.size(); i++)
    {
        weights[i] = 0;
        for(size_t j=0; j < simulated_scans[i].ranges.size(); j++)
            weights[i] += fabs(simulated_scans[i].ranges[j] - real_scan.ranges[j*LASER_DOWNSAMPLING]);
        weights[i] = exp(-weights[i]/simulated_scans[i].ranges.size()/SENSOR_NOISE);
        weights_sum += weights[i];
        max_weight   = std::max(max_weight, weights[i]);
    }
}

int random_choice(std::vector<float>& weights, float weights_sum)
{
    random_numbers::RandomNumberGenerator rnd;
    
    /*
     * TODO:
     * Write an algorithm to choice an integer in the range [0, N), with N, the size of 'weights'.
     * Probability of picking an integer 'i' should be proportional to the corresponding weights[i] value.
     * For example, if weights = [2.0 4.0 6.0 2.0 6.0], the probability distribution for
     * the integers [0 1 2 3 4] would be p = [0.1 0.2 0.3 0.1 0.3].
     * Return the chosen integer. 
     */
    
    float beta = rnd.uniformReal(0, weights_sum);
    for(int i=0; i < weights.size(); i++)
        if(beta < weights[i])
            return i;
        else
            beta -= weights[i];
    return -1;
}

std::vector<geometry_msgs::Pose> resample_particles(std::vector<geometry_msgs::Pose>& particles,
                                                    std::vector<float>& weights, float weights_sum)
{
    random_numbers::RandomNumberGenerator rnd;
    std::vector<geometry_msgs::Pose> resampled_particles;
    resampled_particles.resize(particles.size());
    /*
     * TODO:
     * Sample with replacement, N particles from the set 'particles'.
     * The probability of the i-th particle of being resampled is proportional
     * to the i-th weight. Use the random_choice function to pick a particle with the correct probability.
     * Add gaussian noise, to each sampled particle. Use RESAMPLING_NOISE_POSITION and
     * RESAMPLING_NOISE_ANGLE as variances.
     * Return the set of new particles. 
     */

    for(size_t i=0; i<particles.size(); i++)
    {
        int idx = random_choice(weights, weights_sum);
        resampled_particles[i].position.x = particles[idx].position.x + rnd.gaussian(0, RESAMPLING_NOISE_POSITION);
        resampled_particles[i].position.y = particles[idx].position.y + rnd.gaussian(0, RESAMPLING_NOISE_POSITION);
        float angle = atan2(particles[idx].orientation.z, particles[idx].orientation.w)*2;
        angle += rnd.gaussian(0, RESAMPLING_NOISE_ANGLE);
        resampled_particles[i].orientation.w = cos(angle/2);
        resampled_particles[i].orientation.z = sin(angle/2);
    }
    return resampled_particles;
}

void move_particles(std::vector<geometry_msgs::Pose>& particles, float delta_x, float delta_y, float delta_t)
{
    random_numbers::RandomNumberGenerator rnd;
    /*
     * TODO:
     * Move each particle a displacement given by delta_x, delta_y and delta_t.
     * Displacement is given w.r.t. robot's frame, i.e., to calculate the new position of
     * each particle you need to rotate delta_x and delta_y, on Z axis, an angle theta_i, where theta_i
     * is the orientation of the i-th particle.
     * Add gaussian noise to each new position. Use MOVEMENT_NOISE_POSITION and MOVEMENT_NOISE_ANGLE
     * as covariances. 
     */
    for(size_t i=0; i < particles.size(); i++)
    {
        float a = atan2(particles[i].orientation.z, particles[i].orientation.w)*2;
        particles[i].position.x += delta_x*cos(a) - delta_y*sin(a) + rnd.gaussian(0, MOVEMENT_NOISE_POSITION);
        particles[i].position.y += delta_x*sin(a) + delta_y*cos(a) + rnd.gaussian(0, MOVEMENT_NOISE_POSITION);
        a += delta_t + rnd.gaussian(0, MOVEMENT_NOISE_ANGLE);
        particles[i].orientation.w = cos(a/2);
        particles[i].orientation.z = sin(a/2);
    }
}

void particles_marker(std::vector<geometry_msgs::Pose>& particles, std::vector<float>& weights, float max_weight,
                      visualization_msgs::Marker& mrk)
{
    for(size_t i=0; i < particles.size(); i++)
    {
        mrk.points[i]   = particles[i].position;
        mrk.colors[i].r = 1.0 - weights[i]/max_weight;
        mrk.colors[i].g = weights[i]/max_weight;
        mrk.colors[i].b = 1.0 - weights[i]/max_weight;
        mrk.colors[i].a = 1.0;
    }
}

void get_robot_odometry(tf::TransformListener* listener, float& robot_x, float& robot_y, float& robot_a)
{
    tf::StampedTransform t;
    tf::Quaternion q;
    listener->lookupTransform("odom", "base_link", ros::Time(0), t);
    robot_x = t.getOrigin().x();
    robot_y = t.getOrigin().y();
    q = t.getRotation();
    robot_a = atan2(q.z(), q.w())*2;
}

sensor_msgs::LaserScan real_scan;
void callback_laser_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    real_scan = *msg;
}

int main(int argc, char** argv)
{
    std::cout << "PRACTICE 08 - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practice08");
    ros::NodeHandle n("~");
    ros::Rate loop(10);
    ros::Subscriber sub_scan = n.subscribe("/hardware/scan", 1, callback_laser_scan);
    ros::Publisher  pub_markers  = n.advertise<visualization_msgs::Marker>("/hri/visualization_marker", 1);
    ros::Publisher position = n.advertise<std_msgs::String>("/navigation/localization", 1);
    std_msgs::String cord;

    tf::TransformListener listener;
    listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
    nav_msgs::GetMap srv_get_map;
    ros::service::call("/navigation/localization/static_map", srv_get_map);
    nav_msgs::OccupancyGrid map = srv_get_map.response.map;     
    real_scan = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hardware/scan");
    sensor_msgs::LaserScan scan_info = real_scan;
    scan_info.angle_increment *= LASER_DOWNSAMPLING;

    int N = 1000;
    float min_x = -1;
    float max_x = 10;
    float min_y = -1;
    float max_y = 8;

    visualization_msgs::Marker mrk;
    mrk.header.frame_id = "map";
    mrk.ns = "loc_particles";
    mrk.id = 0;
    mrk.type = visualization_msgs::Marker::POINTS;
    mrk.action = visualization_msgs::Marker::ADD;
    mrk.scale.x = 0.07;
    mrk.scale.y = 0.07;
    mrk.points.resize(N);
    mrk.colors.resize(N);
    
    std::vector<sensor_msgs::LaserScan> simulated_scans;
    std::vector<float> weights;
    float max_weight;
    float weights_sum;
    simulated_scans.resize(N);
    weights.resize(N);

    float robot_x = 0;
    float robot_y = 0;
    float robot_a = 0;
    get_robot_odometry(&listener, robot_x, robot_y, robot_a);
    float last_robot_x = robot_x;
    float last_robot_y = robot_y;
    float last_robot_a = robot_a;
    float delta_x;
    float delta_y;
    float delta_a;
    
    std::vector<geometry_msgs::Pose> particles = get_initial_distribution(N, min_x, min_y, max_x, max_y);
    simulate_particle_scans(particles, map, scan_info, simulated_scans);
    calculate_particle_weights(simulated_scans, real_scan, weights, max_weight, weights_sum);

    while(ros::ok())
    {
        get_robot_odometry(&listener, robot_x, robot_y, robot_a);
        delta_x = robot_x - last_robot_x;
        delta_y = robot_y - last_robot_y;
        delta_a = robot_a - last_robot_a;
        if((fabs(robot_x-last_robot_x) + fabs(robot_y-last_robot_y)) > THRESHOLD_DISTANCE ||
           fabs(robot_a - last_robot_a) > THRESHOLD_ANGLE)
        {
            std::cout << "Updating robot position..." << std::endl;
            /*
             * TODO:
             * - Calculate the displacement w.r.t. robot's frame, i.e. rotate delta_x and delta_y
             *   an angle of -robot_a on Z-axis.
             * - Call function 'move_particles' with the appropiate arguments.
             * - Simulate laser scans for each particle. Use the corresponding function.
             * - Calculate weights for each particle. Use the correspondign function.
             * - Perform a resample according to the weights calculated in the previous step.
             *   Use the corresponding function.
             */
            float dx =  delta_x*cos(robot_a) + delta_y*sin(robot_a);
            float dy = -delta_x*sin(robot_a) + delta_y*cos(robot_a);
            move_particles(particles, dx, dy, delta_a);
            simulate_particle_scans(particles, map, scan_info, simulated_scans);
            calculate_particle_weights(simulated_scans, real_scan, weights, max_weight, weights_sum);
            particles = resample_particles(particles, weights, weights_sum);
            last_robot_x = robot_x;
            last_robot_y = robot_y;
            last_robot_a = robot_a;
            
            cord.data = std::to_string(last_robot_x) + "," + std::to_string(last_robot_y) + "," + std::to_string(last_robot_a);
            position.publish(cord);
            //std::cout << cord << std::endl;
        }
        particles_marker(particles, weights, max_weight, mrk);
        pub_markers.publish(mrk);
        
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}
