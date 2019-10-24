/*
 * AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
 * PRACTICE 8 - LOCALIZATION BY PARTICLE FILTERS - PART I
 *
 * Instructions:
 * Complete the code to perform the following operations:
 * - Initialize N particles with positions uniformly distributed within the map.
 * - For each particle, simulate a laser scan given a map and the particle position.
 * - For each particle, calculate a weight in [0,1) indicating the similarity between
 *   its simulated scan and a real scan. 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "visualization_msgs/Marker.h"
#include "random_numbers/random_numbers.h"
#include "occupancy_grid_utils/ray_tracer.h"
#include "tf/transform_listener.h"
#include "math.h"

#define NOMBRE "Lagunas_Oscar"

#define NUMBER_OF_PARTICLES 1000
#define LASER_DOWNSAMPLING 10
#define SENSOR_NOISE 2

std::vector<geometry_msgs::Pose> get_initial_distribution(int N, float min_x, float min_y, float max_x, float max_y)
{
    /*
     * TODO:
     * Generate a set of N particles (each particle represented by a Pose structure)
     * with positions uniformly distributed within the given bounding box.
     */
    std::vector<geometry_msgs::Pose> particles;
    random_numbers::RandomNumberGenerator rng;
    particles.resize(N);
    for(int i =0;i<N;i++){
        particles[i].position.x = rng.uniformReal(min_x,max_x);
        particles[i].position.y = rng.uniformReal(min_y,max_y);
        particles[i].orientation.x =0;
        particles[i].orientation.y =0;
        double theta = rng.uniformReal(-M_PI,M_PI);
        particles[i].orientation.z =sin(theta/2);
        particles[i].orientation.w =cos(theta/2);
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


     for(int i=0;i<particles.size();i++){
         simulated_scans[i] = *occupancy_grid_utils::simulateRangeScan(map,particles[i],scan_info,false);
     }

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

    weights_sum =0;
    max_weight =0;
    for(size_t i =0;i<simulated_scans.size();i++){
        float d= 0;
        for(size_t j = 0;j<simulated_scans[i].ranges.size();j++){
            d +=fabs(simulated_scans[i].ranges[j] - real_scan.ranges[j*LASER_DOWNSAMPLING]);
        }
        d/= simulated_scans[i].ranges.size();
        weights[i] = exp(-d*d/SENSOR_NOISE);
        weights_sum += weights[i];
        max_weight = std::max(max_weight, weights[i]);
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
    
    std::vector<geometry_msgs::Pose> particles = get_initial_distribution(N, min_x, min_y, max_x, max_y);

    while(ros::ok())
    {
        simulate_particle_scans(particles, map, scan_info, simulated_scans);
        calculate_particle_weights(simulated_scans, real_scan, weights, max_weight, weights_sum);

        particles_marker(particles, weights, max_weight, mrk);
        pub_markers.publish(mrk);
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}
