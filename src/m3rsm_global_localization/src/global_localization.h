#ifndef GLOBAL_LOCALIZATION_H
#define GLOBAL_LOCALIZATION_H

#include "likelihood_field.h"
#include "laser.h"
#include "map.h"
#include "nav_msgs/GetMap.h"
#include <sensor_msgs/LaserScan.h>  

// Performs global localization with the M3RSM method and returns the localized 
// pose
//
class GlobalLocalization {
public:
    
    GlobalLocalization(double& sigma_hit_, double& z_rand_, 
            double& z_hit_, double& max_range, ros::Publisher& ppub) :
        sigma_hit(sigma_hit_),
        z_rand(z_rand_),
        z_hit(z_hit_),
        laser_max_range(max_range),
        pose_pub(ppub) {}


   void processMap(const nav_msgs::OccupancyGrid& msg); 
   void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
   tf::Pose globalLocalization();

   LikelyHoodField* field;

private:
    map_t* map;
    std::string map_frame_id;
    ros::Time map_stamp;

    LaserData data;

    double sigma_hit;
    double z_rand;
    double z_hit;
    double laser_max_range;


    ros::Publisher pose_pub;
};

#endif /* GLOBAL_LOCALIZATION_H */
