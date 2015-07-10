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
   void processMap(const nav_msgs::OccupancyGrid& msg); 
   void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
   tf::Pose globalLocalization();

private:
    LaserData data;
    map_t* map;
    LikelyHoodField* field;
};

#endif /* GLOBAL_LOCALIZATION_H */
