#ifndef LASER_H
#define LASER_H 

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>

class LaserData {

public:

    LaserData() :
    range_min(0.0),
    range_max(0.0) {}

    LaserData(const sensor_msgs::LaserScan::ConstPtr& scan) : 
        range_min(scan->range_min),
        range_max(scan->range_max) { 
                for (size_t i = 0; i < scan->ranges.size(); ++i) {
                    // Readings out of range are ignored
                    // We also ignore max range readings
                    if (scan->ranges[i] < range_min 
                            || scan->ranges[i] >= range_max) {
                        continue;
                    }
                
                    auto angle = scan->angle_min + i * scan->angle_increment;
                    auto x = sin(angle) * scan->ranges[i];
                    auto y = cos(angle) * scan->ranges[i];
                    tf::Vector3 range(x,y,0);
                    ranges.push_back(range);
                }
           }

    double range_min;
    double range_max;
    std::vector<tf::Vector3> ranges;
};

#endif /* LASER_H */
