#include "global_localization.h"

int main(int argc, char** argv) {
    std::cout << "Waiting for map and laserscan messages..."  << std::endl;
    ros::init(argc, argv, "m3rsm_global_localization");
    ros::NodeHandle nh;
    GlobalLocalization glob;
    // Request map
    ros::Subscriber mapSub;
    mapSub = nh.subscribe("map", 1, &GlobalLocalization::processMap, &glob);

   // Request laser scan message
    ros::Subscriber scanSub;
    scanSub = nh.subscribe<sensor_msgs::LaserScan>(
            "frontLaser",10,&GlobalLocalization::processLaserScan, &glob);
    ros::spin();
    return 0;
}



