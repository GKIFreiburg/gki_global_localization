#include "global_localization.h"

int main(int argc, char** argv) {
    ROS_INFO("Waiting for map and laserscan messages...");
    ros::init(argc, argv, "m3rsm_global_localization");

    ros::NodeHandle nh;
    double sigma_hit; 
    double z_rand;
    double z_hit;
    double max_range;
    nh.param("laser_sigma_hit", sigma_hit, 0.2);
    nh.param("laser_z_rand", z_rand, 0.05);
    nh.param("laser_z_hit", z_hit, 0.95);
    nh.param("laser_max_range", max_range, 29.0);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
            "localized_pose", 0 );
    GlobalLocalization glob(sigma_hit, z_rand, z_hit, max_range, pose_pub);


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



