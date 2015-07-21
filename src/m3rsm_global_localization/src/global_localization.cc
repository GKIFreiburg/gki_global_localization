#include "global_localization.h"


void GlobalLocalization::processMap(const nav_msgs::OccupancyGrid& map_msg) {
    map = (map_t*) malloc(sizeof(map_t));

    // Assume we start at (0, 0)
    map->origin_x = 0;
    map->origin_y = 0;

    // Make the size odd
    map->size_x = 0;
    map->size_y = 0;
    map->scale = 0;

    // Allocate storage for main map
    map->cells = (map_cell_t*) NULL;

    ROS_ASSERT(map);

    map->size_x = map_msg.info.width;
    map->size_y = map_msg.info.height;
    map->scale = map_msg.info.resolution;
    map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
    map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

    // Convert to amcl map format
    map->cells = (map_cell_t*)malloc(
            (sizeof(map_cell_t)) * map->size_x * map->size_y);
    ROS_ASSERT(map->cells);
    for (int i = 0; i < map->size_x * map->size_y; ++i) {
        if (map_msg.data[i] == 0) {
            map->cells[i].occ_state = -1; // free
        } else if (map_msg.data[i] == 100) {
            map->cells[i].occ_state = +1; // occ
        } else {
            map->cells[i].occ_state = 0; // unknown
        }
    }
    double max_occ_dist = 2.0;
    map_update_cspace(map, max_occ_dist);

    ROS_INFO("Map received. Creating coarser likelyhood maps.");
    field = new LikelyHoodField(map, sigma_hit, z_rand, 
            z_hit, laser_max_range);
    field->initializeLikelyHoodFieldMap();
    map_frame_id = map_msg.header.frame_id;
    map_stamp = map_msg.header.stamp;
    
}

void GlobalLocalization::processLaserScan(
        const sensor_msgs::LaserScan::ConstPtr& scan) {
    LaserData d(scan, laser_max_range);
    data = d;
    ROS_INFO("Laser scan received");
    tf::Pose localizedPose = globalLocalization();
    ROS_INFO("Global pose is: %.3f %.3f %.3f",
            localizedPose.getOrigin().x(),
            localizedPose.getOrigin().y(),
            tf::getYaw(localizedPose.getRotation()));
    tf::Stamped<tf::Pose> stampedPose(
            localizedPose, map_stamp, map_frame_id);
    geometry_msgs::PoseStamped poseMsg;
    tf::poseStampedTFToMsg(stampedPose, poseMsg);
    pose_pub.publish(poseMsg);
}

tf::Pose GlobalLocalization::globalLocalization() {
    ROS_INFO("Initiating global localization...");
    ros::Time start = ros::Time::now();
    tf::Pose pose = field->likelyHoodFieldModel(data);
    double time_elapsed = (ros::Time::now() - start).toSec();
    ROS_INFO("Global localization took %.3f seconds", time_elapsed);
    return pose;
} 
