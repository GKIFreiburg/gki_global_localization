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
    constexpr double max_occ_dist = 2.0;
    map_update_cspace(map, max_occ_dist);
    std::cout << "Map received." << std::endl;
}

void GlobalLocalization::processLaserScan(
        const sensor_msgs::LaserScan::ConstPtr& scan) {
    LaserData d(scan);
    data = d;
    std::cout << "Laser Scan Received." << std::endl;
    globalLocalization();
}

tf::Pose GlobalLocalization::globalLocalization() {
    double sigma_hit = 0.2;
    double z_rand = 0.05;
    double z_hit = 0.95;
    double laser_max_range = 29;
    field = new LikelyHoodField(map, sigma_hit, z_rand, 
            z_hit, laser_max_range);
    field->initializeLikelyHoodFieldMap();

    std::cout << "Initiating global localization..." << std::endl;
    ros::Time start = ros::Time::now();
    tf::Pose pose = field->likelyHoodFieldModel(data);
    double time_elapsed = (ros::Time::now() - start).toSec();
    std::cout << "done after " << time_elapsed << " sec." <<  std::endl;
    return pose;
} 
