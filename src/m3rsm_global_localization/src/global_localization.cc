#include "global_localization.h"


void GlobalLocalization::processMap(const nav_msgs::OccupancyGrid& map_msg) {
	if (map != NULL)
	{
		bool size_changed = false;
	    size_changed |= map->size_x == map_msg.info.width;
	    size_changed |= map->size_y == map_msg.info.height;
	    size_changed |= map->scale == map_msg.info.resolution;
	    size_changed |= map->origin_x == map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
	    size_changed |= map->origin_y == map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
	    if (size_changed)
	    {
	    	free(map->cells);
	    	free(map);
	    	map = NULL;
	    }
	}
	if (map == NULL)
	{
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
	}
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
    if (field != NULL)
    {
    	delete field;
    }
    field = new LikelyHoodField(map, sigma_hit, z_rand, z_hit, laser_max_range);
    field->initializeLikelyHoodFieldMap();
    map_frame_id = map_msg.header.frame_id;
    map_stamp = map_msg.header.stamp;
    
}

void GlobalLocalization::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ROS_INFO("Laser scan received");
    try
    {
    	tf::Stamped<tf::Pose> localizedPose = globalLocalization(scan);
		ROS_INFO("Global pose is: %.3f %.3f %.3f",
				localizedPose.getOrigin().x(),
				localizedPose.getOrigin().y(),
				tf::getYaw(localizedPose.getRotation()));
		geometry_msgs::PoseStamped poseMsg;
		tf::poseStampedTFToMsg(localizedPose, poseMsg);
		pose_pub.publish(poseMsg);
    }
    catch(...)
    {
    }
}

tf::Stamped<tf::Pose> GlobalLocalization::globalLocalization(const sensor_msgs::LaserScan::ConstPtr& scan) {
	if (field == NULL)
	{
		ROS_INFO_THROTTLE(1.0, "Waiting for map data...");
		throw "No map data available!";
	}
    ROS_INFO("Initiating global localization...");
    data = LaserData(scan, laser_max_range);
    ros::Time start = ros::Time::now();
	tf::Stamped<tf::Pose> stampedPose(field->likelyHoodFieldModel(data), scan->header.stamp, map_frame_id);
    double time_elapsed = (ros::Time::now() - start).toSec();
    ROS_INFO("Global localization took %.3f seconds", time_elapsed);
    return stampedPose;
} 
