#include "global_localization.h"
#include "MapDataMissingException.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher pose_pub;
bool localized = false;
boost::shared_ptr<GlobalLocalization> glob;
std::string robot_frame;
boost::shared_ptr<tf::TransformListener> listener;

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	ROS_DEBUG_STREAM_THROTTLE(1.0, "Laser data received.");
	std::string error_msg;
	if (!listener->canTransform(robot_frame, scan->header.frame_id, scan->header.stamp, &error_msg))
	{
		ROS_ERROR_STREAM_THROTTLE(1.0, "can not transform: "<<robot_frame<<" to "<<scan->header.frame_id);
		return;
	}
	// assumption: one AMCL node awaits initial location
	// stop publishing after first publish
	// resume publishing when AMCL node is restarted
	if (pose_pub.getNumSubscribers() > 0)
	{
		if (!localized)
		{
			try
			{
				tf::Stamped<tf::Pose> laser_pose = glob->localize(scan);
				tf::Stamped<tf::Pose> identity(tf::Pose(), scan->header.stamp, scan->header.frame_id);
				identity.setIdentity();
				tf::Stamped<tf::Pose> robot_to_laser;
				listener->transformPose(robot_frame, identity, robot_to_laser);
				geometry_msgs::PoseWithCovarianceStamped msg;
				msg.header.stamp = laser_pose.stamp_;
				msg.header.frame_id = laser_pose.frame_id_;
				tf::poseTFToMsg(laser_pose * robot_to_laser.inverse(), msg.pose.pose);
				// default covariance matrix sent from rviz to amcl.
				// [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
				//  0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
				//  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				//  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				//  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				//  0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
				// however: we are more certain of our localization
				msg.pose.covariance.elems[0] = 0.05;
				msg.pose.covariance.elems[7] = 0.05;
				msg.pose.covariance.elems[35] = 0.0025;
				pose_pub.publish(msg);
				ROS_INFO("Publishing initial pose.");
				localized = true;

			} catch (tf::InvalidArgument& ex)
			{
				ROS_ERROR_STREAM("tf::InvalidArgument "<<ex.what());
			} catch (MapDataMissingException& ex)
			{
				ROS_ERROR_STREAM("MapDataMissingException "<<ex.what());
			} catch (...)
			{
				ROS_ERROR_STREAM("Unknown error");
			}
		}
	}
	else
	{
		ROS_INFO_THROTTLE(5.0, "No listeners, awaiting subscriptions...");
		localized = false;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gki_global_localization");
	ROS_INFO("Initializing...");

	double sigma_hit;
	double z_rand;
	double z_hit;
	double max_range;
	ros::NodeHandle pnh("~");
	pnh.param("laser_sigma_hit", sigma_hit, 0.2);
	pnh.param("laser_z_rand", z_rand, 0.05);
	pnh.param("laser_z_hit", z_hit, 0.95);
	pnh.param("laser_max_range", max_range, 29.0); // use laser msg to determine max range
	robot_frame = "base_fake";
	if (! pnh.getParam("base_frame_id", robot_frame)) {
		ROS_FATAL_STREAM("missing parameter: base_frame_id");
		return -1;
	}

	ros::NodeHandle nh;
	pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("localized_pose", 1, true);
	glob.reset(new GlobalLocalization(sigma_hit, z_rand, z_hit, max_range, pose_pub));
	listener.reset(new tf::TransformListener());

	// Request map
	ros::Subscriber mapSub = nh.subscribe("map", 1, &GlobalLocalization::processMap, glob);

	// Request laser scan message
	ros::Subscriber scanSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, &processLaserScan);

	ROS_INFO("Waiting for map and laser data...");
	ros::spin();
	return 0;
}

