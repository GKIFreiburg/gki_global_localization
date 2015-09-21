#include "global_localization.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher pose_pub;
bool localized = false;
boost::shared_ptr<GlobalLocalization> glob;

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // assumption: one AMCL node awaits initial location
    // stop publishing after first publish
    // resume publishing when AMCL node is restarted
    if (pose_pub.getNumSubscribers() > 0) {
        if (!localized) {
            try {
                tf::Stamped<tf::Pose> pose = glob->globalLocalization(scan);
                geometry_msgs::PoseWithCovarianceStamped msg;
                msg.header.stamp = pose.stamp_;
                msg.header.frame_id = pose.frame_id_;
                tf::poseTFToMsg(pose, msg.pose.pose);
                // covariance matrix
                // [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                //  0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                //  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                //  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                //  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                //  0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
                msg.pose.covariance.elems[0] = 0.25;
                msg.pose.covariance.elems[7] = 0.25;
                msg.pose.covariance.elems[35] = 0.06853891945200942;
                pose_pub.publish(msg);
                ROS_INFO("Publishing initial pose.");
                localized = true;
            }
            catch(...) {
            }
        }
    }
    else {
        ROS_INFO_THROTTLE(5.0, "No listeners, awaiting subscriptions...");
        localized = false;
    }
}

int main(int argc, char** argv) {
    ROS_INFO("Initializing global localization...");
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

    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            "localized_pose", 1, true );
    glob.reset(new GlobalLocalization(sigma_hit, z_rand, z_hit, max_range, pose_pub));


    // Request map
    ros::Subscriber mapSub = nh.subscribe("map", 1,
            &GlobalLocalization::processMap, glob);

    // Request laser scan message
    ros::Subscriber scanSub = nh.subscribe<sensor_msgs::LaserScan>(
            "frontLaser", 10, &processLaserScan);

    ROS_INFO("Global localization ready.");
    ros::spin();
    return 0;
}



