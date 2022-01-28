#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "geometry_msgs/Point.h"
#include "math.h"
#include "miradar.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
// include filters
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
// include SAC
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
// include others
#include <nav_msgs/Odometry.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/surface/mls.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/PointCloud2.h"

class ScanMatcher {
public:
    ScanMatcher(ros::Rate& rate, ros::NodeHandle& nh) : rate(rate), n(nh) {
        odomPub = n.advertise<nav_msgs::Odometry>("/miradar/odom", 10);
        pcSub =
            n.subscribe("/miradar/points", 20, &ScanMatcher::scanMatch, this);
        regPub = n.advertise<sensor_msgs::PointCloud2>(
            "/miradar/registered_cloud", 10);
    }

private:
    pcl::PointCloud<pcl::PointXYZI> last_pc;
    pcl::PointCloud<pcl::PointXYZI> reg_pc;
    bool isFirst = true;
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    double last_x = 0;
    double last_y = 0;
    double last_z = 0;
    double max_x = 0;
    double max_y = 0;
    int trials = 0;

    ros::Publisher odomPub;
    ros::Publisher regPub;
    ros::Time lastTime, currentTime;
    tf::TransformBroadcaster odomBroadcaster;
    ros::Rate rate;
    ros::NodeHandle n;
    ros::Subscriber pcSub;
    void scanMatch(const sensor_msgs::PointCloud2::Ptr& msg) {
        sensor_msgs::PointCloud2::Ptr input_cloud = msg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        if (!isFirst) {
            pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

            icp.setInputSource(last_pc.makeShared());
            icp.setInputTarget(cloud->makeShared());
            icp.align(reg_pc);

            if (icp.hasConverged()) {
                Eigen::Matrix4d icpmat = Eigen::Matrix4d::Identity();
                icpmat = icp.getFinalTransformation().cast<double>();
                double norm = sqrt(icpmat(0, 3) * icpmat(0, 3) +
                                   icpmat(1, 3) * icpmat(1, 3) +
                                   icpmat(2, 3) * icpmat(2, 3));

                if (norm >= 1) {
                    ROS_INFO("Noise detected, skipping scan");
                    return;
                }
                last_pc = reg_pc;

                sum_x += icpmat(0, 3);
                sum_y += icpmat(1, 3);
                sum_z += icpmat(2, 3);
                max_x = (abs(icpmat(0, 3)) > max_x) ? abs(icpmat(0, 3)) : max_x;
                max_y = (abs(icpmat(1, 3)) > max_y) ? abs(icpmat(1, 3)) : max_y;

                Eigen::Matrix3f rot;
                rot << icpmat(0, 0), icpmat(0, 1), icpmat(0, 2), icpmat(1, 0),
                    icpmat(1, 1), icpmat(1, 2), icpmat(2, 0), icpmat(2, 1),
                    icpmat(2, 2);
                Eigen::Quaternionf q(rot);

                currentTime = ros::Time::now();

                geometry_msgs::Quaternion odom_quat;
                odom_quat.w = q.w();
                odom_quat.x = q.x();
                odom_quat.y = q.y();
                odom_quat.z = q.z();

                double dt = (currentTime - lastTime).toSec();

                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = currentTime;
                odom_trans.header.frame_id = "miradar";
                odom_trans.child_frame_id = "miradar_scan";

                odom_trans.transform.translation.x = icpmat(0, 3) + last_x;
                odom_trans.transform.translation.y = icpmat(1, 3) + last_y;
                odom_trans.transform.translation.z = icpmat(2, 3) + last_z;
                odom_trans.transform.rotation = odom_quat;

                last_x += icpmat(0, 3);
                last_y += icpmat(1, 3);
                last_z += icpmat(2, 3);

                Eigen::Vector3f euler =
                    q.toRotationMatrix().eulerAngles(0, 1, 2).cast<float>();

                nav_msgs::Odometry odom;
                odom.header.stamp = currentTime;
                odom.header.frame_id = "miradar";
                odom.child_frame_id = "miradar_scan";
                odom.twist.twist.linear.x = icpmat(0, 3) / dt;
                odom.twist.twist.linear.y = icpmat(1, 3) / dt;
                odom.twist.twist.angular.z = icpmat(2, 3) / dt;

                odomPub.publish(odom);
                ROS_INFO("rotation : yaw = %f, pitch = %f, roll = %f", euler[0],
                         euler[1], euler[2]);
                ROS_INFO("translation : x = %f, y = %f, z = %f", icpmat(0, 3),
                         icpmat(1, 3), icpmat(2, 3));

                sensor_msgs::PointCloud2 pointcloud_msg;
                pcl::toROSMsg(last_pc, pointcloud_msg);
                pointcloud_msg.header.frame_id = "miradar";
                regPub.publish(pointcloud_msg);

                lastTime = currentTime;
                trials++;
                return;

            } else {
                ROS_INFO("Registration Failed.");
            }
        }
        last_pc = *cloud;
        lastTime = currentTime;
        isFirst = false;
        trials++;
    }
};

int main(int argc, char* argv[]) {
    if (ros::isInitialized()) {
        ros::init(argc, argv, "scan_matching");
    } else {
        ros::Time::init();
        ros::init(argc, argv, "scan_matching");
    }

    ros::Rate rate(10);
    ros::NodeHandle nh("~");

    ScanMatcher scanmatcher(rate, nh);
    ros::spin();
}
