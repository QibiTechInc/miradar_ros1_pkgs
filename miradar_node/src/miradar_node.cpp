#include "cv_bridge/cv_bridge.h"
#include "dynamic_reconfigure/server.h"
#include "geometry_msgs/Point.h"
#include "image_transport/image_transport.h"
#include "math.h"
#include "miradar.h"
#include "miradar_node/MiRadarConfig.h"
#include "miradar_node/PPI.h"
#include "miradar_node/PPIData.h"
#include "opencv2/opencv.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#define DEG2RAG(deg) (((deg) / 360) * 2 * M_PI)

class MiRadarROS {
public:
    void setParam(miradar_node::MiRadarConfig& config, uint32_t level) {
        ROS_INFO("Applying Parameter Change.");

        sensorMode = config.sensor_mode;
        miradarParam.maxDistance =
            static_cast<int>(config.max_distance * 1000.0);
        miradarParam.minDistance =
            static_cast<int>(config.min_distance * 1000.0);
        miradarParam.maxAngle = config.max_angle;
        miradarParam.maxDb = config.max_dB;
        miradarParam.minDb = config.min_dB;

        miradarParam.alarmDistance =
            static_cast<int>(config.alarm_distance * 1000.0);
        miradarParam.nDistance = config.distance_div;
        miradarParam.nAngle = config.angle_div;
        miradarParam.txPower = config.tx_power;
        miradarParam.hpfGain = config.hpf_gain;
        miradarParam.pgaGain = config.pga_gain;
        miradarParam.duration = config.duration;
        laserscanMode = config.scan_mode;
        isConfigUpdate = true;
    }

    explicit MiRadarROS(ros::Rate& rate, ros::NodeHandle& pa)
        : sensorMode(1),
          loopRate(rate),
          param(pa),
          isConfigUpdate(false),
          laserscanMode(1) {
        param.getParam("devicename", deviceFile);
        pub = n.advertise<miradar_node::PPIData>("/miradar/ppidata", 20);
        mapPub = n.advertise<sensor_msgs::Image>("/miradar/image_raw", 20);
        laserPub = n.advertise<sensor_msgs::LaserScan>("/miradar/scan", 20);
        pcPub = n.advertise<sensor_msgs::PointCloud2>("/miradar/points", 20);

        deviceFile =
            (deviceFile.find("/dev/tty") == -1) ? "/dev/ttyUSB0" : deviceFile;

        initConnection();
        f = boost::bind(&MiRadarROS::setParam, this, _1, _2);
        server.setCallback(f);
    }

    void initConnection() {
        int fd;
        fd = serial.CommInit(deviceFile);
        if (fd < 0) {
            ROS_INFO("device open failed.");
            exit(-1);
        }
        miradar.setSerial(serial);
        ROS_INFO("Connected to %s", (char*)deviceFile.c_str());
        changeSensorState();
    }

    void changeSensorState() {
        miradar.setSensorState(sensorMode);
        miradar.sendSensorMode();
    }

    void publishMap() {
        if (miradar.nAngle * miradar.nDistance == miradar.map.size()) {
            sensor_msgs::Image image;
            image.height = miradar.nAngle;
            image.width = miradar.nDistance;
            image.step = image.width;
            image.header.frame_id = "miradar";
            image.encoding = "mono8";
            std::copy(miradar.map.begin(), miradar.map.end(),
                      std::back_inserter(image.data));
            mapPub.publish(image);
        } else {
            ROS_INFO("map is corrupt.");
        }
    }

    void publishLaserScan() {
        if (miradar.nAngle * miradar.nDistance == miradar.map.size()) {
            sensor_msgs::LaserScan ls;
            ls.header.frame_id = "miradar_scan";
            ls.angle_min = DEG2RAG(-static_cast<double>(miradarParam.maxAngle));
            ls.angle_max = DEG2RAG(static_cast<double>(miradarParam.maxAngle));
            ls.range_min =
                static_cast<double>(miradarParam.minDistance) / 1000.0;
            ls.range_max =
                static_cast<double>(miradarParam.maxDistance) / 1000.0;
            ls.angle_increment = (ls.angle_max - ls.angle_min) /
                                 static_cast<double>(miradarParam.nAngle);

            ls.time_increment = 0.001;
            float a = (ls.range_max - ls.range_min) /
                      (static_cast<double>(miradarParam.nDistance - 1));

            for (int i = 0; i < miradarParam.nAngle; i++) {
                int max = -1;
                float distance = -1;
                float intensity = -1;

                for (int j = 0; j < miradarParam.nDistance; j++) {
                    if (laserscanMode == 0) {
                        if (MiRadar::pixel2DB(
                                miradar.map[i * miradarParam.nDistance + j]) >
                            miradarParam.minDb) {
                            distance = a * j + ls.range_min;
                            intensity =
                                static_cast<double>(
                                    miradar
                                        .map[i * miradarParam.nDistance + j]) /
                                255.0;
                            ls.ranges.push_back(distance);
                            ls.intensities.push_back(intensity);
                            break;
                        } else if (j == miradarParam.nDistance - 1) {
                            ls.ranges.push_back(999);
                            ls.intensities.push_back(0);
                        }
                    } else {
                        if (miradar.map[i * miradarParam.nDistance + j] > max) {
                            max = miradar.map[i * miradarParam.nDistance + j];
                            distance = a * j + ls.range_min;
                            intensity =
                                static_cast<double>(
                                    miradar
                                        .map[i * miradarParam.nDistance + j]) /
                                255.0;
                        }
                    }
                }
                if (laserscanMode == 1) {
                    ls.ranges.push_back(distance);
                    ls.intensities.push_back(intensity);
                }
            }
            ls.header.stamp = ros::Time::now();
            laserPub.publish(ls);
        }
    }

    void publishPointClouds() {
        pcl::PointCloud<pcl::PointXYZI> ladarpc;

        if (miradar.nAngle * miradar.nDistance == miradar.map.size()) {
            sensor_msgs::PointCloud2 pointcloud_msg;

            sensor_msgs::LaserScan ls;
            double angle_min =
                DEG2RAG(-static_cast<double>(miradarParam.maxAngle));
            double angle_max =
                DEG2RAG(static_cast<double>(miradarParam.maxAngle));
            double range_min =
                static_cast<double>(miradarParam.minDistance) / 1000.0;
            double range_max =
                static_cast<double>(miradarParam.maxDistance) / 1000.0;
            double angle_increment = (angle_max - angle_min) /
                                     static_cast<double>(miradarParam.nAngle);
            double distance_increment =
                (range_max - range_min) /
                static_cast<double>(miradarParam.nDistance);
            float a = (range_max - range_min) /
                      (static_cast<double>(miradarParam.nDistance - 1));
            float a_angle = (angle_max - angle_min) /
                            (static_cast<double>(miradarParam.nAngle - 1));

            for (int i = 0; i < miradarParam.nAngle; i++) {
                int max = -1;
                float distance = -1;
                float intensity = -1;
                float angle = 0;

                for (int j = 0; j < miradarParam.nDistance; j++) {
                    if (MiRadar::pixel2DB(
                            miradar.map[i * miradarParam.nDistance + j]) >
                        miradarParam.minDb) {
                        distance = a * j + range_min + distance_increment / 2;
                        angle = a_angle * i + angle_min + angle_increment / 2;
                        intensity =
                            static_cast<double>(
                                miradar.map[i * miradarParam.nDistance + j]) /
                            255.0;
                        double y = distance * sin(angle);
                        double x = distance * cos(angle);
                        pcl::PointXYZI point;
                        point.x = x;
                        point.y = y;
                        point.z = 0;
                        point.intensity = intensity;
                        ladarpc.push_back(point);
                    }
                }
            }
            pcl::toROSMsg(ladarpc, pointcloud_msg);
            pointcloud_msg.header.frame_id = "miradar";
            pcPub.publish(pointcloud_msg);
        }
    }

    void publishPPI() {
        if (miradar.ppiEntries.size() > 0) {
            miradar_node::PPIData ppidata;

            for (int i = 0; i < miradar.ppiEntries.size(); i++) {
                miradar_node::PPI ppi;
                double distance =
                    static_cast<double>(miradar.ppiEntries[i].distance) /
                    1000.0;
                double rad =
                    DEG2RAG(static_cast<double>(miradar.ppiEntries[i].angle));
                ppi.position.y = distance * sin(rad);
                ppi.position.x = distance * cos(rad);
                ppi.position.z = 0;

                ppi.speed = miradar.ppiEntries[i].speed;
                ppi.db = miradar.ppiEntries[i].db;
                ppidata.data.push_back(ppi);
            }
            pub.publish(ppidata);
        }
    }

    void run() {
        miradar.run();
        if (isConfigUpdate) {
            changeSensorState();
            miradar.setParam(miradarParam);
            isConfigUpdate = false;
        } else if (sensorMode == 1) {
            publishPPI();
        } else if (sensorMode == 2) {
            publishMap();
            publishLaserScan();
            publishPointClouds();
        }
    }

private:
    Serial serial;
    MiRadarParam miradarParam;
    bool isConfigUpdate;
    MiRadar miradar;
    std::string deviceFile;
    int sensorMode;
    int laserscanMode;
    ros::Publisher pub;
    ros::Publisher laserPub;
    ros::Publisher mapPub;
    ros::Publisher pcPub;
    ros::Rate loopRate;
    ros::NodeHandle n;
    ros::NodeHandle param;
    dynamic_reconfigure::Server<miradar_node::MiRadarConfig> server;
    dynamic_reconfigure::Server<miradar_node::MiRadarConfig>::CallbackType f;
};

int main(int argc, char* argv[]) {
    if (ros::isInitialized()) {
        ros::init(argc, argv, "miradar_node");
    } else {
        ros::Time::init();
        ros::init(argc, argv, "miradar_node");
    }

    ros::Rate rate(10);
    ros::NodeHandle param("~");

    MiRadarROS miradarROS(rate, param);

    while (ros::ok()) {
        miradarROS.run();
        ros::spinOnce();
    }
}
