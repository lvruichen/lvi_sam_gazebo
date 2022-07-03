#include <ros/ros.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace std;

struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float,
                                            intensity,
                                            intensity)(float, time, time)(uint16_t,
                                                                          ring,
                                                                          ring))
using PointXYZIRT = VelodynePointXYZIRT;

class cloudHandle {
private:
    ros::NodeHandle nh_;
    ros::Subscriber subCloud;
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    sensor_msgs::PointCloud2 currentCloudMsg;
    bool hasRing;

public:
    cloudHandle(ros::NodeHandle &nh) :
        nh_(nh) {
        hasRing = false;
        subCloud = nh_.subscribe<sensor_msgs::PointCloud2>("lidar_points", 5, &cloudHandle::cloudHandler, this);
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    };
    ~cloudHandle(){};

    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg) {
        currentCloudMsg = std::move(*laserCloudMsg);
        pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        cout << laserCloudIn->points[0].time << endl;
        cout << laserCloudIn->points[100].time << endl;
        cout << laserCloudIn->points[0].ring << endl;
        cout << laserCloudIn->points[18000].ring << endl;
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cloud_handle");
    ros::NodeHandle nh;
    cloudHandle Cl(nh);
    ros::spin();
    return 0;
}
