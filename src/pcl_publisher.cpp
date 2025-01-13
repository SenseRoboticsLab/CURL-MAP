
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("points2", 1);
    Eigen::Matrix3d A;
    for (int i = 0; i < 9; ++i) {
        A << double(i);
    }

    std::cout << A << std::endl;
    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = "some_tf_frame";
    msg->height = msg->width = 1;
    msg->points.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));

    ros::Rate loop_rate(4);
    while (nh.ok()) {
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        pub.publish(*msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}