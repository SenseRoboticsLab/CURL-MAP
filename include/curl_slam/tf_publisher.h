//
// Created by zkc on 13/07/23.
//

#ifndef SRC_TF_PUBLISHER_H
#define SRC_TF_PUBLISHER_H
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class TfPublisher {
  public:
    TfPublisher() {
        tfb = std::make_shared<tf2_ros::TransformBroadcaster>();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "lidar";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
    }

    void run() {
        ros::Rate rate(10.0); // 10 Hz
        while (ros::ok()) {
            transformStamped.header.stamp = ros::Time::now();
            tfb->sendTransform(transformStamped);
            rate.sleep();
        }
    }

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb;
    geometry_msgs::TransformStamped transformStamped;
};

#endif // SRC_TF_PUBLISHER_H
