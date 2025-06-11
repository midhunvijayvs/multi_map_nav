#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::PoseStamped current_pose;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
    current_pose.header.stamp = ros::Time::now();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_tf_simulator");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("teleport_pose", 10, poseCallback);
    tf::TransformBroadcaster br;

    ros::Rate rate(10.0);

    while (ros::ok()) {
    ros::spinOnce();

    if (current_pose.header.frame_id.empty()) {
        rate.sleep();
        continue;
    }

    tf::Quaternion q(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    );

    // Check if quaternion is valid
    if (q.length2() < 1e-6) {
        ROS_WARN_THROTTLE(5, "Skipping TF broadcast due to invalid quaternion.");
        rate.sleep();
        continue;
    }

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z
    ));
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(
        transform,
        current_pose.header.stamp,
        current_pose.header.frame_id,  // parent frame (e.g., "map")
        "base_link"                     // child frame
    ));

    rate.sleep();
}


    return 0;
}
