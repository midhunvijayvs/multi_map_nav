#include <ros/ros.h>
#include <ros/package.h>

#include <visualization_msgs/Marker.h>
#include "multi_map_nav/wormhole_database.h"
#include <ros/package.h>
#include <sqlite3.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "wormhole_marker_publisher");
    ros::NodeHandle nh;

    // Open DB from correct path
    std::string db_path = ros::package::getPath("multi_map_nav") + "/wormholes.db";
    int rc = sqlite3_open(db_path.c_str(), &db);
    if (rc) {
        ROS_ERROR("Can't open wormhole DB: %s", sqlite3_errmsg(db));
        return 1;
    }

    // Load wormholes
    std::vector<Wormhole> wormholes = getAllWormholes();
    ROS_INFO("Loaded %lu wormholes from DB.", wormholes.size());

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("wormhole_markers", 10, true);
    ros::Rate rate(0.5);  // Publish every 2 seconds

    while (ros::ok()) {
        for (size_t i = 0; i < wormholes.size(); ++i) {
            const Wormhole& wh = wormholes[i];

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "wormholes";
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point source, target;
            source.x = wh.source_x;
            source.y = wh.source_y;
            source.z = 0.1;  // Slightly above ground
            target.x = wh.target_x;
            target.y = wh.target_y;
            target.z = 0.1;

            marker.points.push_back(source);
            marker.points.push_back(target);

            marker.scale.x = 0.1;  // shaft diameter
            marker.scale.y = 0.2;  // head diameter
            marker.scale.z = 0.2;  // head length

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;

            marker.lifetime = ros::Duration();  // permanent

            marker_pub.publish(marker);



            // Text label marker
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "wormhole_labels";
            text_marker.id = 1000 + i;  // Offset ID to avoid conflicts with arrow markers
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;

            text_marker.pose.position.x = (wh.source_x + wh.target_x) / 2.0;
            text_marker.pose.position.y = (wh.source_y + wh.target_y) / 2.0;
            text_marker.pose.position.z = 0.5;  // Slightly above ground for visibility

            text_marker.scale.z = 1;  // Text height

            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0f;

            text_marker.text = wh.source_map + " → " + wh.target_map;

            text_marker.lifetime = ros::Duration();

            marker_pub.publish(text_marker);

        }

        ros::spinOnce();
        rate.sleep();
    }

    sqlite3_close(db);
    return 0;
}
