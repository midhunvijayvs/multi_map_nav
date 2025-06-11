#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_broadcaster.h>
#include "multi_map_nav/wormhole_database.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle nh;

    // Open database
    std::string db_path = ros::package::getPath("multi_map_nav") + "/wormholes.db";
    int rc = sqlite3_open(db_path.c_str(), &db);
    if (rc) {
        ROS_ERROR("Can't open database: %s", sqlite3_errmsg(db));
        return 1;
    }

    std::vector<Wormhole> wormholes = getAllWormholes();
    ROS_INFO("Loaded %lu wormholes", wormholes.size());

    tf::TransformBroadcaster br;
    ros::Rate rate(10);  // 10 Hz

    while (ros::ok()) {
        for (size_t i = 0; i < wormholes.size(); ++i) {
            const Wormhole& wh = wormholes[i];

            // Source marker TF
            tf::Transform src_tf;
            src_tf.setOrigin(tf::Vector3(wh.source_x, wh.source_y, 0.0));
            src_tf.setRotation(tf::Quaternion(0, 0, 0, 1));
            br.sendTransform(tf::StampedTransform(src_tf, ros::Time::now(), "map", "wormhole_src_" + std::to_string(i)));

            // Target marker TF
            tf::Transform tgt_tf;
            tgt_tf.setOrigin(tf::Vector3(wh.target_x, wh.target_y, 0.0));
            tgt_tf.setRotation(tf::Quaternion(0, 0, 0, 1));
            br.sendTransform(tf::StampedTransform(tgt_tf, ros::Time::now(), "map", "wormhole_tgt_" + std::to_string(i)));
        }

        ros::spinOnce();
        rate.sleep();
    }

    sqlite3_close(db);
    return 0;
}

