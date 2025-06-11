#include <ros/package.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <multi_map_nav/WormholeJumpAction.h>
#include "multi_map_nav/wormhole_database.h"
#include <geometry_msgs/PoseStamped.h>



class WormholeJumpAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<multi_map_nav::WormholeJumpAction> as_;
    std::string action_name_;
    multi_map_nav::WormholeJumpFeedback feedback_;
    multi_map_nav::WormholeJumpResult result_;
    ros::Publisher pose_pub_;

public:
    WormholeJumpAction(std::string name)
        : as_(nh_, name, boost::bind(&WormholeJumpAction::executeCB, this, _1), false),
          action_name_(name) {
        
        // Open the wormhole database
        int rc = sqlite3_open((ros::package::getPath("multi_map_nav") + "/wormholes.db").c_str(), &db);
        if (rc) {
            ROS_ERROR("Cannot open database: %s", sqlite3_errmsg(db));
            db = nullptr;
        } else {
            ROS_INFO("Connected to wormholes.db successfully.");
        }
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("teleport_pose", 10);

        as_.start();
    }

    ~WormholeJumpAction() {
        if (db) sqlite3_close(db);
    }

    void executeCB(const multi_map_nav::WormholeJumpGoalConstPtr &goal) {
        ROS_INFO("Requested jump from [%s] to [%s]", goal->source_map.c_str(), goal->target_map.c_str());

        // Prepare SQL query
        std::string query = "SELECT source_x, source_y, target_x, target_y FROM wormholes WHERE source_map='" + goal->source_map +
                            "' AND target_map='" + goal->target_map + "' LIMIT 1;";
        
        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr);
        if (rc != SQLITE_OK) {
            ROS_ERROR("Failed to query database: %s", sqlite3_errmsg(db));
            result_.success = false;
            result_.message = "Database query failed.";
            as_.setAborted(result_);
            return;
        }

        if (sqlite3_step(stmt) != SQLITE_ROW) {
            ROS_WARN("Wormhole not found.");
            sqlite3_finalize(stmt);
            result_.success = false;
            result_.message = "No such wormhole found.";
            as_.setAborted(result_);
            return;
        }

        double sx = sqlite3_column_double(stmt, 0);
        double sy = sqlite3_column_double(stmt, 1);
        double tx = sqlite3_column_double(stmt, 2);
        double ty = sqlite3_column_double(stmt, 3);
        sqlite3_finalize(stmt);

        ROS_INFO("Teleporting from (%f, %f) to (%f, %f)", sx, sy, tx, ty);

        // Simulate progress
        for (int i = 0; i <= 100; i += 10) {
            if (as_.isPreemptRequested() || !ros::ok()) {
                as_.setPreempted();
                return;
            }
            feedback_.progress = i / 100.0;
            as_.publishFeedback(feedback_);
            ros::Duration(0.2).sleep();
        }

        // Send teleportation info to RViz via tf_broadcaster (optional, see node below)
        // Construct the teleport pose
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = tx;
        pose.pose.position.y = ty;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;  // No rotation

        // Publish it
        ROS_INFO("Publishing teleport_pose with frame_id: %s", pose.header.frame_id.c_str());

        pose_pub_.publish(pose);


        result_.success = true;
        result_.message = "Teleportation successful.";
        as_.setSucceeded(result_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigate_wormhole_action");
    WormholeJumpAction server("navigate_wormhole");
    ros::spin();
    return 0;
}
