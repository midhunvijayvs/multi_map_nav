#include <ros/ros.h>
#include <ros/package.h>
#include "multi_map_nav/wormhole_database.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "wormhole_db_initializer");
    ros::NodeHandle nh;

    // Use full path inside the package directory
    std::string db_path = ros::package::getPath("multi_map_nav") + "/wormholes.db";

    // Open or create the database
    int rc = sqlite3_open(db_path.c_str(), &db);
    if (rc) {
        ROS_ERROR("Can't open database: %s", sqlite3_errmsg(db));
        return 1;
    }

    // Create wormholes table
    const char* create_sql = R"(
        CREATE TABLE IF NOT EXISTS wormholes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            source_map TEXT,
            target_map TEXT,
            source_x REAL,
            source_y REAL,
            target_x REAL,
            target_y REAL
        );
    )";

    char* errMsg = nullptr;
    rc = sqlite3_exec(db, create_sql, nullptr, nullptr, &errMsg);
    if (rc != SQLITE_OK) {
        ROS_ERROR("SQL error while creating table: %s", errMsg);
        sqlite3_free(errMsg);
        sqlite3_close(db);
        return 1;
    }


    sqlite3_exec(db, "DELETE FROM wormholes;", nullptr, nullptr, &errMsg);

    // Insert sample wormholes
    bool inserted = true;
    inserted &= insertWormhole("room1", "room2", 1.0, 1.0, 4.8, 5.2);
    inserted &= insertWormhole("room2", "room3", 5.2, 4.8, 9.2, 9.2);
    inserted &= insertWormhole("room3", "room1", 9.0, 9.4, 1.4, 1.2);

    if (inserted) {
        ROS_INFO("Sample wormhole entries inserted successfully.");
    } else {
        ROS_WARN("Some wormhole inserts may have failed.");
    }

    sqlite3_close(db);
    ROS_INFO("Wormhole DB initialized at: %s", db_path.c_str());
    return 0;
}
