#include <ros/ros.h>
#include "multi_map_nav/wormhole_database.h"

sqlite3* db = nullptr;  // Global database pointer

bool insertWormhole(const std::string& source_map,
                    const std::string& target_map,
                    double source_x, double source_y,
                    double target_x, double target_y) {
    if (!db) return false;

    const char* sql = "INSERT INTO wormholes (source_map, target_map, source_x, source_y, target_x, target_y) VALUES (?, ?, ?, ?, ?, ?);";
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        ROS_ERROR("Failed to prepare insert: %s", sqlite3_errmsg(db));
        return false;
    }

    sqlite3_bind_text(stmt, 1, source_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_double(stmt, 3, source_x);
    sqlite3_bind_double(stmt, 4, source_y);
    sqlite3_bind_double(stmt, 5, target_x);
    sqlite3_bind_double(stmt, 6, target_y);

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        ROS_ERROR("Failed to execute insert: %s", sqlite3_errmsg(db));
        return false;
    }

    return true;
}

std::vector<Wormhole> getAllWormholes() {
    std::vector<Wormhole> wormholes;
    const char* sql = "SELECT source_map, target_map, source_x, source_y, target_x, target_y FROM wormholes;";
    sqlite3_stmt* stmt;

    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) == SQLITE_OK) {
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            Wormhole wh;
            wh.source_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            wh.target_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            wh.source_x = sqlite3_column_double(stmt, 2);
            wh.source_y = sqlite3_column_double(stmt, 3);
            wh.target_x = sqlite3_column_double(stmt, 4);
            wh.target_y = sqlite3_column_double(stmt, 5);
            wormholes.push_back(wh);
        }
        sqlite3_finalize(stmt);
    } else {
        std::cerr << "Failed to fetch wormholes: " << sqlite3_errmsg(db) << std::endl;
    }

    return wormholes;
}

