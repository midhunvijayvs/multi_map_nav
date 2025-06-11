#pragma once

#include <string>
#include <vector>
#include <sqlite3.h>

extern sqlite3* db;

// Struct to represent a wormhole
struct Wormhole {
    std::string source_map;
    std::string target_map;
    double source_x;
    double source_y;
    double target_x;
    double target_y;
};

// DB insert
bool insertWormhole(const std::string& source_map,
                    const std::string& target_map,
                    double source_x, double source_y,
                    double target_x, double target_y);



// DB query
std::vector<Wormhole> getAllWormholes();
