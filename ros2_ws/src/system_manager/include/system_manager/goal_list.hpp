#ifndef GOAL_LIST_HPP
#define GOAL_LIST_HPP

#include <string>
#include <vector>

struct GoalData {
    int id;
    std::string label;
    double x;
    double y;
    double theta; // Degrees
};

// 仮のゴールリスト
inline std::vector<GoalData> get_goal_list() {
    return {
        { 1, "kitchen_point",  1.5,  0.0,   0.0 },
        { 2, "living_chair",   2.0,  2.0,  90.0 },
        { 3, "entrance",      -1.0,  0.5, 180.0 }
    };
}

#endif