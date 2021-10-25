#include "pch.h"

/**
 * @file a_star.cpp
 * @author vss2sn
 * @brief Contains the AStar class
 */

#include <cmath>
#include <queue>
#include <unordered_set>
#include <vector>

#include "a_star.h"

std::tuple<bool, std::vector<Node>> AStar::Plan(const Node& start,
    const Node& goal) {
    grid_ = original_grid_;
    std::priority_queue<Node, std::vector<Node>, compare_cost> open_list;
    std::unordered_set<Node, NodeIdAsHash, compare_coordinates> closed_list;

    const std::vector<Node> motion = GetMotion();
    open_list.push(start);

    // Main loop
    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();
        current.id_ = current.x_ * n_ + current.y_;
        if (CompareCoordinates(current, goal)) {
            closed_list.insert(current);
            grid_[current.x_][current.y_] = 2;
            return { true, ConvertClosedListToPath(closed_list, start, goal) };
        }
        grid_[current.x_][current.y_] = 2;  // Point opened
        for (const auto& m : motion) {
            Node new_point = current + m;
            new_point.id_ = n_ * new_point.x_ + new_point.y_;
            new_point.pid_ = current.id_;
            new_point.h_cost_ =
                std::abs(new_point.x_ - goal.x_) + std::abs(new_point.y_ - goal.y_);
            if (CompareCoordinates(new_point, goal)) {
                open_list.push(new_point);
                break;
            }
            if (checkOutsideBoundary(new_point, n_)) {
                continue;  // Check boundaries
            }
            if (grid_[new_point.x_][new_point.y_] != 0) {
                continue;  // obstacle or visited
            }
            open_list.push(new_point);
        }
        closed_list.insert(current);
    }
    return { false, {} };
}

std::vector<Node> AStar::ConvertClosedListToPath(
    std::unordered_set<Node, NodeIdAsHash, compare_coordinates>& closed_list,
    const Node& start, const Node& goal) {
    auto current = *closed_list.find(goal);
    std::vector<Node> path;
    while (!CompareCoordinates(current, start)) {
        path.push_back(current);
        const auto it = closed_list.find(Node(current.pid_ / n_, current.pid_ % n_, 0, 0, current.pid_));
        if (it != closed_list.end()) {
            current = *it;
        }
        else {
            std::cout << "Error in calculating path \n";
            return {};
        }
    }
    path.push_back(start);
    return path;
}
