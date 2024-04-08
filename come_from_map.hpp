#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "log.h"
#include "path_helper.hpp"
#include "point.hpp"
#include "tools.hpp"

class ComeFromMap {
    std::vector<std::unordered_map<Point, PointCost>> come_from_map_list{};
    std::vector<std::unordered_set<Point>> come_from_set_list{};
    std::vector<Point> start_pos;
    std::string name{};

public:
    ComeFromMap() = default;
    ~ComeFromMap() = default;
    int map_size() const { return come_from_map_list.front().size(); }

    void init(const std::string& name, const Point& start, const std::function<bool(const Point&)>& is_barrier,
              const std::function<std::vector<Point>(const Point&)>& neighbors,
              const std::function<int(const Point&)>& get_cost) {
        come_from_map_list.emplace_back(PATHHelper::dijkstra_search(
            start, [](const Point&) { return false; }, is_barrier, neighbors, get_cost, 300000));

        come_from_set_list.emplace_back(Tools::map_to_set(come_from_map_list.front()));
        start_pos.emplace_back(start);
        this->name = name;
        log_trace("ComeFromMap:%s,init. come_from_map size:%d  ", name.c_str(), come_from_set_list.front().size());
    }

    void init(const std::string& name, const Area& start, const std::function<bool(const Point&)>& is_barrier,
              const std::function<std::vector<Point>(const Point&)>& neighbors,
              const std::function<int(const Point&)>& get_cost) {
        const auto first_point = start.left_top;
        const auto last_point = start.width() > start.height() ? start.right_top() : start.left_bottom();
        const std::array<Point, 2> start_points = {first_point, last_point};

        for (int i = 0; i < start_points.size(); i++) {
            come_from_map_list.emplace_back(PATHHelper::dijkstra_search(
                start_points.at(i), [](const Point&) { return false; }, is_barrier, neighbors, get_cost, 300000));
            come_from_set_list.emplace_back(Tools::map_to_set(come_from_map_list.back()));
            this->start_pos.push_back(start_points.at(i));
        }
        this->name = name;
        log_trace("ComeFromMap:%s,init. come_from_map size:%d  ", name.c_str(), come_from_set_list.front().size());
    }

    bool path_exist(const Point& to) { return come_from_set_list.front().find(to) != come_from_set_list.front().end(); }

    std::optional<int> get_point_cost(const Point& to) const {
        std::optional<int> cost = std::nullopt;
        for (int i = 0; i < come_from_set_list.size(); i++) {
            const auto& cur_come_from_set = come_from_set_list.at(i);
            if (cur_come_from_set.find(to) != cur_come_from_set.end()) {
                const auto cur_cost = come_from_map_list.at(i).at(to).cost;
                if (cost.has_value() && cost.value() < cur_cost) {
                    continue;
                }
                cost = cur_cost;
            }
        }
        return cost;
    }

    int mini_cost_idx(const Point& to) const {
        std::optional<int> cost = std::nullopt;
        int idx = 0;
        for (int i = 0; i < come_from_set_list.size(); i++) {
            const auto& cur_come_from_set = come_from_set_list.at(i);
            if (cur_come_from_set.find(to) != cur_come_from_set.end()) {
                const auto cur_cost = come_from_map_list.at(i).at(to).cost;
                if (cost.has_value() && cost.value() < cur_cost) {
                    continue;
                }
                cost = cur_cost;
                idx = i;
            }
        }
        return idx;
    }

    std::vector<Point> get_path_to_point(const Point& to, bool& founded) const {
        const auto sel_index = mini_cost_idx(to);
        founded = false;
        auto path = PATHHelper::get_path(start_pos.at(sel_index), to, come_from_map_list.at(sel_index), founded);
        log_trace("%s,get_path_to_point:%d,path_size:%d", name.c_str(), founded, path.size());
        return path;
    }

    std::vector<Point> get_path_from_point(const Point& from, bool& founded) const {
        const auto sel_index = mini_cost_idx(from);
        founded = false;
        auto path =
            PATHHelper::get_path_reverse(start_pos.at(sel_index), from, come_from_map_list.at(sel_index), founded);
        log_trace("%s,get_path_from_point:%d,path_size:%d", name.c_str(), founded, path.size());
        return path;
    }

    void print_all() const {
        for (auto& p : come_from_map_list.front()) {
            log_trace("%s,come_from_map:(%d,%d), cost %d", name.c_str(), p.first.x, p.first.y, p.second.cost);
        }
    }
};
