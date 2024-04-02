#pragma once

#include "log.h"
#include "path_helper.hpp"
#include "point.hpp"
#include "tools.hpp"
#include <string>
#include <unordered_map>
#include <unordered_set>

class ComeFromMap {
  std::unordered_map<Point, PointCost> come_from_map{};
  std::unordered_set<Point> come_from_set{};
  Point start_pos;
  std::string name{};

public:
  ComeFromMap() = default;
  ~ComeFromMap() = default;
  int map_size() { return come_from_set.size(); }

  void init(std::string name, const Point &start,
            std::function<bool(const Point &)> is_barrier,
            std::function<std::vector<Point>(const Point &)> neighbors,
            std::function<int(const Point &)> get_cost) {
    come_from_map = PATHHelper::dijkstra_search(
        start, [](const Point &p) { return false; }, is_barrier, neighbors,
        get_cost, 300000);
    come_from_set = Tools::map_to_set(come_from_map);
    start_pos = start;
    this->name = name;
    log_trace("ComeFromMap:%s,init. come_from_map size:%d  ", name.c_str(),
              come_from_map.size());
  }
  bool path_exist(const Point &to) {
    return come_from_set.find(to) != come_from_set.end();
  }

  std::optional<int> get_point_cost(const Point &to) {
    if (come_from_set.find(to) != come_from_set.end()) {
      return come_from_map.at(to).cost;
    }
    return std::nullopt;
  }

  std::vector<Point> get_path_to_point(const Point &to, bool &founded) {
    founded = false;
    auto path = PATHHelper::get_path(start_pos, to, come_from_map, founded);
    log_trace("%s,get_path_to_point:%d,path_size:%d", name.c_str(), founded,
              path.size());
    return path;
  }

  std::vector<Point> get_path_from_point(const Point &from, bool &founded) {

    founded = false;
    auto path =
        PATHHelper::get_path_reverse(start_pos, from, come_from_map, founded);
    log_trace("%s,get_path_from_point:%d,path_size:%d", name.c_str(), founded,
              path.size());
    return path;
  }

  void print_all() {
    for (auto &p : come_from_map) {
      log_trace("%s,come_from_map:(%d,%d), cost %d", name.c_str(), p.first.x,
                p.first.y, p.second.cost);
    }
  }
};