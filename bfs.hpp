
#include "log.h"
#include "point.hpp"
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

class BFS {

public:
  static std::unordered_map<Point, PointCost>
  bfs_search(const Point &start, std::function<bool(Point)> goal,
             std::function<bool(Point)> is_barrier,
             std::function<std::vector<Point>(Point)> neighbors,
             int max_level) {

    std::queue<Point> q;
    std::unordered_map<Point, PointCost> come_from;
    int level = 0;

    log_trace("start (%d,%d)", start.x, start.y);
    q.push(start);
    come_from[start] = PointCost(start, 0);

    while (!q.empty() && level < max_level) {
      int cur_level_size = q.size();
      for (int i = 0; i < cur_level_size; i++) {
        Point cur = q.front();

        if (goal(q.front())) {
          return come_from;
        }

        q.pop();
        for (auto &next : neighbors(cur)) {
          if (is_barrier(next) || come_from.find(next) != come_from.end()) {
            continue;
          }
          q.push(next);
          come_from[next] = PointCost(cur, come_from[cur].cost + 1);
        }
      }
      level++;
    }

    return come_from;
  }
  static std::vector<Point>
  get_path(const Point &start, const Point &goal,
           const std::unordered_map<Point, PointCost> &come_from,
           bool &founded) {
    std::vector<Point> path;
    Point cur = goal;
    founded = true;
    while (cur != start) {
      path.emplace_back(cur);
      if (come_from.find(cur) == come_from.end()) {
        founded = false;
        return path;
      }
      cur = come_from.at(cur).pos;
    }
    return path;
  }
};
