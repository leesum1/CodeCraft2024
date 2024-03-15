
#include "log.h"
#include "point.hpp"
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

class BFS {

public:
  static std::unordered_map<Point, PointCost>
  astar_search(const Point &start, const Point &goal,
               std::function<bool(Point)> is_barrier,
               std::function<int(Point)> heuristic,
               std::function<std::vector<Point>(Point)> neighbors) {

    if (is_barrier(goal)) {
      log_fatal("goal is barrier");
      assert(false);
    }

    std::priority_queue<PointCost> q;
    std::unordered_map<Point, PointCost> come_from;
    std::unordered_map<Point, int> cost_so_far;

    q.push(PointCost(start, 0));
    come_from[start] = PointCost(start, 0);
    cost_so_far[start] = 0;

    while (!q.empty()) {
      Point cur = q.top().pos;
      q.pop();

      if (start == goal) {
        return come_from;
      }

      for (auto &next : neighbors(cur)) {
        int new_cost = cost_so_far.at(cur) + 1;
        // 1. 如果新的 cost 比之前的 cost 大，那么就不用更新了
        // 2. 如果是障碍物，那么也不用更新
        // 3. 如果重复访问，那么也不用更新
        if (is_barrier(next)) {
          continue;
        }

        if (cost_so_far.find(next) == cost_so_far.end() ||
            new_cost < cost_so_far.at(next)) {

          int priority = new_cost + heuristic(next);
          cost_so_far.emplace(next, new_cost);
          q.emplace(PointCost(next, priority));
          come_from.emplace(next, PointCost(cur, new_cost));
        }
      }
    }

    return come_from;
  }

  static std::unordered_map<Point, PointCost>
  bfs_search(const Point &start, std::function<bool(Point)> goal,
             std::function<bool(Point)> is_barrier,
             std::function<std::vector<Point>(Point)> neighbors,
             int max_level) {

    std::queue<Point> q;
    std::unordered_map<Point, PointCost> come_from;
    int level = 0;

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
      path.push_back(cur);
      if (come_from.find(cur) == come_from.end()) {
        founded = false;
        return path;
      }
      cur = come_from.at(cur).pos;
    }
    return path;
  }
};
