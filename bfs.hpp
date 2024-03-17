
#include "log.h"
#include "point.hpp"
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

class BFS {

public:
  static std::unordered_map<Point, PointCost>
  cut_path(const Point &start, std::function<bool(Point)> is_barrier,
           std::function<std::vector<Point>(Point)> neighbors,
           std::vector<Point> &cur_path, int cut_step, bool &success) {

    log_assert(cut_step > 0, "cut_step should be greater than 0");
    log_assert(cur_path.size() > 0, "cur_path should not be empty");
    success = false;
    Point Goal = cur_path.front();
    if (cur_path.size() > cut_step) {
      Goal = *(cur_path.end() - cut_step);
    }

    auto come_from = bfs_search(
        start, [&Goal](Point p) { return p == Goal; }, is_barrier, neighbors,
        40);
    bool founded = false;
    auto path = get_path(start, Goal, come_from, founded);

    if (founded) {

      // // 在这里才能清空砍断的路径
      // // 1. 如果路径长度大于 skip_size ,则剪切掉后面的 skip_size 个点
      // // 2. 如果路径长度小于 skip_size ,则清空路径
      // if (robots_path_list[robot_id].size() >= skip_size) {
      //   log_debug("robots_path_list before size:%d,skip_size:%d",
      //             robots_path_list[robot_id].size(), skip_size);
      //   for (int i = 0; i < skip_size; i++) {
      //     robots_path_list[robot_id].pop_back();
      //   }
      // } else {
      //   // 不足 10 个点,直接清空路径
      //   robots_path_list[robot_id].clear();
      // }

      if (cur_path.size() >= cut_step) {
        for (int i = 0; i < cut_step; i++) {
          if (i == path.size() - 1) {
            // log_assert(cur_path.back() == Goal,
            //            "path.at(i) == Goal,back:(%d,%d),Goal:(%d,%d)",
            //            cur_path.back().x, cur_path.back().y, Goal.x, Goal.y);
          }
          cur_path.pop_back();
        }
      } else {
        cur_path.clear();
      }

      for (int i = 0; i < path.size(); i++) {
        cur_path.push_back(path.at(i));
      }
      success = true;
    }

    return come_from;
  }

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
    founded = true;

    if (start == goal) {
      log_info("start == goal, no need to move");
      founded = false;
      return path;
    }

    Point cur = goal;
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
