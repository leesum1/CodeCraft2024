#pragma once

#include "berth.hpp"
#include "io_layer.hpp"
#include "log.h"
#include "point.hpp"
#include <array>
#include <unordered_map>
#include <vector>
class Manager {

  std::unordered_map<Point, Goods> goods_list;

public:
  IoLayer io_layer;

  Manager() {}
  ~Manager() {}
  void init_game() { io_layer.init(); }
  void run_game();

  void test_berths1() {

    static std::array<std::vector<Point>, 10> robots_path_list;
    static std::array<Point, 10> robots_cur_pos_list;
    static std::array<Point, 10> robots_next_pos_list;
    static std::array<int, 10> berths_id_list = {0};

    // 让机器人循环去靠泊点
    auto run_circ = [&](int robot_id, int &berth_id,
                        std::vector<Point> &robot_path) {
      Point robot_pos = io_layer.robots[robot_id].pos;
      if (io_layer.berths[berth_id].in_berth_area(robot_pos) ||
          robot_path.empty()) {
        berth_id = (berth_id + 1) % BERTH_NUM;
        bool founded = false;
        auto path = io_layer.get_berth_path(berth_id, robot_pos, founded);
        if (founded) {
          log_info("berth[%d] (%d,%d)path size:%d", berth_id,
                   io_layer.berths.at(berth_id).pos.x + 1,
                   io_layer.berths.at(berth_id).pos.y + 1, path.size());
          robot_path = path;
        } else {
          log_info("berth[%d] path not found", berth_id);
        }
      }

      if (!robot_path.empty()) {
        // io_layer.robot_move(robot_id, robot_path.back());
        robots_next_pos_list.at(robot_id) = robot_path.back();
        // robot_path.pop_back();
      }
    };

    //---------------------------------------
    // 剪切路径
    // 1. 如果路径长度大于10,则剪切掉后面的10个点
    // 2. 如果路径长度小于10,则清空路径
    // 返回剪切后的路径的最后一个点
    auto cut_path = [&](int robot_id, std::vector<Point> &robot_path) -> Point {
      auto first_point = robot_path.front();
      if (robot_path.size() > 10) {
        robot_path.erase(robot_path.end() - 10, robot_path.end());
        first_point = robot_path.back();
        robot_path.pop_back();
        return first_point;
      }

      robot_path.clear();
      return first_point;
    };

    auto is_barrier = [&](Point p) {
      bool is_barrier1 = io_layer.game_map.is_barrier(p);
      bool is_barrier2 =
          std::any_of(robots_cur_pos_list.begin(), robots_cur_pos_list.end(),
                      [&](Point _pos) { return p == _pos; });
      bool is_barrier3 =
          std::any_of(robots_next_pos_list.begin(), robots_next_pos_list.end(),
                      [&](Point _pos) { return p == _pos; });
      return is_barrier1 || is_barrier2 || is_barrier3;
    };

    auto find_neigh = [&](Point p) { return io_layer.game_map.neighbors(p); };

    // 碰撞检测(采用剪切法)
    auto check_collision = [&](int robot_id) {
      Point robot_next_pos = robots_next_pos_list[robot_id];
      Point robot_cur_pos = robots_cur_pos_list[robot_id];

      log_trace("robot[%d] robot_cur_pos(%d,%d) robot_next_pos(%d,%d)",
                robot_id, robot_cur_pos.x, robot_cur_pos.y, robot_next_pos.x,
                robot_next_pos.y);

      for (int i = 0; i < 10; i++) {
        if (i != robot_id && robot_next_pos != Point()) {
          // 检测是否会碰撞
          if (robot_next_pos == robots_cur_pos_list[i] ||
              robot_next_pos == robots_next_pos_list[i]) {
            log_info("robot[%d] and robot[%d] collision", robot_id, i);
            Point target_point = cut_path(robot_id, robots_path_list[robot_id]);
            log_info("robot[%d] robot_cur_pos(%d,%d) new target point:(%d,%d)",
                     robot_id, robot_cur_pos.x, robot_cur_pos.y, target_point.x,
                     target_point.y);
            // 剪切法
            auto come_from_t = BFS::bfs_search(
                robot_cur_pos, [&](Point p) { return p == target_point; },
                is_barrier, find_neigh, 30);
            log_info("robot[%d] come_from_t size:%d", robot_id,
                     come_from_t.size());
            bool success;
            auto new_path = BFS::get_path(robot_cur_pos, target_point,
                                          come_from_t, success);
            if (success) {
              log_info("robot[%d] new path size:%d", robot_id, new_path.size());
              for (const auto &p : new_path) {
                log_info("new (%d,%d)", p.x, p.y);
              }
              robots_path_list[robot_id].insert(
                  robots_path_list[robot_id].end(), new_path.begin(),
                  new_path.end());

              for (const auto &p : robots_path_list[robot_id]) {
                log_info("new path (%d,%d)", p.x, p.y);
              }

              log_info("robot[%d] new path size:%d", robot_id,
                       robots_path_list[robot_id].size());
              robots_next_pos_list.at(robot_id) =
                  robots_path_list[robot_id].back();
            } else {
              log_info("robot[%d] new path not found", robot_id);
            }
          }
        }
      }
      return false;
    };

    for (int i = 0; i < 15000; i++) {
      io_layer.input_cycle();

      robots_next_pos_list.fill(Point());
      for (int i = 0; i < 10; i++) {
        robots_cur_pos_list[i] = io_layer.robots[i].pos;
      }

      for (int j = 0; j < 3; j++) {
        run_circ(j, berths_id_list[j], robots_path_list[j]);
        check_collision(j);
      }

      for (int j = 0; j < 10; j++) {
        if (robots_next_pos_list[j] != Point()) {
          io_layer.robot_move(j, robots_next_pos_list[j]);
          robots_path_list[j].pop_back();
        }
      }

      io_layer.output_cycle();
    }

    //       for (int i = 0; i < BERTH_NUM; i++) {
    //   Point cur = Point(36, 173);
    //   bool founded = false;
    //   auto path = get_berth_path(i, cur, founded);
    //   if (founded) {
    //     log_info("berth[%d] (%d,%d)path size:%d", i, berths[i].pos.x + 1,
    //              berths[i].pos.y + 1, path.size());
    //     for (const auto &p : path) {
    //       log_info("(%d,%d)", p.x, p.y);
    //     }
    //   } else {
    //     log_info("berth[%d] path not found", i);
    //   }
    // }
  }
};
