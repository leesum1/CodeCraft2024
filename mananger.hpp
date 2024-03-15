#pragma once

#include "berth.hpp"
#include "io_layer.hpp"
#include "log.h"
#include "point.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdlib>
#include <numeric>
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
    static std::array<int, 10> berths_id_list = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    // 让机器人循环去靠泊点
    auto run_circ = [&](int robot_id, int &berth_id,
                        std::vector<Point> &robot_path) {
      Point robot_pos = io_layer.robots[robot_id].pos;
      log_debug("robot[%d] berth_id:%d robot (%d,%d)", robot_id, berth_id,
                robot_pos.x, robot_pos.y);
      if (io_layer.berths[berth_id].in_berth_area(robot_pos) ||
          robot_path.empty()) {
        berth_id = (berth_id + std::rand()) % BERTH_NUM;
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

    // 障碍物检测
    // 1. 地图障碍物
    // 2. 机器人当前位置
    // 3. 机器人下一个位置
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

    //---------------------------------------
    // 剪切路径
    // 1. 如果路径长度大于10,则剪切掉后面的10个点
    // 2. 如果路径长度小于10,则清空路径
    // 返回剪切后的路径的最后一个点
    const int skip_size = 5;

    auto cut_path = [&](int robot_id, std::vector<Point> &robot_path) -> Point {
      // TODO: 不能在这里清空路径,后续 PATH 可能会寻找失败
      auto first_point = robot_path.front();
      if (robot_path.size() > skip_size) {
        // robot_path.erase(robot_path.end() - 10, robot_path.end());
        first_point = *(robot_path.end() - skip_size);
        return first_point;
      }

      return first_point;
    };

    auto is_barrier_large = [&](Point p) {
      bool is_barrier1 = io_layer.game_map.is_barrier(p);
      if (is_barrier1) {
        return true;
      }
      for (auto &_t_pos : robots_cur_pos_list) {
        for (auto &_t_neib : io_layer.game_map.neighbors(_t_pos)) {
          if (p == _t_neib) {
            return true;
          }
        }
      }

      for (auto &_t2_pos : robots_next_pos_list) {
        for (auto &_t2_neib : io_layer.game_map.neighbors(_t2_pos)) {
          if (p == _t2_neib) {
            return true;
          }
        }
      }
      return false;
    };

    auto find_neigh = [&](Point p) { return io_layer.game_map.neighbors(p); };

    // 碰撞检测(采用剪切法)
    auto check_collision = [&](int robot_id) {
      Point robot_next_pos = robots_next_pos_list[robot_id];
      Point robot_cur_pos = robots_cur_pos_list[robot_id];

      log_trace("robot[%d] robot_cur_pos(%d,%d) robot_next_pos(%d,%d)",
                robot_id, robot_cur_pos.x, robot_cur_pos.y, robot_next_pos.x,
                robot_next_pos.y);

      // 将当前机器人与其他机器人进行碰撞检测
      for (int i = 0; i < ROBOT_NUM; i++) {
        if (i != robot_id && robot_next_pos != Point()) {

          // 检测是否会碰撞，当前机器人的下一步为其他机器人的位置或下一步

          // 检测是否会碰撞
          if (robot_next_pos == robots_cur_pos_list[i] ||
              robot_next_pos == robots_next_pos_list[i]) {
            log_info("robot[%d] and robot[%d] collision", robot_id, i);

            // 这里仅仅只是选择一个点,不能清空路径
            Point target_point = cut_path(robot_id, robots_path_list[robot_id]);
            log_info("robot[%d] robot_cur_pos(%d,%d) new target point:(%d,%d)",
                     robot_id, robot_cur_pos.x, robot_cur_pos.y, target_point.x,
                     target_point.y);

            auto goal_func = [&](Point p) { return p == target_point; };
            auto heuristic_func = [&](Point n) {
              return std::abs(n.x - target_point.x) +
                     std::abs(n.y - target_point.y);
            };

            // 剪切法
            auto come_from_t = BFS::bfs_search(robot_cur_pos, goal_func,
                                               is_barrier, find_neigh, 30);

            // auto come_from_t =
            //     BFS::astar_search(robot_cur_pos, target_point, is_barrier,
            //                       heuristic_func, find_neigh);

            log_info("robot[%d] come_from_t size:%d", robot_id,
                     come_from_t.size());
            bool success;
            auto new_path = BFS::get_path(robot_cur_pos, target_point,
                                          come_from_t, success);
            if (success) {
              // 在这里才能清空砍断的路径
              if (robots_path_list.size() > skip_size) {
                // 我们通过剪切法将这 10 个点绕过去了
                for (int i = 0; i < skip_size; i++) {
                  robots_path_list[robot_id].pop_back();
                }
              } else {
                // 不足 10 个点,直接清空路径
                robots_path_list[robot_id].clear();
              }

              log_info("robot[%d] new path size:%d", robot_id, new_path.size());
              for (const auto &p : new_path) {
                // log_info("new (%d,%d) robots_path_list size:%d", p.x, p.y,
                //          robots_path_list[robot_id].size());
                robots_path_list[robot_id].push_back(p);
              }

              log_info("robot[%d] new path size:%d", robot_id,
                       robots_path_list[robot_id].size());
              robots_next_pos_list.at(robot_id) =
                  robots_path_list[robot_id].back();
            } else {
              // 能不能再利用 BFS 的结果，找到一个路径，需要一个终点
              // 需要另外的策略
              // 1. 找不到路就停止 (在狭窄的地方会死锁)
              // 2. 随便找块空地
              auto max_point =
                  std::max_element(come_from_t.begin(), come_from_t.end(),
                                   [&](const auto &p1, const auto &p2) {
                                     return p1.second.cost < p2.second.cost;
                                   });
              bool success2 = false;
              std::vector<Point> path_last = BFS::get_path(
                  robot_cur_pos, max_point->first, come_from_t, success2);

              log_info("robot[%d] new path2 size:%d,success2:%d", robot_id,
                       path_last.size(), success2);
              assert(success2);

              robots_path_list[robot_id] = path_last;

              if (path_last.empty()) {
                robots_next_pos_list.at(robot_id) = Point();
              } else {
                robots_next_pos_list.at(robot_id) = path_last.back();
              }

              /**
              std::vector<Point> cur_neibores =
                  io_layer.game_map.neighbors(robot_cur_pos);
              log_debug("cur_robot_pos(%d,%d)", robot_cur_pos.x,
                        robot_cur_pos.y);

              std::vector<Point> empty_spaces;
              for (auto &cur_neib : cur_neibores) {
                {
                  if (!is_barrier(cur_neib)) {
                    empty_spaces.emplace_back(cur_neib);
                  }
                }
              }

              // 去周围的一个空地上
              if (!empty_spaces.empty()) {
                auto &slected_point =
                    empty_spaces.at(std::rand() % empty_spaces.size());
                // 记录方便回溯
                robots_path_list[robot_id].push_back(
                    robot_cur_pos); // 需要回溯的位置
                robots_path_list[robot_id].push_back(
                    slected_point); // 下一次移动的位置, 在移动后会 pop 掉

                // 去周围的一个空地上
                robots_next_pos_list.at(robot_id) = slected_point;

                log_trace("cur_robot_pos(%d,%d),cur_neib(%d,%d)",
                          robot_cur_pos.x, robot_cur_pos.y, slected_point.x,
                          slected_point.y);
              } else {
                log_info("robot[%d] new path not found", robot_id);
                robots_next_pos_list.at(robot_id) = Point();
              }
              **/
              // assert(false);
            }
          }
        }
      }
      log_debug("robot[%d] check_collision end", robot_id);
    };

    for (int i = 0; i < 15000; i++) {
      io_layer.input_cycle();

      robots_next_pos_list.fill(Point());
      for (int i = 0; i < 10; i++) {
        robots_cur_pos_list[i] = io_layer.robots[i].pos;
      }

      for (int j = 0; j < 10; j++) {
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
