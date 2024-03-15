#pragma once

#include "goods.hpp"
#include "io_layer.hpp"
#include "log.h"
#include "point.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdlib>
#include <unordered_map>
#include <vector>
class Manager {

  std::unordered_map<Point, Goods> map_goods_list;

public:
  IoLayer io_layer;

  Manager() {}
  ~Manager() {}
  void init_game() { io_layer.init(); }
  void run_game();

  void goods_list_cycle() {
    // 将新货物添加到货物列表中
    for (int i = 0; i < io_layer.new_goods_num; i++) {
      map_goods_list[io_layer.new_goods_list[i].pos] =
          io_layer.new_goods_list[i];
      log_assert(io_layer.new_goods_list[i].pos != invalid_point,
                 "invalid goods");
    }

    // 删除 goods_list 中已经消失的货物
    for (auto it = map_goods_list.begin(); it != map_goods_list.end();) {
      if (it->second.end_cycle < io_layer.cur_cycle) {
        it = map_goods_list.erase(it);
      } else {
        it++;
      }
    }

    log_info("map_goods_list size:%d", map_goods_list.size());
  }

  void test_berths1() {

    static std::array<std::vector<Point>, ROBOT_NUM> robots_path_list;
    static std::array<Point, ROBOT_NUM> robots_cur_pos_list;
    static std::array<Point, ROBOT_NUM> robots_next_pos_list;
    static std::array<int, ROBOT_NUM> berths_id_list = {0, 1, 2, 3, 4,
                                                        5, 6, 7, 8, 9};
    static std::array<bool, ROBOT_NUM> robots_get_action;
    static std::array<bool, ROBOT_NUM> robots_pull_action;
    static std::array<Goods, ROBOT_NUM> robots_target_goods_list;
    static std::array<bool, ROBOT_NUM> robots_has_goods;

    // 让机器人循环去靠泊点
    auto go_to_berth = [&](int robot_id, int &berth_id,
                           std::vector<Point> &robot_path) {
      if (!robots_has_goods[robot_id]) {
        log_trace("robot[%d] has no goods, no need go_to_berth", robot_id);
        // 机器人没有货物,不需要去靠泊点
        return;
      }

      Point robot_pos = io_layer.robots[robot_id].pos;
      log_trace("robot[%d] berth_id:%d robot (%d,%d)", robot_id, berth_id,
                P_ARG(robot_pos));

      if (io_layer.berths[berth_id].in_berth_area(robot_pos)) {

        log_trace("robot[%d] in berth area, pull action", robot_id);
        // 机器人已经到达靠泊点
        robots_pull_action[robot_id] = true;
        io_layer.goted_goods_num++;
        io_layer.goted_goods_money += robots_target_goods_list[robot_id].money;

        robots_target_goods_list[robot_id] = invalid_goods;

        // 清空机器人的路径
        robots_path_list[robot_id].clear();
        return;
      }

      if (robot_path.empty()) {
        berth_id = (berth_id + std::rand()) % BERTH_NUM;
        bool founded = false;
        auto path = io_layer.get_berth_path(berth_id, robot_pos, founded);
        log_debug("berth[%d], come_from size %d", berth_id,
                  io_layer.berths_come_from[berth_id].size());

        if (founded) {
          log_info("berth[%d] (%d,%d)path size:%d", berth_id,
                   io_layer.berths.at(berth_id).pos.x + 1,
                   io_layer.berths.at(berth_id).pos.y + 1, path.size());
          robot_path = path;
        } else {
          // 一定可以找到路径
          log_info("berth[%d] path not found", berth_id);
          for (auto p : path) {
            log_info("path (%d,%d)", p.x, p.y);
          }
          // assert(false);
        }
      }

      // io_layer.berths[berth_id].in_berth_area(robot_pos)

      if (!robot_path.empty()) {
        robots_next_pos_list.at(robot_id) = robot_path.back();
      }
    };

    // 让机器人循环去靠泊点
    auto run_circ = [&](int robot_id, int &berth_id,
                        std::vector<Point> &robot_path) {
      if (!robots_has_goods[robot_id]) {
        // 机器人没有货物,不需要去靠泊点
        return;
      }

      Point robot_pos = io_layer.robots[robot_id].pos;
      log_debug("robot[%d] berth_id:%d robot (%d,%d)", robot_id, berth_id,
                robot_pos.x, robot_pos.y);

      if (robot_path.empty()) {
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
          assert(false);
        }
      }

      // io_layer.berths[berth_id].in_berth_area(robot_pos)

      if (!robot_path.empty()) {
        // io_layer.robot_move(robot_id, robot_path.back());
        robots_next_pos_list.at(robot_id) = robot_path.back();
        // robot_path.pop_back();
      }
    };

    // ---------------------------------------
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

    /**
     * @brief 从机器人的路径中剪切出一个点
     *
     * @param robot_id The ID of the robot.
     * @param robot_path The path of the robot.
     * @return The first point of the robot's path.
     */
    auto cut_path = [&](int robot_id, std::vector<Point> &robot_path) -> Point {
      // TODO: 如果选择到了一个机器人的位置,则需要重新选择一个点

      // 不能在这里清空路径,后续 PATH 可能会寻找失败
      auto cut_position = robot_path.front();
      if (robot_path.size() > skip_size) {
        cut_position = *(robot_path.end() - skip_size);
        return cut_position;
      }
      return cut_position;
    };

    /**
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
    **/

    auto find_neigh = [&](Point p) { return io_layer.game_map.neighbors(p); };

    // 碰撞检测(采用剪切法)
    auto check_collision = [&](int robot_id) {
      Point robot_next_pos = robots_next_pos_list[robot_id];
      Point robot_cur_pos = robots_cur_pos_list[robot_id];
      log_trace("robot[%d] check_collision start", robot_id);
      log_trace("robot[%d] robot_cur_pos(%d,%d) robot_next_pos(%d,%d)",
                robot_id, P_ARG(robot_cur_pos), P_ARG(robot_next_pos));

      // 将当前机器人与其他机器人进行碰撞检测
      for (int i = 0; i < ROBOT_NUM; i++) {
        if (i == robot_id) {
          // 不需要检测自己
          continue;
        }
        if (robot_next_pos == Point()) {
          // 不动就不需要检测
          log_trace("robot[%d] robot_next_pos is null, no need check_collision",
                    robot_id);
          continue;
        }
        if (robot_next_pos != robots_cur_pos_list[i] &&
            robot_next_pos != robots_next_pos_list[i]) {
          // 当前机器人的下一步不是其他机器人的位置或下一步
          continue;
        }

        log_info("robot[%d] and robot[%d] collision", robot_id, i);

        // 这里仅仅只是选择一个点,不能清空路径
        Point target_point = cut_path(robot_id, robots_path_list[robot_id]);

        log_info("robot[%d] robot_cur_pos(%d,%d) try to find a path to new "
                 "target point:(%d,%d)",
                 robot_id, robot_cur_pos.x, robot_cur_pos.y, target_point.x,
                 target_point.y);

        auto goal_func = [&](Point p) { return p == target_point; };
        auto heuristic_func = [&](Point n) {
          return std::abs(n.x - target_point.x) +
                 std::abs(n.y - target_point.y);
        };

        // 通过 BFS 找到一个新的路径到断点上
        auto come_from_t = BFS::bfs_search(robot_cur_pos, goal_func, is_barrier,
                                           find_neigh, 30);

        // auto come_from_t =
        //     BFS::astar_search(robot_cur_pos, target_point, is_barrier,
        //                       heuristic_func, find_neigh);

        log_info("robot[%d] come_from_t size:%d", robot_id, come_from_t.size());
        bool success;
        auto new_path =
            BFS::get_path(robot_cur_pos, target_point, come_from_t, success);
        if (success) {
          log_assert(!new_path.empty(), "new_path is empty!");

          log_debug("robots_path_list before size:%d",
                    robots_path_list[robot_id].size());
          // 在这里才能清空砍断的路径
          // 1. 如果路径长度大于 skip_size ,则剪切掉后面的 skip_size 个点
          // 2. 如果路径长度小于 skip_size ,则清空路径
          if (robots_path_list[robot_id].size() >= skip_size) {
            log_debug("robots_path_list before size:%d,skip_size:%d",
                      robots_path_list[robot_id].size(), skip_size);
            for (int i = 0; i < skip_size; i++) {
              robots_path_list[robot_id].pop_back();
            }
          } else {
            // 不足 10 个点,直接清空路径
            robots_path_list[robot_id].clear();
          }

          log_debug("robots_path_list after size:%d",
                    robots_path_list[robot_id].size());
          log_info("robot[%d] new path size:%d", robot_id, new_path.size());

          // 将新的迂回路径添加到原路径中
          for (const auto p : new_path) {
            log_info("new (%d,%d) robots_path_list size:%d", P_ARG(p),
                     robots_path_list[robot_id].size());
            robots_path_list[robot_id].push_back(p);
          }
          log_info("robot[%d] path size (after) :%d", robot_id,
                   robots_path_list[robot_id].size());

          // 更新下一步位置
          robots_next_pos_list.at(robot_id) = robots_path_list[robot_id].back();
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

          robots_path_list[robot_id] = path_last;

          if (path_last.empty()) {
            // 找不到路就停止,可能会死锁? (可以优化吗, 再次寻小路去周围的空地?)
            log_info("robot[%d] new path not found, stop move", robot_id);
            robots_next_pos_list.at(robot_id) = Point();
          } else {
            // 放弃当前路径，去新的空地避免死锁
            log_trace("robot[%d] give up current path, go to new space(%d,%d) "
                      "size:%d",
                      robot_id, P_ARG(path_last.back()), path_last.size());
            robots_next_pos_list.at(robot_id) = path_last.back();
          }
        }
      }
      log_debug("robot[%d] check_collision end", robot_id);
    };

    // auto err_debug = [&](int robot_id) {
    //   log_info(
    //       "after control robot[%d] robot_cur_pos(%d,%d)
    //       robot_next_pos(%d,%d)", robot_id, robots_cur_pos_list[robot_id].x,
    //       robots_cur_pos_list[robot_id].y, robots_next_pos_list[robot_id].x,
    //       robots_next_pos_list[robot_id].y);
    //   if (!io_layer.is_valid_move(robots_cur_pos_list[robot_id],
    //                               robots_next_pos_list[robot_id])) {
    //     log_trace("robot[%d] invalid move, path size", robot_id);
    //     for (auto pos : robots_path_list[robot_id]) {
    //       log_trace("robot[%d] path (%d,%d)", robot_id, pos.x, pos.y);
    //     }
    //   }
    // };

    auto find_new_goods = [&](int robot_id) {
      if (io_layer.cur_cycle < 20) {
        return;
      }

      if (robots_has_goods[robot_id]) {
        log_trace("robot[%d] has goods, no need find_new_goods", robot_id);
        // 机器人已经拿到货物,应该去卸货
        return;
      }

      if (!robots_target_goods_list[robot_id].is_disappeared(
              io_layer.cur_cycle) ||
          robots_target_goods_list[robot_id] != invalid_goods) {

        // 机器人已经有预定的货物,不需要再次寻找
        log_trace(
            "robot[%d] has selected goods (%d,%d), no need find_new_goods",
            robot_id, P_ARG(robots_target_goods_list[robot_id].pos));

        log_assert(!robots_path_list[robot_id].empty(),
                   "robots_path_list[robot_id] is empty!");
        robots_next_pos_list[robot_id] = robots_path_list[robot_id].back();
        return;
      }

      // 机器人位置位置信息
      Point robot_pos = robots_cur_pos_list[robot_id];
      Point robot_next_pos = robots_next_pos_list[robot_id];

      log_trace("robot[%d] find_new_goods start", robot_id);

      // // 机器人当前位置是否有货物
      // // 1. 有货物,没有消失,没有被选中
      // auto it = map_goods_list.find(robot_pos);
      // if (it != map_goods_list.end()) {
      //   if (!it->second.is_disappeared(io_layer.cur_cycle) &&
      //       it->second.be_selected == false) {
      //     log_info("robot[%d] get goods at (%d,%d)", robot_id,
      //              P_ARG(robot_pos));
      //     it->second.be_selected = true;
      //     robots_target_goods_list[robot_id] = it->second;
      //     return;
      //   }
      // }

      // 使用 bfs 寻找最近的货物
      Goods goods_final = invalid_goods;
      auto goods_goal_func = [&](Point p) {
        auto it = map_goods_list.find(p);
        if (it != map_goods_list.end()) {
          log_debug("robot[%d] find goods at (%d,%d) money:%d, end_cycle %d, "
                    "selected %d, cur_cycle:%d",
                    robot_id, P_ARG(p), it->second.money, it->second.end_cycle,
                    it->second.be_selected, io_layer.cur_cycle);

          // TODO: 加入 cost 比较时间
          if (it->second.end_cycle > io_layer.cur_cycle &&
              it->second.be_selected == false) {
            it->second.be_selected = true;
            goods_final = it->second;
            log_trace("robot[%d] find goods at (%d,%d) money:%d", robot_id,
                      P_ARG(p), it->second.money);

            return true;
          }
        }
        return false;
      };

      auto goods_is_barrier = [&](Point p) {
        return io_layer.game_map.is_barrier(p);
      };

      auto goods_come_from = BFS::bfs_search(robot_pos, goods_goal_func,
                                             goods_is_barrier, find_neigh, 30);
      log_trace("robot[%d] goods_come_from size:%d, goods_final (%d,%d)",
                robot_id, goods_come_from.size(), P_ARG(goods_final.pos));

      bool founded;
      auto goods_path =
          BFS::get_path(robot_pos, goods_final.pos, goods_come_from, founded);
      log_trace("robot[%d] goods_path size:%d founded:%d", robot_id,
                goods_path.size(), founded);
      if (founded) {
        log_assert(!goods_path.empty(), "goods_path is empty!");
        log_assert(goods_final != invalid_goods,
                   "goods_final is invalid_goods");
        log_trace("robot[%d] goods_path size:%d", robot_id, goods_path.size());

        // 更新机器人的路径,下一步位置,货物信息
        robots_path_list[robot_id] = goods_path;
        robots_next_pos_list[robot_id] = goods_path.back();
        robots_target_goods_list[robot_id] = goods_final;

      } else {
        // TODO: 机器人在范围内找不到货物,应该随机移动到范围内的一个点
        log_trace("robot[%d] goods_path not found", robot_id);
      }
    };

    for (int zhen = 0; zhen < 15000; zhen++) {
      io_layer.input_cycle();

      // 更新货物信息
      goods_list_cycle();

      // 获取机器人当前位置
      for (int i = 0; i < 10; i++) {
        robots_cur_pos_list[i] = io_layer.robots[i].pos;
        robots_next_pos_list[i] = invalid_point;
        robots_has_goods[i] = io_layer.robots[i].had_goods;
      }

      // 依次控制机器人
      for (int j = 0; j < 10; j++) {
        find_new_goods(j);
      }

      for (int j = 0; j < 10; j++) {
        go_to_berth(j, berths_id_list[j], robots_path_list[j]);
      }

      // // 依次控制机器人
      // for (int j = 0; j < 10; j++) {
      //   run_circ(j, berths_id_list[j], robots_path_list[j]);
      // }

      // // 对计算出来的下一步位置进行合法性检测
      // for (int j = 0; j < 10; j++) {
      //   err_debug(j);
      // }

      // 碰撞检测与规避
      for (int j = 0; j < 10; j++) {
        check_collision(j);
      }

      // 输出命令
      for (int j = 0; j < 10; j++) {
        auto &cur_pos = robots_cur_pos_list[j];
        auto &next_pos = robots_next_pos_list[j];
        auto &cur_berth = io_layer.berths[berths_id_list[j]];
        auto &target_goods = robots_target_goods_list[j];

        if (next_pos != invalid_point) {
          io_layer.robot_move(j, next_pos);

          bool clear_path = false;
          if (cur_berth.in_berth_area(next_pos)) {
            io_layer.robot_pull(j);
            io_layer.goted_goods_num++;
            io_layer.goted_goods_money += target_goods.money;
            robots_target_goods_list[j] = invalid_goods;
            clear_path = true;
          }

          if (target_goods != invalid_goods) {
            if (next_pos == target_goods.pos) {
              io_layer.robot_get(j);
            }
          }

          if (clear_path) {
            robots_path_list[j].clear();
          } else {
            robots_path_list[j].pop_back();
          }
        }
      }

      io_layer.output_cycle();
      io_layer.print_final_info();
    }
  }
};
