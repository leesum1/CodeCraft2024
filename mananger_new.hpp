#pragma once

#include "goods.hpp"
#include "io_laye_new.hpp"
#include "log.h"
#include "point.hpp"
#include "robot.hpp"
#include "tools.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

const int max_robots_num_in_berth = 3;

class ManagerNew {

public:
  IoLayerNew io_layer;

  // std::array<int, ROBOT_NUM> berths_id_list;

  ManagerNew() = default;
  ~ManagerNew() = default;
  void init_game() { io_layer.init(); }

  // auto get_is_barrier_lambda_v3(const int robot_id) {
  //   auto is_barrier = [&](const Point &p) {
  //     bool is_barrier1 = io_layer.game_map.is_barrier(p);
  //     //
  //     bool is_barrier2 = false;

  //     // 与其他机器人的当前位置碰撞
  //     for (int i = 0; i < ROBOT_NUM; i++) {
  //       if (i == robot_id) {
  //         continue;
  //       }
  //       if (p == robots_cur_pos_list[i]) {
  //         is_barrier2 = true;
  //         break;
  //       }
  //     }

  //     bool is_barrier3 = false;
  //     // is_barrier3 =
  //     //     low_prio_next_pos_collision_with_others_next_pos(robot_id, p);

  //     // // 与优先级高的机器人的下一个位置碰撞
  //     for (int i = 0; i < ROBOT_NUM; i++) {
  //       if (i == robot_id) {
  //         continue;
  //       }
  //       if (p == robots_next_pos_list[i]) {
  //         is_barrier3 = true;
  //         break;
  //       }
  //     }
  //     return is_barrier1 || is_barrier2 || is_barrier3;
  //   };
  //   return is_barrier;
  // }

  // auto get_is_barrier_lambda_v2(const int robot_id) {
  //   auto is_barrier = [&](const Point &p) {
  //     bool is_barrier1 = io_layer.game_map.is_barrier(p);
  //     //
  //     bool is_barrier2 = false;

  //     // 与其他机器人的当前位置碰撞
  //     for (int i = 0; i < ROBOT_NUM; i++) {
  //       if (i == robot_id) {
  //         continue;
  //       }
  //       if (p == robots_cur_pos_list[i]) {
  //         is_barrier2 = true;
  //         break;
  //       }
  //     }

  //     bool is_barrier3 = false;
  //     // is_barrier3 =
  //     //     low_prio_next_pos_collision_with_others_next_pos(robot_id, p);

  //     // // 与优先级高的机器人的下一个位置碰撞
  //     for (int i = 0; i < ROBOT_NUM; i++) {
  //       if (i == robot_id) {
  //         continue;
  //       }
  //       if (robots_has_pass_collision[i] == false) {
  //         continue;
  //       }
  //       if (p == robots_next_pos_list[i]) {
  //         is_barrier3 = true;
  //         break;
  //       }
  //     }
  //     return is_barrier1 || is_barrier2 || is_barrier3;
  //   };
  //   return is_barrier;
  // }

  // auto get_is_barrier_lambda_v1(const int robot_id) {
  //   auto is_barrier = [&](const Point &p) {
  //     bool is_barrier1 = io_layer.game_map.is_barrier(p);
  //     //
  //     bool is_barrier2 = false;

  //     is_barrier2 =
  //         low_prio_next_pos_collision_with_others_cur_pos(robot_id, p);

  //     // // 与其他机器人的当前位置碰撞
  //     // for (int i = 0; i < ROBOT_NUM; i++) {
  //     //   if (i == robot_id) {
  //     //     continue;
  //     //   }
  //     //   if (p == robots_cur_pos_list[i]) {
  //     //     is_barrier2 = true;
  //     //     break;
  //     //   }
  //     // }

  //     bool is_barrier3 = false;
  //     is_barrier3 =
  //         low_prio_next_pos_collision_with_others_next_pos(robot_id, p);

  //     // // 与优先级高的机器人的下一个位置碰撞
  //     // for (int i = 0; i < ROBOT_NUM; i++) {
  //     //   if (i == robot_id) {
  //     //     continue;
  //     //   }
  //     //   if (robots_has_pass_collision[i] == false) {
  //     //     continue;
  //     //   }
  //     //   if (p == robots_next_pos_list[i]) {
  //     //     is_barrier3 = true;
  //     //     break;
  //     //   }
  //     // }
  //     return is_barrier1 || is_barrier2 || is_barrier3;
  //   };
  //   return is_barrier;
  // }

  // ---------------------------------------
  // 障碍物检测
  // 1. 地图障碍物
  // 2. 机器人当前位置
  // 3. 机器人下一个位置
  auto get_is_barrier_lambda() {
    auto is_barrier = [&](const Point &p) {
      bool is_barrier1 = io_layer.game_map.is_barrier_for_robot(p);

      bool is_barrier2 =
          std::any_of(io_layer.robots.begin(), io_layer.robots.end(),
                      [&](Robot &robot) { return p == robot.pos; });
      bool is_barrier3 = std::any_of(
          io_layer.robots.begin(), io_layer.robots.end(), [&](Robot &robot) {
            return p == robot.next_pos_before_collision_check;
          });
      return is_barrier1 || is_barrier2 || is_barrier3;
    };
    return is_barrier;
  }

  auto get_find_neighbor_lambda() {
    auto find_neighbor = [&](const Point &p) {
      return io_layer.game_map.neighbors_for_robot(p, false);
    };
    return find_neighbor;
  }

  // // 当前机器人为高优先级
  // int high_prio_next_pos_collision_with_others_cur_pos(const int robot_id) {
  //   const Point &robot_next_pos = robots_next_pos_list[robot_id];
  //   if (robot_next_pos == invalid_point) {
  //     return -1;
  //   }
  //   if (robot_next_pos == stop_point) {
  //     return -1;
  //   }
  //   for (int i = 0; i < ROBOT_NUM; i++) {
  //     if (i == robot_id) {
  //       // 不需要与自己检查
  //       continue;
  //     }

  //     if (robots_has_pass_collision[i]) {
  //       // 只和没有通过碰撞检测的机器人检测
  //       continue;
  //     }

  //     if (robot_next_pos == robots_cur_pos_list[i]) {
  //       return i;
  //     }
  //   }

  //   return -1;
  // }

  // // 当前机器人为高优先级
  // int high_prio_next_pos_collision_with_others_cur_pos_step2(
  //     const int robot_id) {
  //   const Point &robot_next_pos = robots_next_pos_list_copy[robot_id];
  //   if (robot_next_pos == invalid_point) {
  //     return -1;
  //   }
  //   if (robot_next_pos == stop_point) {
  //     return -1;
  //   }
  //   for (int i = 0; i < ROBOT_NUM; i++) {
  //     if (i == robot_id) {
  //       // 不需要与自己检查
  //       continue;
  //     }

  //     if (robot_next_pos == robots_cur_pos_list[i]) {
  //       return i;
  //     }
  //   }

  //   return -1;
  // }

  // // 当前机器人为高优先级
  // bool high_prio_next_pos_collision_with_others_next_pos(const int robot_id)
  // {
  //   const Point &robot_next_pos = robots_next_pos_list[robot_id];

  //   if (robot_next_pos == invalid_point) {
  //     return false;
  //   }
  //   if (robot_next_pos == stop_point) {
  //     return false;
  //   }

  //   for (int i = 0; i < ROBOT_NUM; i++) {
  //     if (i == robot_id) {
  //       // 不需要与自己检查
  //       continue;
  //     }

  //     if (robots_has_pass_collision[i]) {
  //       // 只和没有通过碰撞检测的机器人检测
  //       continue;
  //     }

  //     if (robot_next_pos == robots_next_pos_list[i]) {
  //       return true;
  //     }
  //   }

  //   return false;
  // }
  // 当前机器人为低优先级
  // std::vector<int>
  // low_prio_next_pos_collision_with_others_cur_pos(const int robot_id) {
  //   const Point &robot_next_pos = robots_next_pos_list[robot_id];

  //   std::vector<int> robots_list{};
  //   if (robot_next_pos == invalid_point) {
  //     return robots_list;
  //   }
  //   if (robot_next_pos == stop_point) {
  //     return robots_list;
  //   }

  //   for (int i = 0; i < ROBOT_NUM; i++) {
  //     if (i == robot_id) {
  //       // 不需要与自己检查
  //       continue;
  //     }

  //     if (!robots_has_pass_collision[i]) {
  //       // 只和通过碰撞检测的机器人检测
  //       continue;
  //     }

  //     if (robot_next_pos == robots_cur_pos_list[i]) {
  //       robots_list.emplace_back(i);
  //     }
  //   }

  //   return robots_list;
  // }

  // // 当前机器人为低优先级
  // bool low_prio_next_pos_collision_with_others_next_pos(const int robot_id,
  //                                                       const Point &p) {

  //   if (p == invalid_point) {
  //     return false;
  //   }
  //   if (p == stop_point) {
  //     return false;
  //   }

  //   for (int i = 0; i < ROBOT_NUM; i++) {
  //     if (robot_id == i) {
  //       continue;
  //     }

  //     if (!robots_has_pass_collision[i]) {
  //       // 只和通过碰撞检测的机器人检测
  //       continue;
  //     }

  //     if (p == robots_next_pos_list[i]) {
  //       return true;
  //     }
  //   }

  //   return false;
  // }

  // // 当前机器人为低优先级
  // std::vector<int>
  // low_prio_next_pos_collision_with_others_next_pos(const int robot_id) {
  //   const Point &robot_next_pos = robots_next_pos_list[robot_id];
  //   std::vector<int> robots_list{};
  //   if (robot_next_pos == invalid_point) {
  //     return robots_list;
  //   }
  //   if (robot_next_pos == stop_point) {
  //     return robots_list;
  //   }

  //   for (int i = 0; i < ROBOT_NUM; i++) {
  //     if (i == robot_id) {
  //       // 不需要与自己检查
  //       continue;
  //     }

  //     if (!robots_has_pass_collision[i]) {
  //       // 只和通过碰撞检测的机器人检测
  //       continue;
  //     }

  //     if (robot_next_pos == robots_next_pos_list[i]) {
  //       robots_list.emplace_back(i);
  //     }
  //   }

  //   return robots_list;
  // }

  // // 当前机器人为低优先级
  // std::vector<Point>
  // low_prio_cur_pos_collision_with_others_next_pos(const int robot_id) {
  //   const Point &robot_cur_pos = robots_cur_pos_list[robot_id];

  //   std::vector<Point> next_pos_list{};
  //   if (robot_cur_pos == invalid_point) {
  //     return next_pos_list;
  //   }
  //   if (robot_cur_pos == stop_point) {
  //     return next_pos_list;
  //   }

  //   for (int i = 0; i < ROBOT_NUM; i++) {
  //     if (i == robot_id) {
  //       // 不需要与自己检查
  //       continue;
  //     }

  //     if (!robots_has_pass_collision[i]) {
  //       // 只和通过碰撞检测的机器人检测
  //       continue;
  //     }

  //     if (robot_cur_pos == robots_next_pos_list[i]) {
  //       next_pos_list.emplace_back(robots_next_pos_list[i]);
  //     }
  //   }

  //   return next_pos_list;
  // }

  /**
    // 碰撞检测(采用剪切法)
    void check_collision_v1(int robot_id) {
      robots_has_pass_collision[robot_id] = true;

      const Point robot_next_pos = robots_next_pos_list[robot_id];
      const Point robot_cur_pos = robots_cur_pos_list[robot_id];
      log_trace("robot[%d] check_collision start  robot_cur_pos(%d,%d) "
                "robot_next_pos(%d,%d)",
                robot_id, P_ARG(robot_cur_pos), P_ARG(robot_next_pos));

      if (robot_next_pos == invalid_point || robot_next_pos == stop_point) {
        // 不动就不需要检测
        log_trace("robot[%d] robot_next_pos is null, no need check_collision",
                  robot_id);
        return;
      }

      auto other_high_collision_robot_id =
          low_prio_next_pos_collision_with_others_cur_pos(robot_id);
      if (!other_high_collision_robot_id.empty()) {
        // 1. 下一次移动的位置与优先级高的机器人的当前位置冲突:
        // 选择一个方向移走(为优先级高的开路)

        for (const int id_tmp : other_high_collision_robot_id) {
          log_debug(
              "robot[%d] low_prio_next_pos_collision_with_others[%d]_cur_pos",
              robot_id, id_tmp);
        }

        bool cut_success = false;
        auto come_from_t = PATHHelper::cut_path(
            robot_cur_pos, get_is_barrier_lambda_v1(robot_id),
            get_find_neighbor_lambda(), robots_path_list[robot_id], 10,
            cut_success);
        if (cut_success) {
          robots_next_pos_list.at(robot_id) = robots_path_list[robot_id].back();
          log_debug("robot[%d] cut path success", robot_id);
        } else {
          const auto others_next_pos =
              low_prio_cur_pos_collision_with_others_next_pos(robot_id);

          std::vector<Point> other_cur_pos_list(
              other_high_collision_robot_id.size());

          std::transform(other_high_collision_robot_id.begin(),
                         other_high_collision_robot_id.end(),
                         other_cur_pos_list.begin(),
                         [&](const int id) { return robots_cur_pos_list[id]; });

          const auto flag_tmp =
              std::any_of(other_high_collision_robot_id.begin(),
                          other_high_collision_robot_id.end(), [&](const int id)
    { return robots_next_pos_list[id] == invalid_point;
                          });

          if (!others_next_pos.empty() || flag_tmp) {
            Point select_space = invalid_point;
            int select_cost = 10000;
            auto is_barrier = get_is_barrier_lambda_v1(robot_id);

            for (const auto &p : come_from_t) {
              if (!is_barrier(p.first)) {
                if (std::any_of(other_cur_pos_list.begin(),
                                other_cur_pos_list.end(), [&](const Point &pos)
    { return Point::at_same_row_or_col(pos, p.first);
                                })) {
                  continue;
                }

                if (p.second.cost < select_cost) {
                  select_space = p.first;
                  select_cost = p.second.cost;
                }
              }
            }
            if (select_space == invalid_point) {
              // 没有空地可以走了, 高优先级需要让路, 需要设置一个标识位
              robots_next_pos_list[robot_id] = invalid_point;

              robots_need_move_but_not_move.insert(
                  robots_need_move_but_not_move.end(),
                  other_high_collision_robot_id.begin(),
                  other_high_collision_robot_id.end());

              for (const int id_tmp : other_high_collision_robot_id) {
                log_debug("robot[%d] no space move,other robot[%d] need move",
                          robot_id, id_tmp);
              }

            } else {
              bool bt_success;

              const auto bt_path = PATHHelper::get_path(
                  robot_cur_pos, select_space, come_from_t, bt_success);
              log_assert(bt_success, "error, bt_success is false");

              PATHHelper::add_backtrace_path(robot_cur_pos,
                                             robots_path_list[robot_id],
    bt_path); robots_next_pos_list.at(robot_id) =
                  robots_path_list[robot_id].back();
              log_debug("robot[%d] add bt path size:%d", robot_id,
                        bt_path.size());
            }
          } else {
            robots_next_pos_list[robot_id] = invalid_point;
            log_debug("robot[%d] no need move", robot_id);
          }
        }
      }

      other_high_collision_robot_id =
          low_prio_next_pos_collision_with_others_next_pos(robot_id);
      if (!other_high_collision_robot_id.empty()) {
        log_debug("robot[%d],low_prio_next_pos_collision_with_others_next_pos",
                  robot_id);
        //   2. 下一次移动的位置与优先级高的机器人的下一次位置冲突: 取消当前移动
        //   (给优先级高的让路) (是否可以选择一个方向移走?)

        std::vector<Point> path_copy =
            std::vector<Point>(robots_path_list[robot_id]);

        bool cut_success = false;
        auto come_from_t = PATHHelper::cut_path(
            robot_cur_pos, get_is_barrier_lambda_v1(robot_id),
            get_find_neighbor_lambda(), path_copy, 10, cut_success);
        if (cut_success) {

          robots_next_pos_list[robot_id] = invalid_point;
        } else {
          const auto others_next_pos =
              low_prio_cur_pos_collision_with_others_next_pos(robot_id);

          std::vector<Point> other_cur_pos_list(
              other_high_collision_robot_id.size());

          std::transform(other_high_collision_robot_id.begin(),
                         other_high_collision_robot_id.end(),
                         other_cur_pos_list.begin(),
                         [&](const int id) { return robots_cur_pos_list[id]; });

          Point select_space = invalid_point;
          int select_cost = 10000;
          auto is_barrier = get_is_barrier_lambda_v1(robot_id);

          for (const auto &p : come_from_t) {
            if (!is_barrier(p.first)) {
              if (std::any_of(other_cur_pos_list.begin(),
                              other_cur_pos_list.end(), [&](const Point &pos) {
                                return Point::at_same_row_or_col(pos, p.first);
                              })) {
                continue;
              }

              if (p.second.cost < select_cost) {
                select_space = p.first;
                select_cost = p.second.cost;
              }
            }
          }
          if (select_space == invalid_point) {
            robots_next_pos_list[robot_id] = invalid_point;

            for (const int id_tmp : other_high_collision_robot_id) {
              log_debug("robot[%d] no space move,other robot[%d] need move",
                        robot_id, id_tmp);
            }

          } else {
            bool bt_success;

            const auto bt_path = PATHHelper::get_path(robot_cur_pos,
    select_space, come_from_t, bt_success); log_assert(bt_success, "error,
    bt_success is false");

            PATHHelper::add_backtrace_path(robot_cur_pos,
                                           robots_path_list[robot_id], bt_path);
            robots_next_pos_list.at(robot_id) =
    robots_path_list[robot_id].back(); log_debug("robot[%d] add bt path
    size:%d", robot_id, bt_path.size());
          }
        }
      }

      if (high_prio_next_pos_collision_with_others_next_pos(robot_id)) {
        log_debug("robot[%d],high_prio_next_pos_collision_with_others_next_pos",
                  robot_id);
        //
    下一次移动位置与优先级低的下一次移动的位置冲突:继续移动(因为优先级低的会取消本次移动)
      }

      int other_collision_robot_id =
          high_prio_next_pos_collision_with_others_cur_pos(robot_id);

      if (other_collision_robot_id != -1) {
        log_debug("robot[%d],high_prio_next_pos_collision_with_other[%d]_cur_pos",
                  robot_id, other_collision_robot_id);

        // 下一次移动位置与优先级低的机器人当前位置冲突: 停止当前移动,
        // 原地不动(在下一个周期,其他优先级低的会让出位置)

        robots_next_pos_list[robot_id] = invalid_point;
      }
    }

    void check_collison_step2(const int robot_id) {

      const auto robot_cur_pos = robots_cur_pos_list[robot_id];
      int other_collision_robot_id =
          high_prio_next_pos_collision_with_others_cur_pos_step2(robot_id);

      if (other_collision_robot_id == -1) {
        return;
      }

      // 如果低优先级的机器人不能移动,则高优先级的机器人移动
      if (std::find(robots_need_move_but_not_move.begin(),
                    robots_need_move_but_not_move.end(),
                    robot_id) != robots_need_move_but_not_move.end()) {

        bool cut_success = false;
        auto come_from_t = PATHHelper::cut_path(
            robot_cur_pos, get_is_barrier_lambda_v3(robot_id),
            get_find_neighbor_lambda(), robots_path_list[robot_id], 10,
            cut_success);

        Point select_space = invalid_point;
        int select_cost = 10000;
        auto is_barrier = get_is_barrier_lambda_v3(robot_id);
        const auto other_cur_pos =
    robots_cur_pos_list[other_collision_robot_id];

        for (const auto &p : come_from_t) {
          if (!is_barrier(p.first)) {
            if (p.first.x == other_cur_pos.x || p.first.y == other_cur_pos.y) {
              continue;
            }

            if (p.second.cost < select_cost && p.second.cost > 1) {
              select_space = p.first;
              select_cost = p.second.cost;
            }
          }
        }
        if (select_space == invalid_point) {
          // 没有空地可以走了
          robots_next_pos_list[robot_id] = invalid_point;
          log_debug("robot[%d] check_colliso with [%d]no space to move",
    robot_id, other_collision_robot_id); } else { bool bt_success;

          const auto bt_path = PATHHelper::get_path(robot_cur_pos, select_space,
                                                    come_from_t, bt_success);
          log_assert(bt_success, "error, bt_success is false");

          PATHHelper::add_backtrace_path(robot_cur_pos,
                                         robots_path_list[robot_id], bt_path);
          robots_next_pos_list.at(robot_id) = robots_path_list[robot_id].back();
        }
      }
    }
  */
  // void err_debug(int robot_id){
  //     // if (!io_layer.is_valid_move(robots_cur_pos_list[robot_id],
  //     //                             robots_next_pos_list[robot_id])) {
  //     //   log_trace("robot[%d] invalid move, path size", robot_id);

  //     //   for (auto pos : robots_path_list[robot_id]) {
  //     //     log_trace("robot[%d] path (%d,%d)", robot_id, pos.x, pos.y);
  //     //   }
  //     // }
  // };

  void berth_cycle() {
    for (auto &berth : io_layer.berths) {
      berth.tick(io_layer.cur_cycle);
    }
  }

  // // 选择价值最高的泊位
  // int select_best_berth() {
  //   int target_berth_id = -1;
  //   int max_value = -1;

  //   const int remine_time = 15000 - (io_layer.cur_cycle);
  //   for (int i = 0; i < BERTH_NUM; i++) {
  //     // 如果泊位已经被占用,则跳过
  //     if (berths_visit[i] == true || io_layer.berths[i].is_baned) {
  //       continue;
  //     }
  //     // 选择去价值最高的港口就是最优解
  //     const int value = io_layer.berths[i].goods_value() +
  //                       io_layer.berths[i].money_in_1000cycle();
  //     const int cur_transport_time = io_layer.berths[i].transport_time;

  //     if (remine_time - (cur_transport_time * 2 + 1) < 1) {
  //       continue;
  //     }

  //     // float_t load_wight = 2.0 / io_layer.berths[i].loading_speed;
  //     // float_t trans_wight = io_layer.berths[i].transport_time / 1000.0;
  //     // float_t cur_weight = (value * load_wight * trans_wight);

  //     if (value > max_value) {
  //       max_value = static_cast<int>(value);
  //       target_berth_id = i;
  //     }
  //   }

  //   return target_berth_id;
  // };

  /**
   * @brief 获得在港口[berth_id]的机器人数量
   *
   * @param berth_id
   * @return int
   */
  int get_berth_robot_num(const int berth_id, const int robot_id) {
    int num = 0;

    for (auto &robot : io_layer.robots) {
      if (robot.target_berth_id == berth_id) {
        num++;
      }
    }

    return num;
  }
  // /**
  //  * @brief 获得目的地为港口[berth_id]的船只数量
  //  *
  //  * @param berth_id
  //  * @return int
  //  */
  // int get_berth_ship_num(const int berth_id) {
  //   int num = 0;
  //   for (int i = 0; i < SHIP_NUM; i++) {
  //     if (io_layer.ships[i].berth_id == berth_id) {
  //       num++;
  //     }
  //   }
  //   return num;
  // }

  std::pair<Goods, std::vector<Point>>
  find_best_goods_path_from_berth_v1(const int berth_id, const int robot_id,
                                     bool &founded) {

    // 查找到最优的高于平均值的货物
    Goods goods_final_hi = invalid_goods;
    float max_weight_hi = 0.0;
    // 查找到最优的低于平均值的货物
    Goods goods_final_lo = invalid_goods;
    float max_weight_lo = 0.0;

    bool success = false;
    // 遍历所有的货物,找到最有价值的货物
    for (auto &goods : io_layer.map_goods_list) {

      if (goods.second.status != GoodsStatus::Normal) {
        continue;
      };

      // 找不到路径
      if (io_layer.berths_come_from_set[berth_id].find(goods.second.pos) ==
          io_layer.berths_come_from_set[berth_id].end()) {
        continue;
      }

      // 从港口到货物的路径距离
      const auto to_goods_path_cost =
          io_layer.get_cost_from_berth_to_point(berth_id, goods.first);

      if (!to_goods_path_cost.has_value()) {
        continue;
      }

      // 判断是否能在货物消失之前拿到货物， + 5 是为了给 cut path 预留时间容错
      const bool can_fetch_goods = !goods.second.is_disappeared(
          io_layer.cur_cycle + to_goods_path_cost.value() + 5);

      const int max_path_size = 10000;

      if (!can_fetch_goods && to_goods_path_cost > max_path_size) {
        continue;
      }
      // 货物还剩多久消失
      int goods_remin_time = (goods.second.end_cycle - io_layer.cur_cycle);

      if (goods_remin_time < 200) {
        goods_remin_time = 200;
      }

      // 从货物到港口的最短路径
      const auto to_berth_path_cost =
          io_layer.get_minimum_berth_cost(goods.first);

      // TODO: 优化权重计算
      // 都是越小越好

      // /** 策略1**/
      // const float to_goods_one =
      //     to_goods_path_cost.value() / static_cast<float>(max_path_size);

      // const float goods_remin_time_one =
      //     goods_remin_time / static_cast<float>(max_path_size);

      // float cur_weight =
      //     1 / (0.6 * to_goods_one + 0.017 * goods_remin_time_one);
      // /** 策略1**/

      /** 策略2*/
      const float to_goods_one = 6400.0 / (to_goods_path_cost.value() + 1);
      const float to_berth_one =
          to_berth_path_cost.second / static_cast<float>(max_path_size);
      float goods_remin_time_one =
          4200000.0 / (goods_remin_time * goods_remin_time + 1);

      if (to_goods_path_cost.value() > 71) {
        goods_remin_time_one = goods_remin_time_one / 2;
      }

      float cur_weight = to_goods_one + goods_remin_time_one;
      /** 策略2*/

      /** 策略3*/
      // const float to_goods_one = to_goods_path_cost.value();

      // float cur_weight = 1 / (to_goods_one + 1);
      /** 策略3*/

      //**策略4*/
      // float cur_weight = 1.0 / (to_goods_path_cost.value());

      if (io_layer.final_time) {
        cur_weight =
            1.0 / (to_goods_path_cost.value() + to_berth_path_cost.second);
      }

      // 分别找到高于平均值和低于平均值的货物
      if (goods.second.money >= (io_layer.total_goods_avg_money() / 3)) {
        // if (true) {
        if (cur_weight > max_weight_hi) {
          max_weight_hi = cur_weight;
          goods_final_hi = goods.second;
          founded = true;
        }
      } else {
        if (cur_weight > max_weight_lo) {
          max_weight_lo = cur_weight;
          goods_final_lo = goods.second;
          founded = true;
        }
      }
    }

    const auto &goods_final =
        max_weight_hi != -1 ? goods_final_hi : goods_final_lo;

    auto path_tmp = io_layer.get_path_from_berth_to_point(
        berth_id, goods_final.pos, success);

    log_debug("founded: %d, path_size:%d", founded, path_tmp.size());

    return std::make_pair(goods_final, path_tmp);
  }

  void goods_list_cycle() {
    // 将新货物添加到货物列表中
    for (int i = 0; i < io_layer.new_goods_list.size(); i++) {
      io_layer.map_goods_list[io_layer.new_goods_list[i].pos] =
          io_layer.new_goods_list[i];
      log_assert(io_layer.new_goods_list[i].pos != invalid_point,
                 "invalid goods");
    }

    const bool update_goods_info = io_layer.cur_cycle % 10 == 0;
    // const bool update_goods_info = false;

    if (update_goods_info) {
      for (int i = 0; i < io_layer.berths.size(); i++) {
        io_layer.berths[i].clear_goods_info();
        io_layer.berths[i].tmp_baned = false;
      }
    }

    // 删除 goods_list 中已经消失的货物
    for (auto it = io_layer.map_goods_list.begin();
         it != io_layer.map_goods_list.end();) {
      if (it->second.end_cycle < io_layer.cur_cycle &&
          it->second.status != GoodsStatus::Got) {
        it = io_layer.map_goods_list.erase(it);
      } else {

        if (update_goods_info) {
          for (int i = 0; i < io_layer.berths.size(); i++) {
            auto &cur_berth = io_layer.berths[i];
            auto cur_cost = io_layer.get_cost_from_berth_to_point(i, it->first);

            if (!cur_cost.has_value()) {
              continue;
            }
            auto minimum_cost = io_layer.get_minimum_berth_cost_2(it->first);

            if (minimum_cost.first != i) {
              continue;
            }

            if (cur_cost.value() <= 120 &&
                it->second.money >= io_layer.total_goods_avg_money()) {
              cur_berth.near_goods_num++;
              cur_berth.near_goods_value += it->second.money;
              cur_berth.near_goods_distance += cur_cost.value();
            }
          }
        }

        ++it;
      }
    }
    log_info("map_goods_list size:%d", io_layer.map_goods_list.size());

    // if (update_goods_info) {

    //   std::vector<std::pair<int, int>> berths_sort;
    //   for (int i = 0; i < BERTH_NUM; i++) {
    //     berths_sort.emplace_back(i, io_layer.berths[i].goods_num());
    //   }
    //   std::sort(berths_sort.begin(), berths_sort.end(),
    //             [&](const auto &p1, const auto &p2) {
    //               return p1.second < p2.second;
    //             });

    //   for (int i = 0; i < 10; i++) {
    //     io_layer.berths[i].tmp_baned = false;
    //   }

    //   std::for_each_n(berths_sort.begin(), 2, [&](const auto &p) {
    //     log_info("berth[%d] goods_num:%d", p.first, p.second);
    //     io_layer.berths[p.first].tmp_baned = true;
    //   });
    // }

    // for (int i = 0; i < BERTH_NUM; i++) {
    //   io_layer.berths[i].printf_near_goods_info();

    //   // // 动态禁用港口可能还是副作用
    //   if (update_goods_info) {
    //     if (io_layer.cur_cycle > 1000) {
    //       if (io_layer.berths[i].near_goods_num < 6) {

    //         if (!io_layer.berths[i].tmp_baned) {
    //           bool maybe_baned = true;
    //           io_layer.berths[i].tmp_baned = maybe_baned;
    //         }
    //       } else {
    //         io_layer.berths[i].tmp_baned = false;
    //       }
    //     }
    //   }

    //   log_info("berth[%d] is_baned:%d", i, io_layer.berth_is_baned(i));
    // }
  }

  void go_near_berth(Robot &robot) {
    if (!robot.had_goods && !robot.will_goods_in_this_cycle) {
      log_trace("robot[%d] has no goods, no need go_to_berth", robot.id);
      // 机器人没有货物,不需要去靠泊点
      return;
    }

    auto &robot_path = robot.path_list;
    const Point robot_pos = robot.pos;

    if (!robot_path.empty()) {
      robot.next_pos_before_collision_check = robot_path.back();
    }

    if (robot_path.empty()) {
      bool founded = false;
      int berth_id;

      std::vector<int> exclude_berths{};
      // for (int i = 0; i < BERTH_NUM; i++) {
      //   if (get_berth_robot_num(i, robot_id) > 3 && !io_layer.final_time) {
      //     exclude_berths.emplace_back(i);
      //   }
      // }

      auto path = io_layer.get_near_berth_path_exclude(robot_pos, berth_id,
                                                       founded, exclude_berths);

      if (founded) {
        robot_path = path;
        // berths_id_list[robot_id] = berth_id;
        robot.target_berth_id = berth_id;
        robot.next_pos_before_collision_check = robot_path.back();
      } else {
        // 一定可以找到路径, 如果找不到路径,则机器人被困住了
        log_trace("robot[%d] (%d,%d) not found path to berth, is dead!",
                  robot.id, P_ARG(robot_pos));
        // robots_is_dead[robot_id] = true;
      }
    }
  };

  void find_new_goods(Robot &robot) {

    // 机器人位置位置信息
    auto &cur_robot_target_goods = robot.target_goods;
    const Point &robot_pos = robot.pos;
    const Point &robot_next_pos = robot.next_pos_before_collision_check;

    if (cur_robot_target_goods.status == GoodsStatus::Got) {
      // 机器人已经拿到货物,应该去卸货
      return;
    }

    // if (io_layer.cur_cycle == 1) {
    //   bool cycle1_founded = false;
    //   int cycle1_near_berth_id;
    //   auto near_path = io_layer.get_near_berth_path_exclude(
    //       robot_pos, cycle1_near_berth_id, cycle1_founded,
    //       cycle1_berths_visit);

    //   if (cycle1_founded) {
    //     // robots_path_list[robot_id] = Tools::last_n(near_path, 15);
    //     // robots_next_pos_list[robot_id] = near_path.back();
    //     // io_layer.robots[robot_id].target_berth_id = cycle1_near_berth_id;
    //     // cycle1_berths_visit.emplace_back(cycle1_near_berth_id);
    //     // robots_idle_cycle[robot_id] = near_path.size();

    //     robot.path_list = Tools::last_n(near_path, 15);
    //     robot.next_pos_before_collision_check = near_path.back();
    //     robot.target_berth_id = cycle1_near_berth_id;
    //     robot.idle_cycle = near_path.size();

    //   } else {
    //     // 一定可以找到路径, 如果找不到路径,则机器人被困住了
    //     log_trace("robot[%d] (%d,%d) not found path to berth, is dead!",
    //               robot.id, P_ARG(robot_pos));
    //     // robots_is_dead[robot_id] = true;
    //   }
    // }

    if (robot.had_goods) {
      // 机器人已经拿到货物,应该去卸货
      log_trace("robot[%d] got goods, no need find_new_goods", robot.id);
      log_trace("cur_robot_target_goods (%d,%d) status:%d, cur_cycle:%d, "
                "end_cycle:%d",
                P_ARG(cur_robot_target_goods.pos),
                cur_robot_target_goods.status, io_layer.cur_cycle,
                cur_robot_target_goods.end_cycle);

      // log_assert(cur_robot_target_goods.status == GoodsStatus::Got,
      //            "error, robot should had goods (%d,%d),but status is
      //            %d,", P_ARG(robot_pos),
      //            cur_robot_target_goods.status);
      return;
    }
    if (cur_robot_target_goods.status == GoodsStatus::Booked &&
        !cur_robot_target_goods.is_disappeared(io_layer.cur_cycle)) {
      // 货物没有消失
      log_assert(!robot.path_list.empty(),
                 "error, robot[%d] should had path to goods (%d,%d),but path "
                 "is empty",
                 robot.id, P_ARG(cur_robot_target_goods.pos));
      robot.next_pos_before_collision_check = robot.path_list.back();
      // 机器人已经有预定的货物,并且货物没有消失，不需要再次寻找

      return;
    }

    if (cur_robot_target_goods.status == GoodsStatus::Dead) {
      if (robot.idle_cycle > 0) {
        log_trace("robot[%d] idle_cycle:%d, no need find_new_goods", robot.id,
                  robot.idle_cycle);
        robot.idle_cycle--;

        if (!robot.path_list.empty()) {
          // 还有随机路径时,不需要再次寻找
          robot.next_pos_before_collision_check = robot.path_list.back();
        }
      }
    }

    // 打表法
    // 已经运送货物到港口区域,在港口区域寻找下一个货物
    auto can_search = io_layer.in_berth_search_area(robot_pos);
    if (can_search.has_value()) {

      // 将机器人的目标港口设置为当前选择的港口
      robot.target_berth_id = can_search.value();

      log_debug("robot[%d] in_berth_search_area", robot.id);
      const int search_berth_id = can_search.value();
      log_assert(search_berth_id >= 0 &&
                     search_berth_id < io_layer.berths.size(),
                 "error, search_berth_id is invalid");
      bool search_founded = false;

      auto search_result = find_best_goods_path_from_berth_v1(
          search_berth_id, robot.id, search_founded);

      if (search_founded) {
        const auto searched_goods = search_result.first;
        auto searched_path = search_result.second;
        log_assert(!searched_path.empty(), "error, searched_path is empty");

        bool cut_success = false;
        auto search_come_from = PATHHelper::cut_path(
            robot_pos, get_is_barrier_lambda(), get_find_neighbor_lambda(),
            searched_path, 20, cut_success);

        if (cut_success) {
          log_trace(
              "robot[%d] in_berth_search_area success, find goods(%d,%d), "
              "path size %d ",
              robot.id, P_ARG(searched_goods.pos), searched_path.size());

          log_assert(!searched_path.empty(),
                     "error, search_come_from is "
                     "empty, robot_pos(%d,%d), "
                     "searched_path size:%d",
                     P_ARG(robot_pos), searched_path.size());

          // 更新机器人的路径,下一步位置,货物信息

          robot.path_list = searched_path;
          robot.next_pos_before_collision_check = searched_path.back();
          robot.target_goods = searched_goods;
          robot.target_goods.status = GoodsStatus::Booked;
          io_layer.map_goods_list.at(searched_goods.pos).status =
              GoodsStatus::Booked;

          log_trace("robot[%d] in_berth_search_area success, find "
                    "goods(%d,%d), "
                    "path size %d ",
                    robot.id, P_ARG(searched_goods.pos), searched_path.size());

          return;
        } else {
          log_trace("robot[%d] in_berth_search_area failed, cut path failed",
                    robot.id);
        }
      }
    }

    // 不在港口区域或者找不到货物,选择去港口区域

    log_trace("robot[%d] find_new_goods start from any postion", robot.id);

    const auto goods_set = Tools::map_to_set(io_layer.map_goods_list);

    auto goods_goal_func = [&](Point p) {
      if (goods_set.find(p) != goods_set.end()) {
        auto goods_tmp = io_layer.map_goods_list.at(p);
        if (goods_tmp.status != GoodsStatus::Normal) {
          return false;
        }
        if (goods_tmp.money < io_layer.total_goods_avg_money()) {
          return false;
        }
        if (goods_tmp.is_disappeared(io_layer.cur_cycle + 10)) {
          return false;
        }

        return true;
      }
      return false;
    };
    bool goods_founded = false;

    auto goods_test_path = PATHHelper::bfs_path_v1(
        robot.pos, goods_goal_func, get_is_barrier_lambda(),
        get_find_neighbor_lambda(), 30, goods_founded);

    if (goods_founded) {
      const auto &searched_goods =
          io_layer.map_goods_list.at(goods_test_path.front()); // 获取货物信息

      // 更新机器人的路径,下一步位置,货物信息

      robot.path_list = goods_test_path;
      robot.next_pos_before_collision_check = goods_test_path.back();
      robot.target_goods = searched_goods;
      robot.target_goods.status = GoodsStatus::Booked;
      io_layer.map_goods_list.at(searched_goods.pos).status =
          GoodsStatus::Booked;

      return;
    }

    bool near_founded = false;
    int near_berth_id;

    std::vector<int> exclude_berths{};
    // for (int i = 0; i < BERTH_NUM; i++) {
    //   if (get_berth_robot_num(i, robot.id) > 1 && !io_layer.final_time) {
    //     exclude_berths.emplace_back(i);
    //   }
    // }

    std::vector<Point> near_path = io_layer.get_near_berth_path_exclude(
        robot_pos, near_berth_id, near_founded, exclude_berths);

    if (near_founded) {

      robot.target_berth_id = near_berth_id; // 设置机器人的目标港口
      robot.path_list = Tools::last_n(near_path, 10);
      robot.idle_cycle =
          robot.path_list.size(); // 更新机器人的路径时间,时间内不再寻路
      robot.next_pos_before_collision_check =
          near_path.back(); // 更新机器人的下一步位置

    } else {
      // 一定可以找到路径, 如果找不到路径,则机器人被困住了
      log_trace("robot[%d] (%d,%d) not found path to berth, is dead!", robot.id,
                P_ARG(robot_pos));
      // robots_is_dead[robot_id] = true;
    }
  };

  void run_game() {

    for (int zhen = 1; zhen <= 15000; zhen++) {
      io_layer.input_cycle();

      // 更新货物信息
      goods_list_cycle();

      if (zhen == 1) {
        io_layer.robot_lbot(io_layer.robot_shops.front());
      }

      for (auto &robot : io_layer.robots) {
        robot.clear_flags();
      }
      for (auto &robot : io_layer.robots) {
        robot_get_cycle(robot);
        find_new_goods(robot);
        go_near_berth(robot);
      }

      for (auto &robot : io_layer.robots) {
        robots_move(robot);
        robots_pull_cycle(robot);
      }

      io_layer.output_cycle();
      log_info("map_goods_list size:%d", io_layer.map_goods_list.size());
      io_layer.print_goods_info();

      if (abs(io_layer.cur_cycle) == 15000) {
        break;
      }
    }
    io_layer.print_final_info();
  }

  void robots_move(Robot &robot) {
    const auto &next_pos = robot.get_next_pos();

    if (next_pos != invalid_point && next_pos != stop_point) {
      io_layer.robot_move(robot.id, next_pos);
      robot.path_list.pop_back();
    }
    if (next_pos == stop_point) {
      robot.path_list.pop_back();
    }
  }

  // 机器人卸货
  void robots_pull_cycle(Robot &robot) {
    const auto &next_pos = robot.get_next_pos();
    auto &cur_berth = io_layer.berths[robot.target_berth_id];
    const auto &target_goods = robot.target_goods;
    const auto &had_goods = robot.had_goods;
    const int robot_id = robot.id;

    if (target_goods.status == GoodsStatus::Dead) {
      log_assert(!had_goods, "error, robot should not had goods");
      // 没有货物
      return;
    }
    if (!had_goods) {
      return;
    }

    log_assert(target_goods.status == GoodsStatus::Got,
               "error, robot should had goods ,but status is %d,",
               target_goods.status);

    auto berth_id_opt = io_layer.in_berth_search_area(next_pos);
    if (berth_id_opt.has_value() && had_goods) {

      log_trace("robot[%d] pull goods at (%d,%d)", robot_id,
                P_ARG(target_goods.pos));
      log_trace("robot[%d] goods money:%d,cur_cycle:%d end_cycle:%d, status:%d",
                robot_id, target_goods.money, io_layer.cur_cycle,
                target_goods.end_cycle, target_goods.status);

      // 卸货要求
      // 1. 机器人已经到达靠泊点
      // 2. 机器人已经拿到货物
      // 3. 机器人有预定的货物
      io_layer.robot_pull(robot_id);
      io_layer.goted_goods_num++;
      io_layer.goted_goods_money += target_goods.money;

      // 清空状态位
      // 1. 机器人的目标货物
      // 2. 地图上的货物
      // 3. 机器人的路径
      // map_goods_list.at(target_goods.pos) = invalid_goods;

      io_layer.berths[berth_id_opt.value()].add_goods(target_goods,
                                                      io_layer.cur_cycle);
      robot.target_goods = invalid_goods;
      robot.path_list.clear();
    }
  };

  void robot_get_cycle(Robot &robot) {
    const auto &cur_pos = robot.pos;
    auto &cur_berth = io_layer.berths[robot.target_berth_id];
    auto &target_goods = robot.target_goods;
    const auto &had_goods = robot.had_goods;

    if (target_goods.status == GoodsStatus::Dead) {
      // 没有货物
      return;
    }

    if (had_goods) {
      log_assert(target_goods.status == GoodsStatus::Got,
                 "error, robot should had goods (%d,%d),but status is %d,",
                 P_ARG(cur_pos), target_goods.status);
    }

    if (target_goods.status == GoodsStatus::Booked &&
        target_goods.is_disappeared(io_layer.cur_cycle)) {
      // 正在去货物的路上,货物消失了
      log_trace("robot[%d] target_goods is disappeared", robot.id);
      robot.target_goods.status = GoodsStatus::Dead;
      robot.path_list.clear();
      return;
    }
    log_assert(target_goods.status != GoodsStatus::Normal,
               "goods states error");

    if (target_goods.status == GoodsStatus::Booked && !had_goods) {
      if (cur_pos == target_goods.pos) {
        // 装货要求
        // 1. 机器人已经到达货物位置
        // 2. 机器人没有拿到货物
        // 3. 机器人有预定的货物
        io_layer.robot_get(robot.id);
        log_trace("robot[%d] get goods at (%d,%d)", robot.id,
                  P_ARG(target_goods.pos));
        // 更改货物状态
        robot.target_goods.status = GoodsStatus::Got;
        robot.will_goods_in_this_cycle = true;
      }
    }
  }
};
