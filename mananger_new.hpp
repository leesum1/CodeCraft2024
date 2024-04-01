#pragma once

#include "goods.hpp"
#include "io_laye_new.hpp"
#include "log.h"
#include "point.hpp"
#include "robot.hpp"
#include "robot_collision_avoid.hpp"
#include "robot_control.hpp"
#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

class ManagerNew {

public:
  IoLayerNew io_layer;
  RobotCollisionAvoid robot_collision_avoid{&io_layer};
  ManagerNew() = default;
  ~ManagerNew() = default;
  void init_game() { io_layer.init(); }

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

  void run_game() {

    for (int zhen = 1; zhen <= 15000; zhen++) {
      io_layer.input_cycle();

      // 更新货物信息
      goods_list_cycle();

      if (zhen < 10) {
        io_layer.robot_lbot(io_layer.robot_shops.front());
      }

      for (auto &robot : io_layer.robots) {
        robot.clear_flags();
      }
      for (auto &robot : io_layer.robots) {
        RobotControl::robot_get_goods(robot, io_layer);
        RobotControl::find_new_goods(robot, io_layer);
        RobotControl::go_near_berth(robot, io_layer);
      }

      for (auto &robot : io_layer.robots) {
        robot_collision_avoid.collision_avoid_step1(robot.id);
      }

      for (auto &robot : io_layer.robots) {
        RobotControl::robots_move(robot, io_layer);
        RobotControl::robots_pull_cycle(robot, io_layer);
      }
      check_collision();
      io_layer.output_cycle();
      log_info("map_goods_list size:%d", io_layer.map_goods_list.size());
      io_layer.print_goods_info();

      if (abs(io_layer.cur_cycle) == 15000) {
        break;
      }
    }
    io_layer.print_final_info();
  }

  void check_collision() {
    std::unordered_set<Point> points_set;
    bool has_collision = false;
    for (auto &robot : io_layer.robots) {
      const auto &cur_pos = robot.pos;
      const auto &next_pos = robot.get_next_pos();
      if (points_set.find(next_pos) != points_set.end()) {
        log_fatal("collision");
        has_collision = true;
        break;
      } else {
        if (!Point::is_stop_point(next_pos)) {
          points_set.insert(next_pos);
        }
        points_set.insert(cur_pos);
      }
    }
    log_assert(!has_collision, "has_collision");
  }
};
