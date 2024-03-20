#pragma once

#include "goods.hpp"
#include "io_layer.hpp"
#include "log.h"
#include "point.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>
class Manager {

public:
  IoLayer io_layer;

  Manager() {}
  ~Manager() {}
  void init_game() { io_layer.init(); }
  void run_game();

  void berth_cycle() {
    for (int i = 0; i < BERTH_NUM; i++) {
      io_layer.berths[i].tick(io_layer.cur_cycle);
    }
  }

  /**
   * @brief 获得在港口[berth_id]的机器人数量
   *
   * @param berth_id
   * @return int
   */
  int get_berth_robot_num(const int berth_id) {
    int num = 0;
    for (int i = 0; i < ROBOT_NUM; i++) {
      if (io_layer.robots[i].target_berth_id == berth_id) {
        num++;
      }
    }
    return num;
  }
  /**
   * @brief 获得目的地为港口[berth_id]的船只数量
   *
   * @param berth_id
   * @return int
   */
  int get_berth_ship_num(const int berth_id) {
    int num = 0;
    for (int i = 0; i < SHIP_NUM; i++) {
      if (io_layer.ships[i].berth_id == berth_id) {
        num++;
      }
    }
    return num;
  }

  std::pair<Goods, std::vector<Point>>
  find_best_goods_path_from_berth_v1(const int berth_id, const int robot_id,
                                     bool &founded) {

    // 查找到最优的高于平均值的货物
    Goods goods_final_hi = invalid_goods;
    float max_weight_hi = 1000000.0;
    // 查找到最优的低于平均值的货物
    Goods goods_final_lo = invalid_goods;
    float max_weight_lo = 1000000.0;

    bool success = false;
    // 遍历所有的货物,找到最有价值的货物
    for (auto &goods : io_layer.map_goods_list) {

      if (goods.second.status != GoodsStatus::Normal) {
        continue;
      };

      // 找不到路径
      if (io_layer.berths_come_from[berth_id].find(goods.second.pos) ==
          io_layer.berths_come_from[berth_id].end()) {
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
      const int goods_remin_time =
          (goods.second.end_cycle - io_layer.cur_cycle);

      // 从货物到港口的最短路径
      const auto to_berth_path_cost =
          io_layer.get_minimum_berth_cost(goods.first);

      // TODO: 优化权重计算
      // 都是越小越好
      const float to_goods_one =
          to_goods_path_cost.value() / static_cast<float>(max_path_size);
      // const float to_berth_one =
      //     to_berth_path_cost.second / static_cast<float>(max_path_size);
      const float goods_remin_time_one =
          goods_remin_time / static_cast<float>(max_path_size);

      float cur_weight = 0.6 * to_goods_one + 0.017 * goods_remin_time_one;

      if (io_layer.final_time) {
        cur_weight = to_goods_one;
      }

      // 分别找到高于平均值和低于平均值的货物
      if (goods.second.money >= (io_layer.total_goods_avg_money() / 2)) {
        // if (true) {
        if (cur_weight < max_weight_hi) {
          max_weight_hi = cur_weight;
          goods_final_hi = goods.second;
          founded = true;
        }
      } else {
        if (cur_weight < max_weight_lo) {
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

  void ship_cycle(const int ship_id) {
    static std::array<bool, 10> berths_visit;
    auto &cur_ship = io_layer.ships[ship_id];
    auto &cur_berth = io_layer.berths[cur_ship.berth_id];

    auto go_to_virtual_point = [&]() {
      io_layer.go(ship_id);
      const int transport_cycle = cur_berth.transport_time;
      berths_visit[cur_ship.berth_id] = false; // 释放泊位
      cur_ship.new_inst(transport_cycle);
      cur_ship.has_change_berth = false;
      cur_ship.berth_wait_cycle = 0;
      cur_ship.goods_wait_cycle = 0;
      cur_ship.spend_cycle = 0;
      log_trace("ship[%d] go to virtual point, transport time %d", ship_id,
                transport_cycle);
    };

    // 选择价值最高的泊位
    auto select_best_berth = [&]() {
      int target_berth_id = -1;
      int max_value = -1;

      const int remine_time = 15000 - (io_layer.cur_cycle);
      for (int i = 0; i < BERTH_NUM; i++) {
        // 如果泊位已经被占用,则跳过
        if (berths_visit[i] == true || io_layer.berths[i].is_baned) {
          continue;
        }
        // 选择去价值最高的港口就是最优解
        const int value = io_layer.berths[i].goods_value() +
                          io_layer.berths[i].money_in_1000cycle();
        const int cur_transport_time = io_layer.berths[i].transport_time;

        if (remine_time - (cur_transport_time * 2 + 1) < 1) {
          continue;
        }

        // float_t load_wight = 2.0 / io_layer.berths[i].loading_speed;
        // float_t trans_wight = io_layer.berths[i].transport_time / 1000.0;
        // float_t cur_weight = (value * load_wight * trans_wight);

        if (value > max_value) {
          max_value = static_cast<int>(value);
          target_berth_id = i;
        }
      }

      return target_berth_id;
    };

    auto select_fit_berth = [&]() {
      int target_berth_id_hi = -1;
      int target_berth_id_lo = -1;
      int fit_dis_hi = 999999;
      int fit_dis_lo = -200;

      const int remine_capacity = cur_ship.capacity - cur_ship.cur_capacity;
      for (int i = 0; i < BERTH_NUM; i++) {

        log_debug("leesum berth[%d] goods_num:%d", i,
                  io_layer.berths[i].goods_num());
        // 如果泊位已经被占用,或者被 ban 了,则跳过
        if (berths_visit[i] == true || io_layer.berths[i].is_baned) {
          continue;
        }
        int cur_dis = io_layer.berths[i].goods_num() - remine_capacity;

        if (cur_dis < 0) {
          target_berth_id_lo = cur_dis > fit_dis_lo ? i : target_berth_id_lo;
          fit_dis_lo = std::max(fit_dis_lo, cur_dis);
        } else {
          target_berth_id_hi = cur_dis < fit_dis_hi ? i : target_berth_id_hi;
          fit_dis_hi = std::min(fit_dis_hi, cur_dis);
        }
        log_debug("leesum target_berth_id_hi:%d, target_berth_id_lo:%d, cur "
                  "dis %d                ",
                  target_berth_id_hi, target_berth_id_lo, cur_dis);
      }
      int target_berth_id = -1;

      if (std::abs(fit_dis_lo) <= 4) {
        target_berth_id = target_berth_id_lo;
      } else {
        target_berth_id =
            target_berth_id_hi == -1 ? target_berth_id_lo : target_berth_id_hi;
      }

      log_assert(target_berth_id != -1, "error, target_berth_id is -1");
      return target_berth_id;
    };

    auto move_to_next_berth = [&](const int next_berth_id) {
      cur_ship.has_change_berth = true;
      cur_ship.goods_wait_cycle = 0;

      berths_visit[next_berth_id] = true;
      berths_visit[cur_ship.berth_id] = false;
      io_layer.ship(ship_id, next_berth_id);
      // 泊位之间的移动时间为 500
      cur_ship.new_inst(500);
      cur_ship.spend_cycle += 500;
    };

    if (cur_ship.status == 0) {
      // 船只移动中
      log_trace("ship[%d] is moving to %d, remine time %d", ship_id,
                cur_ship.berth_id, cur_ship.inst_remine_cycle);
      cur_ship.inst_remine_cycle--;
      return;
    }

    if (cur_ship.status == 2) {
      // 在泊位外等待
      cur_ship.berth_wait_cycle++;
      log_trace("ship[%d] is waiting berth[%d], wait cycle %d", ship_id,
                cur_ship.berth_id, cur_ship.berth_wait_cycle);
      log_assert(cur_ship.berth_wait_cycle < 100, "error, ship wait too long");
      return;
    }
    // 重置等待周期
    cur_ship.berth_wait_cycle = 0;

    // 船只已经到达虚拟点或者泊位
    log_assert(cur_ship.status == 1, "error, ship status error, status:%d",
               cur_ship.status);

    cur_ship.inst_remine_cycle = 0;

    if (cur_ship.berth_id == -1) {
      // 船只已经到达虚拟点
      // 1. 卸货
      if (io_layer.cur_cycle != 1) {
        // 船只的出生点在虚拟点,第一周期不需要卸货
        io_layer.selled_goods_num += cur_ship.cur_capacity;
        io_layer.selled_goods_money += cur_ship.cur_value;
        cur_ship.unload();
      }

      // 2. 选择一个价值最高的泊位,并且标记为已经占用
      int target_berth_id = select_best_berth();
      const int remine_time = 15000 - (io_layer.cur_cycle);

      if (target_berth_id == -1) {
        log_trace("ship[%d] don't have enough time to move to berth[%d], "
                  "remin_time:%d, transport_cycle:%d",
                  ship_id, target_berth_id, remine_time);
        return;
      }

      const int transport_cycle =
          io_layer.berths[target_berth_id].transport_time;

      berths_visit[target_berth_id] = true;
      // 3. 出发去该泊位
      io_layer.ship(ship_id, target_berth_id);
      cur_ship.new_inst(transport_cycle);
      cur_ship.spend_cycle = transport_cycle;

      log_trace(
          "ship[%d] will go to berth[%d] from virtual point , trans_time ",
          ship_id, target_berth_id, transport_cycle);
      return;
    }

    log_info("ship[%d] is in berth[%d]", ship_id, cur_ship.berth_id);
    // 船只已经到达泊位
    log_assert(cur_ship.berth_id != -1, "error, ship berch_id is -1");

    if (cur_ship.full()) {
      // 船只已经满载,应该去虚拟点卸货
      go_to_virtual_point();
      log_info("ship[%d] is full at berth[%d], cur capacity:%d, cur money:%d "
               "will go to virtual point trans time:%d",
               ship_id, cur_ship.berth_id, cur_ship.cur_capacity,
               cur_ship.cur_value, cur_berth.transport_time);
      return;
    }
    if (!cur_ship.full()) {
      // 船只没有满
      // 1. 剩余时间不足以虚拟点卸货,停留在当前泊位(无论当前泊位是否有货物)
      // 2. 当前泊位有货物,继续装货
      // 3.
      // 当前泊位没有货物,并且剩余时间无法去下一个泊位装货,并去虚拟点卸货,停留在当前泊位
      // 4.
      // 当前泊位没有货物,并且剩余时间可以去下一个泊位装货,并且去虚拟点卸货,移动到下一个泊位
      const int cur_transport_time = cur_berth.transport_time;
      const int remine_time = 15000 - (io_layer.cur_cycle);

      if ((remine_time - cur_transport_time) < 1) {
        // 必须出发去虚拟点卸货了
        log_trace(
            "ship[%d] don't have enough time to move to next berth[%d], go "
            "to virtual point,remin_time:%d, cur_transport_time:%d,cur "
            "capacity:%d, cur money:%d",
            ship_id, cur_ship.berth_id, remine_time, cur_transport_time,
            cur_ship.cur_capacity, cur_ship.cur_value);

        go_to_virtual_point();
        return;
      }

      if (cur_berth.is_empty()) {

        cur_ship.goods_wait_cycle++;

        // 当前泊位已经没有货物
        // 1. 选择一个最适合的泊位，货物最接近满载
        int new_select_berth_id = select_fit_berth();
        // 2. 计算去下一个泊位的装货然后去虚拟点卸货的时间
        const int next_transport_time =
            io_layer.berths[new_select_berth_id].transport_time + 500;
        // 3. 如果剩余时间不足以去下一个泊位装货,停留在当前泊位
        if (remine_time > (next_transport_time + 50)) {
          // if (!cur_ship.has_change_berth) {

          auto &next_berth = io_layer.berths[new_select_berth_id];
          const int remine_capacity = cur_ship.capacity - cur_ship.cur_capacity;
          const int next_berth_money =
              next_berth.get_goods_value_sum_n(remine_capacity);

          const float cur_berth_weight =
              static_cast<float>(cur_ship.cur_value) /
              (cur_transport_time + cur_ship.spend_cycle + 1);
          const float next_berth_weight =
              static_cast<float>(next_berth_money + cur_ship.cur_value) /
              (cur_ship.spend_cycle + next_transport_time + 500);
          if (next_berth_weight > cur_berth_weight) {
            cur_ship.has_change_berth = true;
            cur_ship.goods_wait_cycle = 0;
            log_trace("ship[%d] wait too long in berth[%d], go to "
                      "berth[%d],remin_time:%d, next_transport_time:%d",
                      ship_id, cur_ship.berth_id, new_select_berth_id,
                      remine_time, next_transport_time);

            move_to_next_berth(new_select_berth_id);
            // }
          } else {
            go_to_virtual_point();
          }
        } else {
          io_layer.final_time = true;

          // 剩余时间不足以去下一个泊位装货,停留在当前泊位
          log_trace("ship[%d] wait too long in berth[%d], but no time to move "
                    "to next berth, remine_time:%d, next_transport_time:%d",
                    ship_id, cur_ship.berth_id, remine_time,
                    next_transport_time);
          return;
        }

      } else {
        // 当前泊位还有货物,继续装货
        cur_ship.goods_wait_cycle = 0;

        int load_size =
            std::min(cur_berth.loading_speed, cur_berth.goods_num());
        log_assert(load_size > 0, "error, load_size is 0");

        while (load_size > 0 && !cur_ship.full() && !cur_berth.is_empty()) {
          auto goods = cur_berth.get_goods();
          cur_ship.load(goods.money);
          log_trace("ship[%d] load goods  money:%d", ship_id, goods.money);
          load_size--;
        }
        // 同一帧是否可以出发去卸货, 不可以
      }
      return;
    }
  }

  void goods_list_cycle() {
    // 将新货物添加到货物列表中
    for (int i = 0; i < io_layer.new_goods_num; i++) {
      io_layer.map_goods_list[io_layer.new_goods_list[i].pos] =
          io_layer.new_goods_list[i];
      log_assert(io_layer.new_goods_list[i].pos != invalid_point,
                 "invalid goods");
    }

    for (int i = 0; i < BERTH_NUM; i++) {
      io_layer.berths[i].clear_goods_info();
    }

    // 删除 goods_list 中已经消失的货物
    for (auto it = io_layer.map_goods_list.begin();
         it != io_layer.map_goods_list.end();) {
      if (it->second.end_cycle < io_layer.cur_cycle &&
          it->second.status != GoodsStatus::Got) {
        it = io_layer.map_goods_list.erase(it);
      } else {
        for (int i = 0; i < 10; i++) {
          auto &cur_berth = io_layer.berths[i];
          auto cur_cost = io_layer.get_cost_from_berth_to_point(i, it->first);
          if (!cur_cost.has_value()) {
            continue;
          }
          if (cur_cost.value() <= 80) {
            cur_berth.near_goods_num++;
            cur_berth.near_goods_value += it->second.money;
            cur_berth.near_goods_distance += cur_cost.value();
          }
        }
        ++it;
      }
    }
    log_info("map_goods_list size:%d", io_layer.map_goods_list.size());
    for (int i = 0; i < BERTH_NUM; i++) {
      io_layer.berths[i].printf_near_goods_info();
    }
  }

  void test_berths1() {

    static std::array<std::vector<Point>, ROBOT_NUM> robots_path_list;
    static std::array<Point, ROBOT_NUM> robots_cur_pos_list;
    static std::array<Point, ROBOT_NUM> robots_next_pos_list;
    static std::array<int, ROBOT_NUM> berths_id_list;
    static std::array<bool, ROBOT_NUM> robots_get_action;
    static std::array<bool, ROBOT_NUM> robots_pull_action;
    static std::array<Goods, ROBOT_NUM> robots_target_goods_list;
    static std::array<bool, ROBOT_NUM> robots_has_goods;
    static std::array<bool, ROBOT_NUM> robots_is_dead;
    static std::array<bool, ROBOT_NUM> robots_first_get_goods;
    static std::array<int, ROBOT_NUM> robots_idle_cycle;
    static int search_count = 0;

    robots_is_dead.fill(false);
    robots_get_action.fill(false);
    berths_id_list.fill(-1);

    auto go_near_berth = [&](const int robot_id) {
      if (!robots_has_goods[robot_id] && !robots_first_get_goods[robot_id]) {
        log_trace("robot[%d] has no goods, no need go_to_berth", robot_id);
        // 机器人没有货物,不需要去靠泊点
        return;
      }

      auto &robot_path = robots_path_list[robot_id];
      const Point robot_pos = robots_cur_pos_list[robot_id];

      if (!robot_path.empty()) {
        robots_next_pos_list.at(robot_id) = robot_path.back();
      }

      if (robot_path.empty()) {
        bool founded = false;
        int berth_id;
        auto path =
            io_layer.get_near_berth_path_v1(robot_pos, berth_id, founded);

        if (founded) {
          robot_path = path;
          berths_id_list[robot_id] = berth_id;
          robots_next_pos_list.at(robot_id) = robot_path.back();
        } else {
          // 一定可以找到路径, 如果找不到路径,则机器人被困住了
          log_trace("robot[%d] (%d,%d) not found path to berth, is dead!",
                    robot_id, P_ARG(robot_pos));
          // robots_is_dead[robot_id] = true;
        }
      }
    };

    // ---------------------------------------
    // 障碍物检测
    // 1. 地图障碍物
    // 2. 机器人当前位置
    // 3. 机器人下一个位置
    auto is_barrier = [&](const Point &p) {
      bool is_barrier1 = io_layer.game_map.is_barrier(p);
      bool is_barrier2 =
          std::any_of(robots_cur_pos_list.begin(), robots_cur_pos_list.end(),
                      [&](Point _pos) { return p == _pos; });
      bool is_barrier3 =
          std::any_of(robots_next_pos_list.begin(), robots_next_pos_list.end(),
                      [&](Point _pos) { return p == _pos; });
      return is_barrier1 || is_barrier2 || is_barrier3;
    };

    auto find_neigh = [&](const Point &p) {
      return io_layer.game_map.neighbors(p);
    };

    // 碰撞检测(采用剪切法)
    auto check_collision = [&](int robot_id) {
      Point robot_next_pos = robots_next_pos_list[robot_id];
      Point robot_cur_pos = robots_cur_pos_list[robot_id];
      log_trace("robot[%d] check_collision start", robot_id);
      log_trace("robot[%d] robot_cur_pos(%d,%d) robot_next_pos(%d,%d)",
                robot_id, P_ARG(robot_cur_pos), P_ARG(robot_next_pos));

      if (robot_next_pos == Point()) {
        // 不动就不需要检测
        log_trace("robot[%d] robot_next_pos is null, no need check_collision",
                  robot_id);
        return;
      }

      // 将当前机器人与其他机器人进行碰撞检测
      for (int i = 0; i < ROBOT_NUM; i++) {
        if (i == robot_id) {
          // 不需要检测自己
          continue;
        }

        if (robot_next_pos != robots_cur_pos_list[i] &&
            robot_next_pos != robots_next_pos_list[i]) {
          // 当前机器人的下一步不是其他机器人的位置或下一步
          continue;
        }

        log_info("robot[%d] and robot[%d] collision", robot_id, i);

        bool test_success;
        auto come_from_t =
            PATHHelper::cut_path(robot_cur_pos, is_barrier, find_neigh,
                                 robots_path_list[robot_id], 20, test_success);

        if (test_success) {

          // 更新下一步位置
          robots_next_pos_list.at(robot_id) = robots_path_list[robot_id].back();
        } else {
          // 能不能再利用 BFS 的结果，找到一个路径，需要一个终点
          // 需要另外的策略
          // 1. 找不到路就停止 (在狭窄的地方会死锁)
          // 2. 随便找块空地

          // auto max_point =
          //     std::max_element(come_from_t.begin(), come_from_t.end(),
          //                      [&](const auto &p1, const auto &p2) {
          //                        return p1.second.cost < p2.second.cost;
          //                      });

          std::vector<Point> come_from_values;

          for (const auto &pair : come_from_t) {
            come_from_values.push_back(pair.first);
          }

          std::random_device rd;
          std::shuffle(come_from_values.begin(), come_from_values.end(), rd);

          auto random_point = come_from_values.back();

          bool success2 = false;
          std::vector<Point> path_last = PATHHelper::get_path(
              robot_cur_pos, random_point, come_from_t, success2);

          if (path_last.empty()) {
            // 找不到路就停止,可能会死锁? (可以优化吗,
            // 再次寻小路去周围的空地?)
            log_info("robot[%d] new path not found, stop move", robot_id);
            robots_next_pos_list.at(robot_id) = Point();
          } else {

            // 最对去 path_last 的五个点
            // robots_path_list[robot_id] = path_last;

            int new_size = std::min(static_cast<int>(path_last.size()), 3);

            robots_path_list[robot_id] =
                std::vector<Point>(path_last.end() - new_size, path_last.end());

            // 放弃当前路径，去新的空地避免死锁
            log_trace("robot[%d] give up current path, go to new space(%d,%d) "
                      "size:%d",
                      robot_id, P_ARG(path_last.back()), path_last.size());
            robots_next_pos_list.at(robot_id) = path_last.back();

            // 如果机器人有预定的货物(还没有拿到),则取消预定
            if ((robots_target_goods_list[robot_id].status ==
                 GoodsStatus::Booked) &&
                robots_has_goods[robot_id] == false) {

              const auto &target_goods = robots_target_goods_list[robot_id];
              log_debug("robot[%d] give up current target_good"
                        "(%d,%d) status:%d, money:%d,end_cycle:%d",
                        robot_id, P_ARG(target_goods.pos), target_goods.status,
                        target_goods.money, target_goods.end_cycle);

              // 取消预定状态
              io_layer.map_goods_list.at(target_goods.pos).status =
                  GoodsStatus::Normal;

              robots_target_goods_list[robot_id] = invalid_goods;
            }
          }
        }
      }
      log_debug("robot[%d] check_collision end", robot_id);
    };

    auto err_debug = [&](int robot_id) {
      // if (!io_layer.is_valid_move(robots_cur_pos_list[robot_id],
      //                             robots_next_pos_list[robot_id])) {
      //   log_trace("robot[%d] invalid move, path size", robot_id);

      //   for (auto pos : robots_path_list[robot_id]) {
      //     log_trace("robot[%d] path (%d,%d)", robot_id, pos.x, pos.y);
      //   }
      // }
    };

    std::vector<int> cycle1_berths_visit;

    auto find_new_goods = [&](int robot_id) {
      // 机器人位置位置信息
      auto &cur_robot_target_goods = robots_target_goods_list[robot_id];
      auto &cur_robot = io_layer.robots[robot_id];
      const Point &robot_pos = robots_cur_pos_list[robot_id];
      const Point &robot_next_pos = robots_next_pos_list[robot_id];

      if (robots_target_goods_list[robot_id].status == GoodsStatus::Got) {
        // 机器人已经拿到货物,应该去卸货
        return;
      }

      if (io_layer.cur_cycle < 10) {
        bool cycle1_founded = false;
        int cycle1_near_berth_id;
        auto near_path = io_layer.get_near_berth_path_exclude(
            robot_pos, cycle1_near_berth_id, cycle1_founded,
            cycle1_berths_visit);

        if (cycle1_founded) {
          robots_path_list[robot_id] = near_path;
          robots_idle_cycle[robot_id] =
              std::min(std::rand() % 7, static_cast<int>(near_path.size()));
        } else {
          // 一定可以找到路径, 如果找不到路径,则机器人被困住了
          log_trace("robot[%d] (%d,%d) not found path to berth, is dead!",
                    robot_id, P_ARG(robot_pos));
          // robots_is_dead[robot_id] = true;
        }
      }

      if (robots_has_goods[robot_id]) {
        // 机器人已经拿到货物,应该去卸货
        log_trace("robot[%d] got goods, no need find_new_goods", robot_id);
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
        log_assert(!robots_path_list[robot_id].empty(),
                   "error, robot[%d] should had path to goods (%d,%d),but path "
                   "is empty",
                   robot_id, P_ARG(cur_robot_target_goods.pos));
        robots_next_pos_list[robot_id] = robots_path_list[robot_id].back();
        return;
      }

      if (cur_robot_target_goods.status == GoodsStatus::Booked &&
          !cur_robot_target_goods.is_disappeared(io_layer.cur_cycle)) {
        // 货物没有消失
        log_assert(!robots_path_list[robot_id].empty(),
                   "error, robot should had path to goods (%d,%d),but path "
                   "is empty",
                   P_ARG(cur_robot_target_goods.pos));
        robots_next_pos_list[robot_id] = robots_path_list[robot_id].back();
        // 机器人已经有预定的货物,并且货物没有消失，不需要再次寻找
        return;
      }

      if (cur_robot_target_goods.status == GoodsStatus::Dead) {
        if (robots_idle_cycle[robot_id] > 0) {
          log_trace("robot[%d] idle_cycle:%d, no need find_new_goods", robot_id,
                    robots_idle_cycle[robot_id]);
          robots_idle_cycle[robot_id]--;
          if (!robots_path_list[robot_id].empty()) {
            robots_next_pos_list[robot_id] = robots_path_list[robot_id].back();
            // 还有随机路径时,不需要再次寻找
            return;
          }
        }
      }

      // 打表法
      // 已经运送货物到港口区域,在港口区域寻找下一个货物
      auto can_search = io_layer.in_berth_search_area(robot_pos);
      if (can_search.has_value()) {

        // 将机器人的目标港口设置为当前选择的港口
        cur_robot.target_berth_id = can_search.value();

        log_debug("robot[%d] in_berth_search_area", robot_id);
        const int search_berth_id = can_search.value();
        log_assert(search_berth_id >= 0 && search_berth_id < BERTH_NUM,
                   "error, search_berth_id is invalid");
        bool search_founded = false;

        auto search_result = find_best_goods_path_from_berth_v1(
            search_berth_id, robot_id, search_founded);

        if (search_founded) {
          const auto searched_goods = search_result.first;
          auto searched_path = search_result.second;
          log_assert(!searched_path.empty(), "error, searched_path is empty");

          bool cut_success = false;
          auto search_come_from =
              PATHHelper::cut_path(robot_pos, is_barrier, find_neigh,
                                   searched_path, 20, cut_success);

          if (cut_success) {
            log_assert(!searched_path.empty(),
                       "error, search_come_from is "
                       "empty, robot_pos(%d,%d), "
                       "searched_path size:%d",
                       P_ARG(robot_pos), searched_path.size());

            // 更新机器人的路径,下一步位置,货物信息
            robots_path_list[robot_id] = searched_path;
            robots_next_pos_list[robot_id] = searched_path.back();
            robots_target_goods_list[robot_id] = searched_goods;
            robots_target_goods_list[robot_id].status = GoodsStatus::Booked;
            io_layer.map_goods_list.at(searched_goods.pos).status =
                GoodsStatus::Booked;

            log_trace("robot[%d] in_berth_search_area success, find "
                      "goods(%d,%d), "
                      "path size %d ",
                      robot_id, P_ARG(searched_goods.pos),
                      searched_path.size());

            return;
          }
        }
      }

      // 不在港口区域或者找不到货物,选择去港口区域
      search_count++;

      log_trace("robot[%d] find_new_goods start from any postion", robot_id);

      // bfs_path_v1(const Point &start, const Point &goal,
      //             std::function<bool(Point)> is_barrier,
      //             std::function<std::vector<Point>(Point)> neighbors, int
      //             max_level, bool &founded) {

      auto goods_goal_func = [&](Point p) {
        if (io_layer.map_goods_list.find(p) != io_layer.map_goods_list.end()) {
          if (io_layer.map_goods_list.at(p).status == GoodsStatus::Normal) {
            return true;
          };
        }
        return false;
      };
      bool goods_founded = false;

      auto goods_test_path =
          PATHHelper::bfs_path_v1(cur_robot.pos, goods_goal_func, is_barrier,
                                  find_neigh, 10, goods_founded);

      if (goods_founded) {
        const auto &searched_goods =
            io_layer.map_goods_list.at(goods_test_path.front()); // 获取货物信息

        // 更新机器人的路径,下一步位置,货物信息
        robots_path_list[robot_id] = goods_test_path;
        robots_next_pos_list[robot_id] = goods_test_path.back();
        robots_target_goods_list[robot_id] = searched_goods;
        robots_target_goods_list[robot_id].status = GoodsStatus::Booked;
        io_layer.map_goods_list.at(searched_goods.pos).status =
            GoodsStatus::Booked;

        return;
      }

      bool near_founded = false;
      int near_berth_id;

      std::vector<Point> near_path = io_layer.get_near_berth_path_v1(
          robot_pos, near_berth_id, near_founded);

      if (near_founded) {
        cur_robot.target_berth_id = near_berth_id; // 设置机器人的目标港口
        robots_path_list[robot_id] = near_path; // 更新机器人的路径
        robots_idle_cycle[robot_id] = std::min(
            std::rand() % 7,
            static_cast<int>(
                near_path.size())); // 更新机器人的路径时间,时间内不再寻路
        robots_next_pos_list[robot_id] =
            near_path.back(); // 更新机器人的下一步位置
      } else {
        // 一定可以找到路径, 如果找不到路径,则机器人被困住了
        log_trace("robot[%d] (%d,%d) not found path to berth, is dead!",
                  robot_id, P_ARG(robot_pos));
        // robots_is_dead[robot_id] = true;
      }
    };

    auto robots_pull_cycle = [&](const int robot_id) {
      const auto &next_pos = robots_next_pos_list[robot_id];
      auto &cur_berth = io_layer.berths[berths_id_list[robot_id]];
      const auto &target_goods = robots_target_goods_list[robot_id];
      const auto &had_goods = robots_has_goods[robot_id];

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

      auto berth_id_opt = io_layer.in_berth_area(next_pos);
      if (berth_id_opt.has_value() && had_goods) {

        log_trace("robot[%d] pull goods at (%d,%d)", robot_id,
                  P_ARG(target_goods.pos));
        log_trace(
            "robot[%d] goods money:%d,cur_cycle:%d end_cycle:%d, status:%d",
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
        robots_target_goods_list[robot_id] = invalid_goods;
        robots_path_list[robot_id].clear();
      }
    };

    auto robots_get_cycle = [&](const int robot_id) {
      const auto &cur_pos = robots_cur_pos_list[robot_id];
      auto &cur_berth = io_layer.berths[berths_id_list[robot_id]];
      auto &target_goods = robots_target_goods_list[robot_id];
      const auto &had_goods = robots_has_goods[robot_id];

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
        log_trace("robot[%d] target_goods is disappeared", robot_id);
        robots_target_goods_list[robot_id].status = GoodsStatus::Dead;
        robots_path_list[robot_id].clear();
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
          io_layer.robot_get(robot_id);
          log_trace("robot[%d] get goods at (%d,%d)", robot_id,
                    P_ARG(target_goods.pos));

          // 更改货物状态
          robots_target_goods_list[robot_id].status = GoodsStatus::Got;
          // map_goods_list.at(target_goods.pos).status = GoodsStatus::Got;
          robots_first_get_goods[robot_id] = true;
        }
      }
    };

    // io_layer.berths[0].is_baned = true;
    // io_layer.berths[1].is_baned = true;
    // io_layer.berths[3].is_baned = true;

    for (int zhen = 1; zhen <= 15000; zhen++) {
      io_layer.input_cycle();

      // 更新货物信息
      goods_list_cycle();

      search_count = 0;
      // 获取机器人当前位置
      for (int i = 0; i < 10; i++) {
        robots_cur_pos_list[i] = io_layer.robots[i].pos;
        robots_next_pos_list[i] = invalid_point;
        robots_has_goods[i] = io_layer.robots[i].had_goods;
      }

      for (int i = 0; i < 10; i++) {
        log_trace("robot[%d] is dead:%d", i, robots_is_dead[i]);
      }

      // berth_cycle();

      for (int i = 0; i < SHIP_NUM; i++) {
        ship_cycle(i);
      }

      // for (int i = 0; i < ROBOT_NUM; i++) {
      //   if (robots_is_dead[i]) {
      //     continue;
      //   }
      //   robots_pull_get_cycle(i);
      //   find_new_goods(i);
      //   go_near_berth(i);
      //   err_debug(i);
      //   check_collision(i);
      // }

      robots_first_get_goods.fill(false);

      for (int i = 0; i < ROBOT_NUM; i++) {
        if (robots_is_dead[i]) {
          continue;
        }
        robots_get_cycle(i);
      }

      for (int i = 0; i < ROBOT_NUM; i++) {
        if (robots_is_dead[i]) {
          continue;
        }
        find_new_goods(i);
        go_near_berth(i);
        // check_collision(i);
      }

      for (int i = 0; i < ROBOT_NUM; i++) {
        if (robots_is_dead[i]) {
          continue;
        }
        if (robots_has_goods[i]) {
          check_collision(i);
        }
      }

      for (int i = 0; i < ROBOT_NUM; i++) {
        if (robots_is_dead[i]) {
          continue;
        }
        if (!robots_has_goods[i]) {
          check_collision(i);
        }
      }

      // 输出命令
      for (int j = 0; j < 10; j++) {
        if (robots_is_dead[j]) {
          continue;
        }
        const auto &cur_pos = robots_cur_pos_list[j];
        const auto &next_pos = robots_next_pos_list[j];
        auto &cur_berth = io_layer.berths[berths_id_list[j]];
        auto &target_goods = robots_target_goods_list[j];
        const auto &had_goods = robots_has_goods[j];

        if (next_pos != invalid_point) {
          io_layer.robot_move(j, next_pos);
          robots_path_list[j].pop_back();
        }

        robots_pull_cycle(j);
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
};
