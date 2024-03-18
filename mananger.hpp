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

  std::unordered_map<Point, Goods> map_goods_list;

public:
  IoLayer io_layer;

  Manager() {}
  ~Manager() {}
  void init_game() { io_layer.init(); }
  void run_game();

  void berth_cycle() {
    for (int i = 0; i < BERTH_NUM; i++) {
      io_layer.berths[i].clear_goods_info();
    }
    for (const auto &goods : map_goods_list) {
      if (goods.second.status == GoodsStatus::Normal) {

        for (int i = 0; i < 1; i++) {
          auto &cur_berth = io_layer.berths[i];
          auto cur_cost = io_layer.get_cost_from_berth_to_point(i, goods.first);
          if (!cur_cost.has_value()) {
            continue;
          }
          if (cur_cost.value() < 100) {
            io_layer.berths[i].goods_list.push_back(goods.second);
            cur_berth.near_goods_num++;
            cur_berth.near_goods_value += goods.second.money;
            cur_berth.near_goods_distance += cur_cost.value();
          }
        }
      }
    }

    // for (int i = 0; i < BERTH_NUM; i++) {
    //   io_layer.berths[i].printf_near_goods_info();
    // }
  }

  int calc_goods_weight(const Goods &goods, const int path_size) {
    log_assert(path_size > 0, "error, path_size is 0");
    log_assert(goods.money > 0, "error, goods.money is 0");

    int to_berth_path_size = 999999;
    for (int i = 0; i < BERTH_NUM; i++) {
      if (io_layer.berths_come_from[i].find(goods.pos) !=
          io_layer.berths_come_from[i].end()) {

        float_t trans_wight = io_layer.berths[i].transport_time / 1000.0;
        float_t load_wight = 2.0 / io_layer.berths[i].loading_speed;

        float_t cur_berth_weight =
            io_layer.berths_come_from[i].at(goods.pos).cost * trans_wight *
            load_wight;

        to_berth_path_size =
            std::min(to_berth_path_size, static_cast<int>(cur_berth_weight));
      }
    }

    log_assert(to_berth_path_size != 999999, "error, to_berth_path_size is 0");

    return ((goods.money) * 1024) / (path_size + to_berth_path_size);
  }

  std::pair<Goods, std::vector<Point>>
  find_best_goods_path_from_berth_v1(const int berth_id, bool &founded) {

    Goods goods_final = invalid_goods;
    int max_weight = -1;

    bool success = false;
    // 遍历所有的货物,找到最有价值的货物
    for (auto &goods : map_goods_list) {
      if (goods.second.status == GoodsStatus::Normal) {

        // 找不到路径
        if (io_layer.berths_come_from[berth_id].find(goods.second.pos) ==
            io_layer.berths_come_from[berth_id].end()) {
          continue;
        }

        auto cur_path_cost =
            io_layer.berths_come_from[berth_id].at(goods.second.pos).cost;

        // 判断是否能在货物消失之前拿到货物
        const bool can_fetch_goods = !goods.second.is_disappeared(
            io_layer.cur_cycle + cur_path_cost + 5);

        if (!can_fetch_goods) {
          continue;
        }

        int cur_weight = calc_goods_weight(goods.second, cur_path_cost);
        // int cur_weight = goods.second.money * 1024 / cur_path_cost;
        if (cur_weight > max_weight) {
          max_weight = cur_weight;
          // cur_path = path_tmp;
          goods_final = goods.second;
          founded = true;
        }
      }
    }

    auto path_tmp = io_layer.get_path_from_berth_to_point(
        berth_id, goods_final.pos, success);

    log_debug("founded: %d, path_size:%d", founded, path_tmp.size());

    return std::make_pair(goods_final, path_tmp);
  }

  void ship_cycle(const int ship_id) {
    static std::array<bool, 10> berths_visit;
    auto &cur_ship = io_layer.ships[ship_id];

    // 选择价值最高的泊位
    auto select_best_berth = [&]() {
      int target_berth_id = -1;
      int max_value = -1;
      for (int i = 0; i < BERTH_NUM; i++) {
        // 如果泊位已经被占用,则跳过
        if (berths_visit[i] == true) {
          continue;
        }
        int value = io_layer.berths[i].goods_value();
        float_t load_wight = 2.0 / io_layer.berths[i].loading_speed;
        float_t trans_wight = io_layer.berths[i].transport_time / 1000.0;
        float_t cur_weight = (value * load_wight * trans_wight);

        if (cur_weight > max_value) {
          max_value = static_cast<int>(cur_weight);
          target_berth_id = i;
        }
      }
      log_assert(target_berth_id != -1, "error, target_berth_id is -1");

      return target_berth_id;
    };

    auto select_fit_berth = [&]() {
      int target_berth_id = -1;
      int fit_dis = 999999;
      const int remine_capacity = cur_ship.capacity - cur_ship.cur_capacity;
      for (int i = 0; i < BERTH_NUM; i++) {
        // 如果泊位已经被占用,则跳过
        if (berths_visit[i] == true) {
          continue;
        }
        int cur_dis =
            std::abs(io_layer.berths[i].goods_num() - remine_capacity);
        if (cur_dis < fit_dis) {
          fit_dis = cur_dis;
          target_berth_id = i;
        }
      }
      log_assert(target_berth_id != -1, "error, target_berth_id is -1");
      return target_berth_id;
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

    log_debug("ship[%d] is in berth[%d],remine wait time %d", ship_id,
              cur_ship.berth_id, cur_ship.inst_remine_cycle);

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
      berths_visit[target_berth_id] = true;
      // 3. 出发去该泊位
      io_layer.ship(ship_id, target_berth_id);
      int transport_cycle = io_layer.berths[target_berth_id].transport_time;
      cur_ship.new_inst(transport_cycle);

      log_trace("ship[%d] go to berth[%d], trans_time ", ship_id,
                target_berth_id, transport_cycle);
      return;
    }

    log_info("ship[%d] is in berth[%d]", ship_id, cur_ship.berth_id);
    // 船只已经到达泊位
    log_assert(cur_ship.berth_id != -1, "error, ship berch_id is -1");

    if (cur_ship.full()) {
      // 船只已经满载,应该去虚拟点卸货
      io_layer.go(ship_id);
      int transport_cycle = io_layer.berths[cur_ship.berth_id].transport_time;
      cur_ship.new_inst(transport_cycle);

      // 释放泊位
      berths_visit[cur_ship.berth_id] = false;

      log_info("ship[%d] is full, go", ship_id);
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
      auto &cur_berth = io_layer.berths[cur_ship.berth_id];
      const int cur_transport_time = cur_berth.transport_time;
      const int remine_time = 15000 - (io_layer.cur_cycle + 1);

      log_debug("cur_cycle:%d, remine_time:%d, cur_transport_time:%d",
                io_layer.cur_cycle, remine_time, cur_transport_time);

      if ((remine_time - cur_transport_time) < 1) {
        // 必须出发去虚拟点卸货了
        log_trace(
            "ship[%d] don't have enough time to move to next berth[%d], go "
            "to virtual point,remin_time:%d, cur_transport_time:%d",
            ship_id, cur_ship.berth_id, remine_time, cur_transport_time);
        io_layer.go(ship_id);
        int transport_cycle = io_layer.berths[cur_ship.berth_id].transport_time;
        cur_ship.new_inst(transport_cycle);
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
        if (remine_time > (next_transport_time + 1)) {
          // 足够时间去下一个泊位装货
          // if (cur_ship.good_wait_tolong() && cur_ship.capacity_percent() < 80) {
            cur_ship.goods_wait_cycle = 0;
            log_trace("ship[%d] wait too long in berth[%d], go to "
                      "berth[%d],remin_time:%d, next_transport_time:%d",
                      ship_id, cur_ship.berth_id, new_select_berth_id,
                      remine_time, next_transport_time);
            berths_visit[new_select_berth_id] = true;
            berths_visit[cur_ship.berth_id] = false;
            io_layer.ship(ship_id, new_select_berth_id);
            // 泊位之间的移动时间为 500
            cur_ship.new_inst(500);
            return;
          // }

        } else {

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
      }

      return;
    }
  }

  void goods_list_cycle() {
    // 将新货物添加到货物列表中
    for (int i = 0; i < io_layer.new_goods_num; i++) {
      map_goods_list[io_layer.new_goods_list[i].pos] =
          io_layer.new_goods_list[i];
      log_assert(io_layer.new_goods_list[i].pos != invalid_point,
                 "invalid goods");
    }

    for (int i = 0; i < BERTH_NUM; i++) {
      io_layer.berths[i].clear_goods_info();
    }

    // 删除 goods_list 中已经消失的货物
    for (auto it = map_goods_list.begin(); it != map_goods_list.end();) {
      if (it->second.end_cycle < io_layer.cur_cycle &&
          it->second.status != GoodsStatus::Got) {
        it = map_goods_list.erase(it);
      } else {
        for (int i = 0; i < 10; i++) {
          auto &cur_berth = io_layer.berths[i];
          auto cur_cost = io_layer.get_cost_from_berth_to_point(i, it->first);
          if (!cur_cost.has_value()) {
            continue;
          }
          if (cur_cost.value() < 80) {
            cur_berth.near_goods_num++;
            cur_berth.near_goods_value += it->second.money;
            cur_berth.near_goods_distance += cur_cost.value();
          }
        }
        ++it;
      }
    }
    log_info("map_goods_list size:%d", map_goods_list.size());
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
        auto path = io_layer.get_near_berth_path(robot_pos, berth_id, founded);

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

    // 让机器人循环去靠泊点
    auto go_to_berth = [&](const int robot_id, int &berth_id,
                           std::vector<Point> &robot_path) {
      if (!robots_has_goods[robot_id]) {
        log_trace("robot[%d] has no goods, no need go_to_berth", robot_id);
        // 机器人没有货物,不需要去靠泊点
        return;
      }

      const Point &robot_pos = robots_cur_pos_list[robot_id];
      log_trace("robot[%d] berth_id:%d robot (%d,%d)", robot_id, berth_id,
                P_ARG(robot_pos));

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
          // 一定可以找到路径, 如果找不到路径,则机器人被困住了
          log_trace("robot[%d] (%d,%d) not found path to berth[%d], is dead!",
                    robot_id, P_ARG(robot_pos), berth_id);

          robots_is_dead[robot_id] = true;
        }
      }

      // io_layer.berths[berth_id].in_berth_area(robot_pos)

      if (!robot_path.empty()) {
        robots_next_pos_list.at(robot_id) = robot_path.back();
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
            BFS::cut_path(robot_cur_pos, is_barrier, find_neigh,
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
          std::vector<Point> path_last =
              BFS::get_path(robot_cur_pos, random_point, come_from_t, success2);

          if (path_last.empty()) {
            // 找不到路就停止,可能会死锁? (可以优化吗, 再次寻小路去周围的空地?)
            log_info("robot[%d] new path not found, stop move", robot_id);
            robots_next_pos_list.at(robot_id) = Point();
          } else {

            // 最对去 path_last 的五个点
            // robots_path_list[robot_id] = path_last;

            int new_size = std::min(static_cast<int>(path_last.size()), 5);

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
              map_goods_list.at(target_goods.pos).status = GoodsStatus::Normal;

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

    auto find_new_goods = [&](int robot_id) {
      // 机器人位置位置信息
      auto &cur_robot_target_goods = robots_target_goods_list[robot_id];
      const Point &robot_pos = robots_cur_pos_list[robot_id];
      const Point &robot_next_pos = robots_next_pos_list[robot_id];

      if (robots_target_goods_list[robot_id].status == GoodsStatus::Got) {
        // 机器人已经拿到货物,应该去卸货
        return;
      }

      // if (io_layer.cur_cycle < 20) {
      //   // 前 20 个周期不需要寻找货物
      //   return;
      // }

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
      auto can_search = io_layer.in_berth_search_area(robot_pos);
      if (can_search.has_value()) {
        log_debug("robot[%d] in_berth_search_area", robot_id);
        const int search_berth_id = can_search.value();
        log_assert(search_berth_id >= 0 && search_berth_id < BERTH_NUM,
                   "error, search_berth_id is invalid");
        bool search_founded = false;
        auto search_result =
            find_best_goods_path_from_berth_v1(search_berth_id, search_founded);

        if (search_founded) {
          const auto searched_goods = search_result.first;
          auto searched_path = search_result.second;
          log_assert(!searched_path.empty(), "error, searched_path is empty");

          bool cut_success = false;
          auto search_come_from =
              BFS::cut_path(robot_pos, is_barrier, find_neigh, searched_path,
                            10, cut_success);

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
            map_goods_list.at(searched_goods.pos).status = GoodsStatus::Booked;

            log_trace("robot[%d] in_berth_search_area success, find "
                      "goods(%d,%d), "
                      "path size %d ",
                      robot_id, P_ARG(searched_goods.pos),
                      searched_path.size());

            return;
          }
        }
      }

      search_count++;

      log_trace("robot[%d] find_new_goods start", robot_id);

      bool near_founded = false;
      int near_berth_id;
      auto near_path =
          io_layer.get_near_berth_path(robot_pos, near_berth_id, near_founded);

      if (near_founded) {
        robots_path_list[robot_id] = near_path;
        robots_idle_cycle[robot_id] = near_path.size();
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

        io_layer.berths[berth_id_opt.value()].add_goods(target_goods);
        robots_target_goods_list[robot_id] = invalid_goods;
        robots_path_list[robot_id].clear();
      }
    };

    auto robots_pull_get_cycle = [&](const int robot_id) {
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

      // auto berth_id_opt = io_layer.in_berth_area(cur_pos);

      // if (berth_id_opt.has_value() && had_goods) {

      //   log_trace("robot[%d] pull goods at (%d,%d)", robot_id,
      //             P_ARG(target_goods.pos));
      //   log_trace(
      //       "robot[%d] goods money:%d,cur_cycle:%d end_cycle:%d, status:%d",
      //       robot_id, target_goods.money, io_layer.cur_cycle,
      //       target_goods.end_cycle, target_goods.status);

      //   // 卸货要求
      //   // 1. 机器人已经到达靠泊点
      //   // 2. 机器人已经拿到货物
      //   // 3. 机器人有预定的货物
      //   io_layer.robot_pull(robot_id);
      //   io_layer.goted_goods_num++;
      //   io_layer.goted_goods_money += target_goods.money;

      //   // 清空状态位
      //   // 1. 机器人的目标货物
      //   // 2. 地图上的货物
      //   // 3. 机器人的路径
      //   // map_goods_list.at(target_goods.pos) = invalid_goods;

      //   io_layer.berths[berth_id_opt.value()].add_goods(target_goods);
      //   robots_target_goods_list[robot_id] = invalid_goods;
      //   robots_path_list[robot_id].clear();
      // }

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
        robots_pull_get_cycle(i);
      }

      for (int i = 0; i < ROBOT_NUM; i++) {
        if (robots_is_dead[i]) {
          continue;
        }
        find_new_goods(i);
      }
      for (int i = 0; i < ROBOT_NUM; i++) {
        if (robots_is_dead[i]) {
          continue;
        }
        go_near_berth(i);
      }
      for (int i = 0; i < ROBOT_NUM; i++) {
        if (robots_is_dead[i]) {
          continue;
        }
        check_collision(i);
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
      log_info("map_goods_list size:%d", map_goods_list.size());

      log_info("total_goods_num:%d,total_goods_money:%d,goted_goods_num:%d,"
               "goted_goods_money:%d,selled_goods_num:%d,selled_goods_money:%d",
               io_layer.total_goods_num, io_layer.total_goods_money,
               io_layer.goted_goods_num, io_layer.goted_goods_money,
               io_layer.selled_goods_num, io_layer.selled_goods_money);

      if (abs(io_layer.cur_cycle) == 15000) {
        break;
      }
    }
    io_layer.print_final_info();
  }
};
