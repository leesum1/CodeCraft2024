#pragma once

#include "io_laye_new.hpp"
#include "log.h"
#include "point.hpp"
#include "robot.hpp"
#include <algorithm>
#include <utility>

class RobotControl {
  struct GoodsInfo {
    int value;
    int distance_to_robot;
    int distance_to_berth;
    int remain_time;
  };

  IoLayerNew* io_layer = nullptr;

public:
  explicit RobotControl(IoLayerNew* io_layer) : io_layer(io_layer) {}
  ~RobotControl() = default;


  /**
   * @brief 从机器人商店找到最优的货物路径
   *
   * @param robot_id
   * @param robot_shop_pos
   * @param founded
   * @return std::pair<Goods, std::vector<Point>>
   */
  std::pair<Goods, std::vector<Point>>
  find_best_goods_path_from_robot_shop(
    const Point& robot_shop_pos,
    bool& founded) {
    auto robot_shop = std::find(io_layer->robot_shops.begin(),
                                io_layer->robot_shops.end(), robot_shop_pos);

    if (robot_shop == io_layer->robot_shops.end()) {
      log_fatal("robot shop(%d,%d) not found", P_ARG(robot_shop_pos));
    }

    auto robot_shop_index =
      std::distance(io_layer->robot_shops.begin(), robot_shop);
    auto& robot_shop_come_from =
      io_layer->robot_shops_come_from.at(robot_shop_index);

    // 查找到最优的高于平均值的货物
    Goods goods_final_hi = invalid_goods;
    float max_weight_hi = 0.0;
    // 查找到最优的低于平均值的货物
    Goods goods_final_lo = invalid_goods;
    float max_weight_lo = 0.0;
    bool success = false;

    // 遍历所有的货物,找到最有价值的货物
    for (auto& goods : io_layer->map_goods_list) {
      if (goods.second.status != GoodsStatus::Normal) {
        continue;
      };

      // 找不到路径
      if (!robot_shop_come_from.path_exist(goods.first)) {
        continue;
      }

      // 去货物的路径距离
      const auto to_goods_path_cost =
        robot_shop_come_from.get_point_cost(goods.first);

      if (!to_goods_path_cost.has_value()) {
        continue;
      }

      // 判断是否能在货物消失之前拿到货物， + 5 是为了给 cut path 预留时间容错
      const bool can_fetch_goods = !goods.second.is_disappeared(
        io_layer->cur_cycle + to_goods_path_cost.value() + 5);

      const int max_path_size = 10000;

      if (!can_fetch_goods && to_goods_path_cost > max_path_size) {
        continue;
      }
      // 货物还剩多久消失
      int goods_remin_time =
        std::max(300, goods.second.end_cycle - io_layer->cur_cycle);

      // 从货物到港口的最短路径
      const auto to_berth_path_cost =
        io_layer->get_minimum_berth_cost(goods.first);

      // 使用贪心
      // float cur_weight = static_cast<float>(goods.second.money) /
      //                    (to_goods_path_cost.value() + goods_remin_time);
      float cur_weight = static_cast<float>(goods.second.money);
      // if (cur_weight > max_weight_hi) {
      //   max_weight_hi = cur_weight;
      //   goods_final_hi = goods.second;
      //   founded = true;
      // }

      // 分别找到高于平均值和低于平均值的货物
      if (goods.second.money >= (io_layer->total_goods_avg_money())) {
        if (cur_weight > max_weight_hi) {
          max_weight_hi = cur_weight;
          goods_final_hi = goods.second;
          founded = true;
        }
      }
      else {
        if (cur_weight > max_weight_lo) {
          max_weight_lo = cur_weight;
          goods_final_lo = goods.second;
          founded = true;
        }
      }
    }

    const auto& goods_final =
      max_weight_hi != -1 ? goods_final_hi : goods_final_lo;

    auto path_tmp =
      robot_shop_come_from.get_path_to_point(goods_final.pos, success);

    log_debug("founded: %d, path_size:%d", founded, path_tmp.size());

    return std::make_pair(goods_final, path_tmp);
  }

  /**
   * @brief 从港口找到最优的货物路径
   *
   * @param berth_id
   * @param robot_id
   * @param founded
   * @return std::pair<Goods, std::vector<Point>>
   */
  std::pair<Goods, std::vector<Point>>
  find_best_goods_path_from_berth(const int berth_id,
                                  const int robot_id, bool& founded) {
    // 查找到最优的高于平均值的货物
    Goods goods_final_hi = invalid_goods;
    float max_weight_hi = 0.0;
    // 查找到最优的低于平均值的货物
    Goods goods_final_lo = invalid_goods;
    float max_weight_lo = 0.0;

    bool success = false;
    // 遍历所有的货物,找到最有价值的货物
    for (auto& goods : io_layer->map_goods_list) {
      if (goods.second.status != GoodsStatus::Normal) {
        continue;
      }

      // 找不到路径
      if (!io_layer->berths_come_from_for_robot[berth_id].path_exist(
        goods.first)) {
        continue;
      }

      // // 从港口到货物的路径距离
      const auto to_goods_path_cost =
        io_layer->berths_come_from_for_robot[berth_id].get_point_cost(
          goods.first);

      if (!to_goods_path_cost.has_value()) {
        continue;
      }

      // 判断是否能在货物消失之前拿到货物， + 5 是为了给 cut path 预留时间容错
      const bool can_fetch_goods = !goods.second.is_disappeared(
        io_layer->cur_cycle + to_goods_path_cost.value() + 5);

      const int max_path_size = 10000;

      if (!can_fetch_goods && to_goods_path_cost > max_path_size) {
        continue;
      }
      // 货物还剩多久消失
      int goods_remin_time = (goods.second.end_cycle - io_layer->cur_cycle);

      if (goods_remin_time < 200) {
        goods_remin_time = 200;
      }

      // 从货物到港口的最短路径
      const auto to_berth_path_cost =
        io_layer->get_minimum_berth_cost(goods.first);

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

      // const float to_goods_one = 6400.0 / (to_goods_path_cost.value() + 1);
      // const float to_berth_one =
      //     to_berth_path_cost.second / static_cast<float>(max_path_size);
      // float goods_remin_time_one =
      //     4200000.0 / (goods_remin_time * goods_remin_time + 1);

      // if (to_goods_path_cost.value() > 71) {
      //   goods_remin_time_one = goods_remin_time_one / 2;
      // }

      // float cur_weight = to_goods_one + goods_remin_time_one;

      /** 策略2*/

      /** 策略3*/
      // const float to_goods_one = to_goods_path_cost.value();

      // float cur_weight = 1 / (to_goods_one + 1);
      /** 策略3*/

      //**策略4*/
      // float cur_weight = 1.0 / (to_goods_path_cost.value());

      // if (io_layer->final_time) {
      //   cur_weight =
      //       1.0 / (to_goods_path_cost.value() + to_berth_path_cost.second);
      // }

      float cur_weight =
        static_cast<float>(goods.second.money) / (to_goods_path_cost.value());

      if (cur_weight > max_weight_hi) {
        max_weight_hi = cur_weight;
        goods_final_hi = goods.second;
        founded = true;
      }

      // // 分别找到高于平均值和低于平均值的货物
      // if (goods.second.money >= (io_layer->total_goods_avg_money())) {
      //   // if (true) {
      //   if (cur_weight > max_weight_hi) {
      //     max_weight_hi = cur_weight;
      //     goods_final_hi = goods.second;
      //     founded = true;
      //   }
      // } else {
      //   if (cur_weight > max_weight_lo) {
      //     max_weight_lo = cur_weight;
      //     goods_final_lo = goods.second;
      //     founded = true;
      //   }
      // }
    }

    const auto& goods_final =
      max_weight_hi != -1 ? goods_final_hi : goods_final_lo;

    // auto path_tmp = io_layer->get_path_from_berth_to_point(
    //     berth_id, goods_final.pos, success);

    auto path_tmp =
      io_layer->berths_come_from_for_robot[berth_id].get_path_to_point(
        goods_final.pos, success);

    log_debug("founded: %d, path_size:%d", founded, path_tmp.size());

    return std::make_pair(goods_final, path_tmp);
  }


  void go_near_berth(Robot& robot) {
    if (!robot.had_goods && !robot.will_goods_in_this_cycle) {
      log_trace("robot[%d] has no goods, no need go_to_berth", robot.id);
      // 机器人没有货物,不需要去靠泊点
      return;
    }

    auto& robot_path = robot.path_list;
    const Point robot_pos = robot.pos;

    if (!robot_path.empty()) {
      robot.update_next_pos();
    }

    bool founded = false;
    int berth_id;

    std::vector<int> exclude_berths{};
    // for (int i = 0; i < BERTH_NUM; i++) {
    //   if (get_berth_robot_num(i, robot_id) > 3 && !io_layer->final_time) {
    //     exclude_berths.emplace_back(i);
    //   }
    // }

    auto path = io_layer->get_near_berth_path_exclude(robot_pos, berth_id,
                                                      founded, exclude_berths);

    if (founded) {
      robot_path = path;
      // berths_id_list[robot_id] = berth_id;
      robot.target_berth_id = berth_id;
      robot.update_next_pos();
    }
    else {
      // 一定可以找到路径, 如果找不到路径,则机器人被困住了
      log_trace("robot[%d] (%d,%d) not found path to berth, is dead!", robot.id,
                P_ARG(robot_pos));
    }
  };

  // 机器人卸货
  void robots_pull_cycle(Robot& robot) {
    const auto& next_pos = robot.get_next_pos();
    auto& cur_berth = io_layer->berths[robot.target_berth_id];
    const auto& target_goods = robot.target_goods;
    const auto& had_goods = robot.had_goods;
    const int robot_id = robot.id;

    if (target_goods.status == GoodsStatus::Dead || !had_goods) {
      log_assert(!had_goods, "error, robot should not had goods");
      // 没有货物
      return;
    }

    log_assert(target_goods.status == GoodsStatus::Got,
               "error, robot should had goods ,but status is %d,",
               target_goods.status);

    auto berth_id_opt = io_layer->in_berth_area(next_pos);
    if (berth_id_opt.has_value() && had_goods) {
      log_trace("robot[%d] pull goods at (%d,%d)", robot_id,
                P_ARG(target_goods.pos));
      log_trace("robot[%d] goods money:%d,cur_cycle:%d end_cycle:%d,status: %d",
                robot_id, target_goods.money, io_layer->cur_cycle,
                target_goods.end_cycle, target_goods.status);

      // 卸货要求
      // 1. 机器人已经到达靠泊点(下一个位置)
      // 2. 机器人已经拿到货物
      // 3. 机器人有预定的货物
      io_layer->robot_pull(robot_id);
      io_layer->statistic.goted_goods_list.emplace_back(target_goods);
      robot.pull_goods_statistic(target_goods);
      // 更新港口货物信息
      io_layer->berths[berth_id_opt.value()].add_goods(target_goods,
                                                       io_layer->cur_cycle);
      robot.target_goods = invalid_goods;
      robot.path_list.clear();
    }
  };

  void find_new_goods(Robot& robot) {
    const Point& robot_pos = robot.pos;
    const Point& robot_next_pos = robot.next_pos_before_collision_check;

    if (robot.had_goods || (robot.target_goods.status == GoodsStatus::Got)) {
      // 机器人已经拿到货物,应该去卸货
      log_trace("robot[%d] got goods, no need find_new_goods", robot.id);
      log_trace("cur_robot_target_goods (%d,%d) status:%d, cur_cycle:%d, "
                "end_cycle:%d",
                P_ARG(robot.target_goods.pos), robot.target_goods.status,
                io_layer->cur_cycle, robot.target_goods.end_cycle);

      return;
    }
    if (robot.target_goods.status == GoodsStatus::Booked &&
      !robot.target_goods.is_disappeared(io_layer->cur_cycle)) {
      // 机器人已经有预定的货物,并且货物没有消失，不需要再次寻找
      log_assert(!robot.path_list.empty(),
                 "error, robot[%d] should had path to goods (%d,%d),but path "
                 "is empty",
                 robot.id, P_ARG(robot.target_goods.pos));
      robot.update_next_pos();
      return;
    }

    if (robot.target_goods.status == GoodsStatus::Dead) {
      if (robot.idle_cycle > 0) {
        log_trace("robot[%d] idle_cycle:%d, no need find_new_goods", robot.id,
                  robot.idle_cycle);
        robot.idle_cycle--;

        if (!robot.path_list.empty()) {
          // 还有随机路径时,不需要再次寻找
          robot.update_next_pos();
          return;
        }
      }
    }

    // 1. 如果机器人位于港口区域,则从港口区域查表选择货物,并计算路径
    log_trace("robot[%d] find_new_goods from berth from  ", robot.id);
    bool found_goods_path_from_berth =
      select_goods_path_from_berth(robot);
    log_trace("robot[%d] find_new_goods start from berth, result:%d  ",
              robot.id, found_goods_path_from_berth);

    if (found_goods_path_from_berth) {
      return;
    }
    // 2. 如果机器人位于机器人商店区域,则从机器人商店查表选择货物,并计算路径
    log_trace("robot[%d] find_new_goods from robot shop ", robot.id);
    bool found_goods_path_from_robot_shop =
      select_goods_path_from_robot_shop(robot);
    log_trace("robot[%d] find_new_goods start from robot shop, result:%d  ",
              robot.id, found_goods_path_from_robot_shop);
    if (found_goods_path_from_robot_shop) {
      return;
    }

    // 3. 不在港口区域,使用寻路算法从当前位置寻找货物
    log_trace("robot[%d] find_new_goods start from any postion", robot.id);
    bool found_goods_path_from_cur_pos =
      select_goods_path_from_cur_pos(robot);
    log_trace("robot[%d] find_new_goods start from any postion, result:%d  ",
              robot.id, found_goods_path_from_cur_pos);
    if (found_goods_path_from_cur_pos) {
      return;
    }

    // 4. 机器人没有找到货物,则去靠泊点
    log_trace("robot[%d] can not found goods, go_near_berth", robot.id);
    bool near_berth_founded = false;
    int near_berth_id;

    std::vector<int> exclude_berths{};

    // for (int i = 0; i < BERTH_NUM; i++) {
    //   if (get_berth_robot_num(i, robot.id) > 1 && !io_layer->final_time) {
    //     exclude_berths.emplace_back(i);
    //   }
    // }

    std::vector<Point> near_path = io_layer->get_near_berth_path_exclude(
      robot_pos, near_berth_id, near_berth_founded, exclude_berths);

    if (near_berth_founded) {
      log_trace("robot[%d] find_new_goods go_near_berth success, berth_id:%d",
                robot.id, near_berth_id);

      robot.target_berth_id = near_berth_id; // 设置机器人的目标港口
      robot.path_list = Tools::last_n(near_path, 10);
      robot.idle_cycle =
        robot.path_list.size(); // 更新机器人的路径时间,时间内不再寻路
      robot.next_pos_before_collision_check =
        near_path.back(); // 更新机器人的下一步位置

      return;
    }

    // 一定可以找到路径, 如果找不到路径,则机器人被困住了
    log_trace("robot[%d] (%d,%d) not found path to berth, is dead!", robot.id,
              P_ARG(robot_pos));
    // robots_is_dead[robot_id] = true;
  };

  void robots_move(Robot& robot) {
    const auto& next_pos = robot.get_next_pos();

    if (next_pos != invalid_point && next_pos != stop_point) {
      io_layer->robot_move(robot.id, next_pos);
      robot.path_list.pop_back();
    }
    if (next_pos == stop_point) {
      robot.path_list.pop_back();
    }
  }

  void robot_get_goods(Robot& robot) {
    const auto& cur_pos = robot.pos;
    auto& cur_berth = io_layer->berths[robot.target_berth_id];
    auto& target_goods = robot.target_goods;
    const auto& had_goods = robot.had_goods;

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
      target_goods.is_disappeared(io_layer->cur_cycle)) {
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
        io_layer->robot_get(robot.id);
        log_trace("robot[%d] get goods at (%d,%d)", robot.id,
                  P_ARG(target_goods.pos));
        // 更改货物状态
        robot.target_goods.status = GoodsStatus::Got;
        robot.will_goods_in_this_cycle = true;
      }
    }
  }

  // 机器人卸货
  void robots_pull_goods(Robot& robot) {
    const auto& next_pos = robot.get_next_pos();
    auto& cur_berth = io_layer->berths[robot.target_berth_id];
    const auto& target_goods = robot.target_goods;
    const auto& had_goods = robot.had_goods;
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

    auto berth_id_opt = io_layer->in_berth_area(next_pos);

    if (berth_id_opt.has_value() && had_goods) {
      log_trace("robot[%d] pull goods at (%d,%d)", robot_id,
                P_ARG(target_goods.pos));
      log_trace("robot[%d] goods money:%d,cur_cycle:%d end_cycle:%d, status:%d",
                robot_id, target_goods.money, io_layer->cur_cycle,
                target_goods.end_cycle, target_goods.status);

      // 卸货要求
      // 1. 机器人已经到达靠泊点
      // 2. 机器人已经拿到货物
      // 3. 机器人有预定的货物
      io_layer->robot_pull(robot_id);
      io_layer->statistic.goted_goods_list.emplace_back(target_goods);

      // 清空状态位
      // 1. 机器人的目标货物
      // 2. 地图上的货物
      // 3. 机器人的路径
      // map_goods_list.at(target_goods.pos) = invalid_goods;

      io_layer->berths[berth_id_opt.value()].add_goods(target_goods,
                                                       io_layer->cur_cycle);
      robot.target_goods = invalid_goods;
      robot.path_list.clear();
    }
  };

  bool select_goods_path_from_cur_pos(Robot& robot) {
    log_trace("robot[%d] find_new_goods start from any postion", robot.id);

    auto start_time = std::chrono::steady_clock::now();
    const auto goods_set = Tools::map_to_set(io_layer->map_goods_list);

    auto goods_goal_func = [&](Point p) {
      if (goods_set.find(p) == goods_set.end()) {
        return false;
      }

      auto goods_tmp = io_layer->map_goods_list.at(p);
      if (goods_tmp.status != GoodsStatus::Normal) {
        return false;
      }

      if (goods_tmp.is_disappeared(io_layer->cur_cycle)) {
        return false;
      }

      // 找到一个货物 p
      return true;
    };

    bool goods_founded = false;
    auto goods_path = PATHHelper::bfs_path_v1(
      robot.pos, goods_goal_func, io_layer->get_is_barrier_for_robot_lambda(robot.id, false, false, true),
      io_layer->get_find_neighbor_for_robot_lambda(), 20, goods_founded);

    auto end_time = std::chrono::steady_clock::now();
    log_info("robot[%d] find goods path cost time:%d ms", robot.id,
             std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
               start_time)
             .count());

    if (!goods_founded) {
      return false;
    }

    const auto& searched_goods =
      io_layer->map_goods_list.at(goods_path.front()); // 获取货物信息

    // 更新机器人的路径,下一步位置,货物信息
    robot.book_new_goods(searched_goods, goods_path);
    io_layer->map_goods_list.at(searched_goods.pos).status = GoodsStatus::Booked;

    return true;
  }

  bool select_goods_path_from_berth(Robot& robot) {
    const auto& robot_pos = robot.pos;
    const int robot_id = robot.id;

    // 已经运送货物到港口区域,在港口区域寻找下一个货物
    auto can_search = io_layer->in_berth_area(robot.pos);

    if (!can_search.has_value()) {
      return false;
    }

    // 将机器人的目标港口设置为当前选择的港口
    robot.target_berth_id = can_search.value();

    log_debug("robot[%d] in_berth_search_area", robot_id);
    const int search_berth_id = can_search.value();
    log_assert(search_berth_id >= 0 && search_berth_id < io_layer->berths.size(),
               "error, search_berth_id is invalid");
    bool goods_founded = false;

    auto search_result = find_best_goods_path_from_berth(
      search_berth_id, robot.id, goods_founded);

    if (!goods_founded) {
      log_trace("robot[%d] in_berth_search_area failed, not found goods",
                robot.id);
      return false;
    }
    log_trace("robot[%d] in_berth_search_area success, find goods(%d,%d), "
              "path size %d ",
              robot.id, P_ARG(search_result.first.pos),
              search_result.second.size());

    const auto searched_goods = search_result.first;
    auto searched_path = search_result.second;
    log_assert(!searched_path.empty(), "error, searched_path is empty");

    bool cut_success = false;
    auto search_come_from = PATHHelper::cut_path(
      robot_pos, io_layer->get_is_barrier_for_robot_lambda(robot.id, false, false, true),
      io_layer->get_find_neighbor_for_robot_lambda(), searched_path, 8, cut_success);

    if (!cut_success) {
      log_trace("robot[%d] in_berth_search_area failed, cut path failed",
                robot.id);
      return false;
    }

    log_trace("robot[%d] in_berth_search_area success, find goods(%d,%d), "
              "path size %d ",
              robot.id, P_ARG(searched_goods.pos), searched_path.size());

    log_assert(!searched_path.empty(),
               "error, search_come_from is "
               "empty, robot_pos(%d,%d), "
               "searched_path size:%d",
               P_ARG(robot_pos), searched_path.size());

    // 更新机器人的路径,下一步位置,货物信息
    robot.book_new_goods(searched_goods, searched_path);
    io_layer->map_goods_list.at(searched_goods.pos).status = GoodsStatus::Booked;

    log_trace("robot[%d] in_berth_search_area success, find "
              "goods(%d,%d), "
              "path size %d ",
              robot.id, P_ARG(searched_goods.pos), searched_path.size());

    return true;
  }

  bool select_goods_path_from_robot_shop(Robot& robot) {
    const auto& robot_pos = robot.pos;
    const int robot_id = robot.id;

    // 已经运送货物到港口区域,在港口区域寻找下一个货物
    auto can_search = io_layer->in_robot_shop_area(robot.pos);

    if (!can_search.has_value()) {
      return false;
    }

    log_debug("robot[%d] in_robot_shop_area", robot_id);

    bool goods_founded = false;

    auto search_result = find_best_goods_path_from_robot_shop(
      can_search.value(), goods_founded);

    if (!goods_founded) {
      log_trace("robot[%d] in_robot_shop_area failed, not found goods",
                robot.id);
      return false;
    }
    log_trace("robot[%d] in_robot_shop_area success, find goods(%d,%d), "
              "path size %d ",
              robot.id, P_ARG(search_result.first.pos),
              search_result.second.size());

    const auto searched_goods = search_result.first;
    auto searched_path = search_result.second;
    log_assert(!searched_path.empty(), "error, searched_path is empty");

    bool cut_success = false;
    auto search_come_from = PATHHelper::cut_path(
      robot_pos, io_layer->get_is_barrier_for_robot_lambda(robot.id, false, false, true),
      io_layer->get_find_neighbor_for_robot_lambda(), searched_path, 10, cut_success);

    if (!cut_success) {
      log_trace("robot[%d] in_robot_shop_area failed, cut path failed",
                robot.id);
      return false;
    }

    log_trace("robot[%d] in_robot_shop_area success, find goods(%d,%d), "
              "path size %d ",
              robot.id, P_ARG(searched_goods.pos), searched_path.size());

    log_assert(!searched_path.empty(),
               "error, search_come_from is "
               "empty, robot_pos(%d,%d), "
               "searched_path size:%d",
               P_ARG(robot_pos), searched_path.size());

    // 更新机器人的路径,下一步位置,货物信息
    robot.book_new_goods(searched_goods, searched_path);
    io_layer->map_goods_list.at(searched_goods.pos).status = GoodsStatus::Booked;

    log_trace("robot[%d] in_robot_shop_area success, find "
              "goods(%d,%d), "
              "path size %d ",
              robot.id, P_ARG(searched_goods.pos), searched_path.size());

    return true;
  }
};
