#pragma once

#include "goods.hpp"
#include "io_laye_new.hpp"
#include "log.h"
#include "point.hpp"
#include "robot.hpp"
#include "tools.hpp"
#include <algorithm>
#include <cstdlib>
#include <utility>

class RobotControl {
  IoLayerNew* io_layer = nullptr;

public:
  struct GoodsInfo {
    int value;
    int avg_value;
    int distance_to_robot;
    int distance_to_berth;
    int remain_time;
  };


  explicit RobotControl(IoLayerNew* io_layer) : io_layer(io_layer) {}
  ~RobotControl() = default;


  /**
   * @brief 获取货物对于当前机器人的价值信息
   * @param goods 货物
   * @param robot_come_from 机器人当前位置的打表
   * @return
   */
  GoodsInfo get_goods_info_for_robot(const Goods& goods, const ComeFromMap& robot_come_from) const {
    GoodsInfo goods_info{};
    goods_info.value = goods.money;
    goods_info.avg_value = std::max(70, io_layer->statistic.avg_total_goods_value());
    goods_info.distance_to_robot = robot_come_from.get_point_cost(goods.pos).value_or(20000);
    goods_info.distance_to_berth = goods.to_near_berth_cost;
    goods_info.remain_time = goods.end_cycle - io_layer->cur_cycle;
    return goods_info;
  }


  /**
   * @brief 寻物策略1: 优先选择价值最高的货物
   * @param goods_info
   * @return int 计算后的比重,越大越好
   */
  static int goods_strategy_value_first(const GoodsInfo& goods_info) {
    if (goods_info.value > goods_info.avg_value) {
      return goods_info.value * 100;
    }
    return goods_strategy_quality_first(goods_info, false);
  }

  /**
   * @brief 寻物策略2: 优先选择距离最近的货物
   * @param goods_info
   * @param only_care_robot_distance 是否只考虑机器人距离
   * @return int 计算后的比重,越大越好
   */
  static int goods_strategy_distance_first(const GoodsInfo& goods_info, const bool only_care_robot_distance) {
    constexpr int max_distance = 2000;
    int w = goods_info.value < 20 ? 2 : 1;
    if (only_care_robot_distance) {
      return max_distance - w * goods_info.distance_to_robot;
    }
    return max_distance - (goods_info.distance_to_robot + goods_info.distance_to_berth) * w;
  }

  /**
   * @brief 寻物策略3: 优先性价比最高的货物
   * @param goods_info
   * @param only_care_robot_distance 是否只考虑机器人距离
   * @return int 计算后的比重,越大越好
   */
  static int goods_strategy_quality_first(const GoodsInfo& goods_info, const bool only_care_robot_distance) {
    int w = 2;
    if (std::abs(goods_info.distance_to_robot - goods_info.distance_to_berth) > 5) {
      w = 1;
    }
    // 最大值200
    if (only_care_robot_distance) {
      return 50 * w * goods_info.value / (goods_info.distance_to_robot + 1);
    }
    return 50 * w * goods_info.value / (goods_info.distance_to_robot + goods_info.distance_to_berth + 1);
  }


  /**
   * @brief 寻物策略4: 优先选择剩余时间最少的货物
   * @param goods_info
   * @param only_care_robot_distance 是否只考虑机器人距离
   * @return int 计算后的比重,越大越好
   */
  static int goods_strategy_remain_time_first(const GoodsInfo& goods_info, const bool only_care_robot_distance) {
    constexpr int time_level = TIME_LEVEL;
    constexpr int level_step = time_level / 6;


    if (goods_info.remain_time > time_level) {
      return goods_strategy_quality_first(goods_info, only_care_robot_distance);
    }

    if (std::abs(goods_info.distance_to_robot - goods_info.distance_to_berth) > 5) {
      return goods_strategy_quality_first(goods_info, only_care_robot_distance);
    }


    // 最大值200
    const double value_sig = Tools::scaled_sigmoid(1, 200, goods_info.avg_value, goods_info.value);

    // 价值太低,不考虑
    // x:55,y:3.186402
    // x:56,y:3.413559
    // x:57,y:3.663997
    // x:58,y:3.940032
    // x:59,y:4.244197
    // x:60,y:4.579256
    // x:61,y:4.948221
    // x:62,y:5.354373
    // x:63,y:5.801277
    // x:64,y:6.292802
    // x:65,y:6.833134
    // x:66,y:7.426797
    // x:67,y:8.078667
    // x:68,y:8.793979
    // x:69,y:9.578344
    // x:70,y:10.437749
    // x:71,y:11.378559
    // x:72,y:12.407511
    // x:73,y:13.531698
    // x:74,y:14.758546
    // x:75,y:16.095778
    // x:76,y:17.551367
    // x:77,y:19.133469
    // x:78,y:20.850347
    // x:79,y:22.710267
    // x:80,y:24.721381
    // x:81,y:26.891586
    // x:82,y:29.228362
    // x:83,y:31.738588
    // x:84,y:34.428341
    // x:85,y:37.302679
    // x:86,y:40.365406
    // x:87,y:43.618838
    // x:88,y:47.063568
    // x:89,y:50.698239
    // x:90,y:54.519343
    // x:91,y:58.521049
    // x:92,y:62.695078
    // x:93,y:67.030633
    // x:94,y:71.514395
    // x:95,y:76.130593
    // x:96,y:80.861156
    // x:97,y:85.685939
    // x:98,y:90.583035
    // x:99,y:95.529142
    if (value_sig < goods_info.avg_value) {
      if (goods_info.value < 20) {
        return goods_strategy_quality_first(goods_info, false);
      }
      const int level_idx = goods_info.remain_time / level_step;


      const int rate = 2 * (8 - level_idx);
      log_assert(level_idx < 8, "error, level_idx should less than 8,%d", level_idx);
      return rate * goods_strategy_quality_first(goods_info, false);
    }
    // 最大值为150,确保低价值不会被选中
    const int remine_time_val = (time_level - goods_info.remain_time) / 2;
    // 给一个乘数,确保选择剩余时间少的货物
    return (remine_time_val + static_cast<int>(value_sig)) * (80000 / (goods_info.distance_to_robot));
  }


  /**
   * @brief 寻物策略4: 优先选择剩余时间最少的货物
   * @param goods_info
   * @param only_care_robot_distance 是否只考虑机器人距离
   * @return int 计算后的比重,越大越好
   */
  static int goods_strategy_remain_time_first2(const GoodsInfo& goods_info, const bool only_care_robot_distance) {
    if (goods_info.remain_time > 100) {
      return goods_strategy_quality_first(goods_info, only_care_robot_distance);
    }

    // 距离其他港口近，则其他港口的机器人管
    if (std::abs(goods_info.distance_to_robot - goods_info.distance_to_berth) > 10) {
      return goods_strategy_quality_first(goods_info, only_care_robot_distance);
    }


    // 最大值200
    const double value_sig = Tools::scaled_sigmoid(1, 200, goods_info.avg_value, goods_info.value);

    // 价值太低,不考虑
    // x:55,y:3.186402
    // x:56,y:3.413559
    // x:57,y:3.663997
    // x:58,y:3.940032
    // x:59,y:4.244197
    // x:60,y:4.579256
    // x:61,y:4.948221
    // x:62,y:5.354373
    // x:63,y:5.801277
    // x:64,y:6.292802
    // x:65,y:6.833134
    // x:66,y:7.426797
    // x:67,y:8.078667
    // x:68,y:8.793979
    // x:69,y:9.578344
    // x:70,y:10.437749
    // x:71,y:11.378559
    // x:72,y:12.407511
    // x:73,y:13.531698
    // x:74,y:14.758546
    // x:75,y:16.095778
    // x:76,y:17.551367
    // x:77,y:19.133469
    // x:78,y:20.850347
    // x:79,y:22.710267
    // x:80,y:24.721381
    // x:81,y:26.891586
    // x:82,y:29.228362
    // x:83,y:31.738588
    // x:84,y:34.428341
    // x:85,y:37.302679
    // x:86,y:40.365406
    // x:87,y:43.618838
    // x:88,y:47.063568
    // x:89,y:50.698239
    // x:90,y:54.519343
    // x:91,y:58.521049
    // x:92,y:62.695078
    // x:93,y:67.030633
    // x:94,y:71.514395
    // x:95,y:76.130593
    // x:96,y:80.861156
    // x:97,y:85.685939
    // x:98,y:90.583035
    // x:99,y:95.529142
    if (value_sig < 60) {
      // 90 以下
      return goods_strategy_quality_first(goods_info, true);
    }
    // 最大值为150,确保低价值不会被选中
    const int remine_time_val = 100 - goods_info.remain_time;
    // 给一个乘数,确保选择剩余时间少的货物
    return (remine_time_val + static_cast<int>(value_sig)) * 30000 / (goods_info.distance_to_robot + goods_info.
      distance_to_berth);
  }


  /**
   * @brief 从机器人商店找到最优的货物路径
   *
   * @param robot_id
   * @param robot_shop_pos
   * @param goods_strategy 货物选择策略
   * @param founded
   * @return std::pair<Goods, std::vector<Point>>
   */
  std::pair<Goods, std::vector<Point>>
  find_best_goods_path_from_robot_shop(
    const Point& robot_shop_pos,
    const std::function<int(const GoodsInfo&)>& goods_strategy, bool& founded) const {
    const auto robot_shop = std::find(io_layer->robot_shops.begin(),
                                      io_layer->robot_shops.end(), robot_shop_pos);

    if (robot_shop == io_layer->robot_shops.end()) {
      log_fatal("robot shop(%d,%d) not found", P_ARG(robot_shop_pos));
    }

    const auto robot_shop_index =
      std::distance(io_layer->robot_shops.begin(), robot_shop);
    auto& robot_shop_come_from =
      io_layer->robot_shops_come_from.at(robot_shop_index);

    Goods goods_final_hi = invalid_goods;
    int max_weight_hi = -1;
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


      if (!can_fetch_goods) {
        continue;
      }

      const auto goods_info = get_goods_info_for_robot(goods.second, robot_shop_come_from);
      const int cur_weight = goods_strategy(goods_info);

      if (cur_weight > max_weight_hi) {
        max_weight_hi = cur_weight;
        goods_final_hi = goods.second;
        founded = true;
      }
    }

    const auto& goods_final = goods_final_hi;

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
   * @param goods_strategy 货物选择策略
   * @param founded
   * @return std::pair<Goods, std::vector<Point>>
   */
  std::pair<Goods, std::vector<Point>>
  find_best_goods_path_from_berth(const int berth_id,
                                  const int robot_id, const std::function<int(const GoodsInfo&)>& goods_strategy,
                                  bool& founded) const {
    Goods goods_final_hi = invalid_goods;
    int max_weight_hi = -1;


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


      if (!can_fetch_goods) {
        continue;
      }
      const auto goods_info = get_goods_info_for_robot(goods.second, io_layer->berths_come_from_for_robot[berth_id]);
      const int cur_weight = goods_strategy(goods_info);

      if (cur_weight > max_weight_hi) {
        max_weight_hi = cur_weight;
        goods_final_hi = goods.second;
        founded = true;
      }
    }

    const auto& goods_final = goods_final_hi;


    auto path_tmp =
      io_layer->berths_come_from_for_robot[berth_id].get_path_to_point(
        goods_final.pos, success);

    log_debug("founded: %d, path_size:%d", founded, path_tmp.size());

    return std::make_pair(goods_final, path_tmp);
  }

  /**
   * @brief 控制机器人去最近的港口
   * @param robot
   */
  void go_near_berth(Robot& robot) const {
    if (!robot.can_go_berth()) {
      log_trace("robot[%d] has no goods, no need go_to_berth", robot.id);
      // 机器人没有货物,不需要去靠泊点
      return;
    }


    const Point robot_pos = robot.pos;
    bool robot_target_berth_was_baned = false;
    if (robot.target_berth_id != -1) {
      if (io_layer->berths.at(robot.target_berth_id).max_robot_num == 0) {
        robot_target_berth_was_baned = true;
        log_info("cycle[%d], robot[%d] target berth:%d was baned, should change target berth", io_layer->cur_cycle,
                 robot.id,
                 robot.target_berth_id);
      }
    }

    if (!robot.path_list.empty() && !robot_target_berth_was_baned) {
      robot.update_next_pos();
      return;
    }

    bool founded = false;
    int berth_id;
    std::vector<int> exclude_berths{};
    for (int i = 0; i < io_layer->berths.size(); i++) {
      if (io_layer->get_berth_robot_num(i, robot.id) >= io_layer->berths.at(i).max_robot_num) {
        exclude_berths.emplace_back(i);
      }
    }

    const auto path = io_layer->get_near_berth_path_exclude(robot_pos, berth_id,
                                                            founded, exclude_berths);

    if (founded) {
      log_trace("robot[%d] go_near_berth success, berth_id:%d, path_size:%d", robot.id,
                berth_id, path.size());
      robot.path_list = path;
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
  void robots_pull_cycle(Robot& robot) const {
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
      io_layer->statistic.add_goted_goods(target_goods);
      robot.pull_goods_statistic(target_goods);
      // 更新港口货物信息
      io_layer->berths[berth_id_opt.value()].add_goods(target_goods,
                                                       io_layer->cur_cycle);
      if(robot.goods_num == 2){
        io_layer->statistic.add_goted_goods(robot.second_target_goods);
        robot.pull_goods_statistic(robot.second_target_goods);
        // 更新港口货物信息
        io_layer->berths[berth_id_opt.value()].add_goods(robot.second_target_goods,
                                                       io_layer->cur_cycle);
        robot.second_target_goods = invalid_goods;
      }

      robot.target_goods = invalid_goods;
      robot.path_list.clear();
    }
  };

  void find_second_goods(Robot& robot) {
      const Point& robot_pos = robot.pos;
      const Point& robot_next_pos = robot.next_pos_before_collision_check;
      if(robot.target_goods.status != GoodsStatus::Got){
        // 如果没有拿到第一个物品
        return;
      }
      if (robot.goods_full() || robot.can_go_berth()){
        // 
        return;
      }

    if (robot.second_target_goods.status == GoodsStatus::Booked &&
      !robot.second_target_goods.is_disappeared(io_layer->cur_cycle)) {
      // 机器人已经有预定的货物,并且货物没有消失，不需要再次寻找
      log_info("robot[%d],second size:%d",robot.id, robot.path_list.size());

      log_assert(!robot.path_list.empty(),
                 "error, robot[%d] should had path to goods (%d,%d),but path "
                 "is empty",
                 robot.id, P_ARG(robot.target_goods.pos));
      robot.update_next_pos();
      return;
    }

    // 3. 不在港口区域,使用寻路算法从当前位置寻找货物
    log_trace("robot[%d] find_second_goods start from any postion", robot.id);
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
    for (int i = 0; i < io_layer->berths.size(); i++) {
      if (io_layer->get_berth_robot_num(i, robot.id) >= io_layer->berths.at(i).max_robot_num) {
        exclude_berths.emplace_back(i);
      }
    }

    std::vector<Point> near_path = io_layer->get_near_berth_path_exclude(
      robot_pos, near_berth_id, near_berth_founded, exclude_berths);

    if (near_berth_founded) {
      log_trace("robot[%d] find_new_goods go_near_berth success, berth_id:%d",
                robot.id, near_berth_id);

      robot.target_berth_id = near_berth_id; // 设置机器人的目标港口
      robot.path_list = Tools::last_n(near_path, 10);
      robot.idle_cycle = robot.path_list.size();
      robot.next_pos_before_collision_check =
        near_path.back(); // 更新机器人的下一步位置

      return;
    }
  }


  void find_new_goods(Robot& robot, const std::function<int(const GoodsInfo&)>& goods_strategy) const {
    const Point& robot_pos = robot.pos;
    const Point& robot_next_pos = robot.next_pos_before_collision_check;

    if (robot.goods_num == 1 || (robot.target_goods.status == GoodsStatus::Got)) {
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
    const bool found_goods_path_from_berth =
      select_goods_path_from_berth(robot, goods_strategy);
    log_trace("robot[%d] find_new_goods start from berth, result:%d  ",
              robot.id, found_goods_path_from_berth);

    if (found_goods_path_from_berth) {
      return;
    }
    // 2. 如果机器人位于机器人商店区域,则从机器人商店查表选择货物,并计算路径
    log_trace("robot[%d] find_new_goods from robot shop ", robot.id);
    const bool found_goods_path_from_robot_shop =
      select_goods_path_from_robot_shop(robot, [](const GoodsInfo& goods_info) -> int {
        return goods_strategy_quality_first(goods_info, false);
      });

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
    for (int i = 0; i < io_layer->berths.size(); i++) {
      if (io_layer->get_berth_robot_num(i, robot.id) >= io_layer->berths.at(i).max_robot_num) {
        exclude_berths.emplace_back(i);
      }
    }

    std::vector<Point> near_path = io_layer->get_near_berth_path_exclude(
      robot_pos, near_berth_id, near_berth_founded, exclude_berths);

    if (near_berth_founded) {
      log_trace("robot[%d] find_new_goods go_near_berth success, berth_id:%d",
                robot.id, near_berth_id);

      robot.target_berth_id = near_berth_id; // 设置机器人的目标港口
      robot.path_list = near_path;
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

  void robots_move(Robot& robot) const {
    const auto& next_pos = robot.get_next_pos();

    if (next_pos != invalid_point && next_pos != stop_point) {
      io_layer->robot_move(robot.id, next_pos);
      robot.path_list.pop_back();
    }
    if (next_pos == stop_point) {
      robot.path_list.pop_back();
    }
  }

  void robot_get_goods(Robot& robot) const {
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
        io_layer->map_goods_list.at(target_goods.pos).status = GoodsStatus::Got;
        robot.will_goods_in_this_cycle = true;
      }
    } else if(robot.second_target_goods.status == GoodsStatus::Booked && !robot.goods_full()) {
      if (cur_pos == robot.second_target_goods.pos) {
        // 装货要求
        // 1. 机器人已经到达货物位置
        // 2. 机器人没有拿到货物
        // 3. 机器人有预定的货物
        io_layer->robot_get(robot.id);
        log_trace("robot[%d] get second goods at (%d,%d)", robot.id,
                  P_ARG(target_goods.pos));
        // 更改货物状态
        robot.second_target_goods.status = GoodsStatus::Got;
        io_layer->map_goods_list[cur_pos].status = GoodsStatus::Got;
        robot.will_goods_in_this_cycle = true;
      }
    }
  }

  // 机器人卸货
  void robots_pull_goods(Robot& robot) const {
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
      io_layer->statistic.add_goted_goods(target_goods);

      // 清空状态位
      // 1. 机器人的目标货物
      // 2. 地图上的货物
      // 3. 机器人的路径
      // map_goods_list.at(target_goods.pos) = invalid_goods;

      io_layer->berths[berth_id_opt.value()].add_goods(target_goods,
                                                       io_layer->cur_cycle);

      if(robot.goods_num==2){
        io_layer->statistic.add_goted_goods(robot.second_target_goods);
        io_layer->berths[berth_id_opt.value()].add_goods(robot.second_target_goods,
                                                       io_layer->cur_cycle);
        robot.second_target_goods = invalid_goods;
      }

      robot.target_goods = invalid_goods;
      robot.path_list.clear();
    }
  };

  bool select_goods_path_from_cur_pos(Robot& robot) const {
    log_trace("robot[%d] find_new_goods start from any postion", robot.id);

    const auto start_time = std::chrono::steady_clock::now();
    const auto goods_set = Tools::map_to_set(io_layer->map_goods_list);

    auto goods_goal_func = [&](const Point& p) {
      if (goods_set.find(p) == goods_set.end()) {
        return false;
      }

      auto goods_tmp = io_layer->map_goods_list.at(p);
      if (goods_tmp.status != GoodsStatus::Normal) {
        return false;
      }
      if(goods_tmp.money<70){
        return false;
      }

      if (goods_tmp.is_disappeared(io_layer->cur_cycle)) {
        return false;
      }

      // 找到一个货物 p
      return true;
    };

    bool goods_founded = false;
    const auto goods_path = PATHHelper::bfs_path_v1(
      robot.pos, goods_goal_func, io_layer->get_is_barrier_for_robot_lambda(robot.id, false, true, true),
      io_layer->get_find_neighbor_for_robot_lambda(), 40, goods_founded);

    const auto end_time = std::chrono::steady_clock::now();
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

  bool select_goods_path_from_berth(Robot& robot, const std::function<int(const GoodsInfo&)>& goods_strategy) const {
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
      search_berth_id, robot.id, goods_strategy, goods_founded);

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
      robot_pos, io_layer->get_is_barrier_for_robot_lambda(robot.id, false, true, true),
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

  bool select_goods_path_from_robot_shop(Robot& robot,
                                         const std::function<int(const GoodsInfo&)>& goods_strategy) const {
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
      can_search.value(), goods_strategy, goods_founded);

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
      robot_pos, io_layer->get_is_barrier_for_robot_lambda(robot.id, false, true, true),
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
