#pragma once

#include "berth.hpp"
#include "game_map.hpp"
#include "goods.hpp"
#include "log.h"
#include "path_helper.hpp"
#include "point.hpp"
#include "robot.hpp"
#include "ship.hpp"
#include "tools.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <stdio.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#define SHIP_NUM 5
#define ROBOT_NUM 10
#define BERTH_NUM 10
#define NEW_GOODS_NUM 10

// This is the IoLayer class. It is responsible for all input/output operations.
class IoLayer {

private:
  enum CommandInst { ROBOT_MOVE, ROBOT_GET, ROBOT_PULL, SHIP, GO };

  struct Command {
    CommandInst inst;
    int arg1;
    int arg2;
  };

public:
  GameMap game_map; // 初始地图
  std::unordered_map<Point, Goods> map_goods_list;

  /* 每帧更新的信息*/
  std::array<Berth, BERTH_NUM> berths;             // 靠泊点信息
  std::array<Ship, SHIP_NUM> ships;                // 船只信息
  std::array<Robot, ROBOT_NUM> robots;             // 机器人信息
  std::array<Goods, NEW_GOODS_NUM> new_goods_list; // 新增货物信息
  int new_goods_num = 0;                           // 新增货物数量
  int cur_cycle = 0;                               // 当前周期
  int cur_money = 0;                               // 当前金钱

  bool final_time = false; // 到最后的时间后,机器人之往指定的港口运输货物

  // 港口打表
  std::array<std::unordered_map<Point, PointCost>, 10> berths_come_from;
  std::array<std::unordered_set<Point>, 10> berths_come_from_set;

  // 机器人初始位置打表
  std::array<std::unordered_map<Point, PointCost>, 10>
      robots_first_pos_come_from;

  std::array<bool, ROBOT_NUM> robots_dead;

  int total_goods_num = 0;    // 总货物数量
  int total_goods_money = 0;  // 总货物价值
  int goted_goods_num = 0;    // 已经获取的货物数量
  int goted_goods_money = 0;  // 已经获取的货物价值
  int selled_goods_num = 0;   // 已经卖出的货物数量
  int selled_goods_money = 0; // 已经卖出的货物价值

  /* 每帧的输出指令 */
  std::vector<Command> commands;

  explicit IoLayer() {}
  ~IoLayer(){};

  bool berth_is_baned(const int berth_id) {
    // 到了最后时刻,只能往指定的港口运输货物
    if (final_time) {
      std::vector<int> final_berth;
      // 将还能动的船的目的地加入
      for (int i = 0; i < SHIP_NUM; i++) {
        if (ships[i].berth_id != -1) {
          final_berth.push_back(ships[i].berth_id);
        }
      }
      if (!berths[berth_id].is_baned) {
        bool in_final_berth =
            std::any_of(final_berth.begin(), final_berth.end(),
                        [berth_id](int v) { return v == berth_id; });
        return !in_final_berth;
      }
      return true;
    }

    return (berths[berth_id].is_baned || berths[berth_id].tmp_baned);
  }

  int total_goods_avg_money() {
    if (total_goods_num == 0) {
      return 0;
    }
    return total_goods_money / total_goods_num;
  }
  int goted_goods_avg_money() {
    if (goted_goods_num == 0) {
      return 0;
    }
    return goted_goods_money / goted_goods_num;
  }
  int selled_goods_avg_money() {
    if (selled_goods_num == 0) {
      return 0;
    }
    return selled_goods_money / selled_goods_num;
  }

  /**
   * @brief Represents a pair of integers.
   *        first: berth id
   *        second: cost
   */
  std::pair<int, int> get_minimum_berth_cost(const Point &p) {
    int min_cost = 999999;
    int min_berth_id = 0;
    for (int i = 0; i < BERTH_NUM; i++) {
      if (berth_is_baned(i)) {
        continue;
      }

      auto cur_cost = get_cost_from_berth_to_point(i, p);
      if (cur_cost.has_value()) {
        if (cur_cost.value() < min_cost) {
          min_cost = cur_cost.value();
          min_berth_id = i;
        }
      }
    }
    return {min_berth_id, min_cost};
  }

  std::pair<int, int> get_minimum_berth_cost_2(const Point &p) {
    int min_cost = 999999;
    int min_berth_id = 0;
    for (int i = 0; i < BERTH_NUM; i++) {

      auto cur_cost = get_cost_from_berth_to_point(i, p);
      if (cur_cost.has_value()) {
        if (cur_cost.value() < min_cost) {
          min_cost = cur_cost.value();
          min_berth_id = i;
        }
      }
    }
    return {min_berth_id, min_cost};
  }

  std::optional<int> get_cost_from_berth_to_point(const int berth_id,
                                                  const Point &to) {
    const Point &berth_pos =
        Point(berths[berth_id].pos.x + 1, berths[berth_id].pos.y + 1);

    auto &cur_berth_com_from_set = berths_come_from_set[berth_id];
    auto &cur_berth_com_from = berths_come_from[berth_id];

    // 如何确定查找正确？
    if (cur_berth_com_from_set.find(to) != cur_berth_com_from_set.end()) {
      return cur_berth_com_from[to].cost;
    }
    return std::nullopt;
  }

  std::vector<Point> get_path_from_berth_to_point(const int berth_id,
                                                  const Point &to,
                                                  bool &founded) {
    const Point &berth_pos =
        Point(berths[berth_id].pos.x + 1, berths[berth_id].pos.y + 1);
    auto path = PATHHelper::get_path(berth_pos, to, berths_come_from[berth_id],
                                     founded);
    return path;
  }

  std::vector<Point> get_path_from_point_to_berth(const int berth_id,
                                                  const Point &from,
                                                  bool &founded) {
    if (berth_id < 0 || berth_id >= BERTH_NUM) {
      log_fatal("berth id out of range, expect 0~%d, actual:%d", BERTH_NUM,
                berth_id);
      assert(false);
    }
    // find the path
    const Point &berth_pos =
        Point(berths[berth_id].pos.x + 1, berths[berth_id].pos.y + 1);
    const Point &goal_pos = from;

    auto path = PATHHelper::get_path_reverse(
        berth_pos, goal_pos, berths_come_from[berth_id], founded);

    log_info("berth[%d] (%d,%d) to (%d,%d) size:%d", berth_id, P_ARG(berth_pos),
             P_ARG(goal_pos), path.size());

    return path;
  }

  int minimal_transport_time() {

    int minimal_time = 999999;
    for (int i = 0; i < BERTH_NUM; i++) {
      if (berths[i].transport_time < minimal_time) {
        minimal_time = berths[i].transport_time;
      }
    }
    return minimal_time;
  }

  void print_goods_info() {
    for (int i = 0; i < BERTH_NUM; i++) {
      log_info("cur_goods_num:%d,cur_goods_value:%d", berths[i].goods_num(),
               berths[i].goods_value());
    }
    log_info("total_goods_num:%d,total_goods_money:%d,goted_goods_num:%d,"
             "goted_goods_money:%d,selled_goods_num:%d,selled_goods_money:%d",
             total_goods_num, total_goods_money, goted_goods_num,
             goted_goods_money, selled_goods_num, selled_goods_money);
    log_info("total_goods_avg_money:%d,goted_goods_avg_money:%d,selled_goods_"
             "avg_money:%d",
             total_goods_avg_money(), goted_goods_avg_money(),
             selled_goods_avg_money());
  }

  void print_final_info() {

    print_goods_info();

    for (auto &berth : berths) {
      berth.print();
    }

    fprintf(stderr,
            "total_goods_num:%d,total_goods_money:%d,goted_goods_num:%d,"
            "goted_goods_money:%d,selled_goods_num:%d,selled_goods_money:%d\n",
            total_goods_num, total_goods_money, goted_goods_num,
            goted_goods_money, selled_goods_num, selled_goods_money);
  }

  bool init_game_map() {
    char c_buf[210] = {0};
    for (int i = 0; i < 200; i++) {
      memset(c_buf, 0, sizeof(c_buf));
      scanf("%s", c_buf);
      log_raw("%s,len:%d\n", c_buf, strlen(c_buf));
      game_map.write_line(c_buf, i);
    }
    log_info("Game map initialized");
    game_map.print_map();
    return true;
  }
  void init_berths() {
    for (int i = 0; i < BERTH_NUM; i++) {
      scanf("%d%d%d%d%d", &berths[i].id, &berths[i].pos.x, &berths[i].pos.y,
            &berths[i].transport_time, &berths[i].loading_speed);

      log_info("berth id:%d,(%d,%d),transport_time:%d,loading_speed:%d",
               berths[i].id, berths[i].pos.x, berths[i].pos.y,
               berths[i].transport_time, berths[i].loading_speed);
    }

    // 计算 transport_time 平均数
    int transport_time_sum = 0;
    int loading_speed_sum = 0;
    for (int i = 0; i < BERTH_NUM; i++) {
      transport_time_sum += berths[i].transport_time;
      loading_speed_sum += berths[i].loading_speed;
    }
    float transport_time_avg =
        static_cast<float>(transport_time_sum) / BERTH_NUM;
    float loading_speed_avg = static_cast<float>(loading_speed_sum) / BERTH_NUM;

    for (int i = 0; i < BERTH_NUM; i++) {
      berths[i].avg_berth_transport_time = static_cast<int>(transport_time_avg);
      berths[i].avg_berth_loading_speed = loading_speed_avg;
    }

    log_info("Berths initialized,transport_time_avg:%f,loading_speed_avg:%f",
             transport_time_avg, loading_speed_avg);
  }
  void berths_come_from_init() {
    for (int i = 0; i < BERTH_NUM; i++) {
      const Point &start1 = Point(berths[i].pos.x + 1, berths[i].pos.y + 1);

      auto is_barrier = [&](const Point &p) { return game_map.is_barrier(p); };
      auto neighbors = [&](const Point &p) { return game_map.neighbors(p); };

      berths_come_from[i] =
          PATHHelper::bfs_search(start1, ([&](Point p) { return false; }),
                                 is_barrier, neighbors, 300000);
      berths_come_from_set[i] = Tools::map_to_set(berths_come_from[i]);
    }
    log_info("berths_come_from initialized");
    for (int i = 0; i < BERTH_NUM; i++) {
      log_info("berth[%d] come from map size: %d", i,
               berths_come_from[i].size());
    }
  }

  void robots_first_pos_from_init() {
    for (int i = 0; i < ROBOT_NUM; i++) {
      const Point &start = game_map.robots_first_pos[i];

      auto is_barrier = [&](const Point &p) { return game_map.is_barrier(p); };
      auto neighbors = [&](const Point &p) { return game_map.neighbors(p); };

      robots_first_pos_come_from[i] = PATHHelper::bfs_search(
          start, ([&](Point) { return false; }), is_barrier, neighbors, 300000);
    }
    log_info("robots_first_pos_from_init initialized");

    for (int i = 0; i < ROBOT_NUM; i++) {
      log_info("robots_first_pos_from[%d] come from map size: %d", i,
               robots_first_pos_come_from[i].size());
    }
  }

  std::vector<Point> get_near_goods_path_from_robot_init_pos(const int robot_id,
                                                             bool &founded) {
    const Point &robot_init_pos = game_map.robots_first_pos[robot_id];
    auto &cur_com_from = robots_first_pos_come_from[robot_id];

    Goods goal_goods = invalid_goods;
    int min_cost = 999999;
    for (const auto &goods : map_goods_list) {
      if (goods.second.status == GoodsStatus::Normal) {

        // 找不到路路径
        if (cur_com_from.find(goods.first) == cur_com_from.end()) {
          continue;
        }
        int cur_cost = cur_com_from[goods.first].cost;
        if (cur_cost < min_cost) {
          min_cost = cur_cost;
          goal_goods = goods.second;
        }
      }
    }

    if (goal_goods == invalid_goods) {
      log_info("robot[%d] can't find goods", robot_id);
      founded = false;
      return {};
    }

    auto path = PATHHelper::get_path_reverse(robot_init_pos, goal_goods.pos,
                                             cur_com_from, founded);

    return path;
  }

  /**
   * @brief 获得离机器人最近的港口路径
   *
   * @param from 机器人位置
   * @param berth_id 返回的港口id
   * @param founded  是否找到路径
   * @return std::vector<Point> 返回的路径
   */
  std::vector<Point> get_near_berth_path_v1(const Point &from, int &berth_id,
                                            bool &founded) {
    std::vector<Point> path;
    float final_wight = 444444.f;
    int min_berth_id = 0;
    bool found_path = false;
    // 只要能到达任意一个港口就行
    for (int i = 0; i < BERTH_NUM; i++) {
      if (berth_is_baned(i)) {
        continue;
      }
      Point berth_pos = Point(berths[i].pos.x + 1, berths[i].pos.y + 1);
      auto cur_path = get_path_from_point_to_berth(i, from, found_path);
      auto &cur_berth = berths[i];
      if (found_path) {
        // float berth_wight = cur_berth.calc_berth_wight(from); // 越大越好
        float cur_wight =
            static_cast<float>(cur_path.size() + 1) / 1; // 越小越好

        if (cur_wight < final_wight) {
          final_wight = cur_wight;
          min_berth_id = i;
          path = cur_path;
          founded = true;
        }
      }
    }
    berth_id = min_berth_id;
    log_info("from(%d,%d) to berth[%d] (%d,%d) size:%d", from.x, from.y,
             min_berth_id, berths[min_berth_id].pos.x + 1,
             berths[min_berth_id].pos.y + 1, path.size());
    return path;
  }

  /**
   * @brief 获得最富有的港口(港口周围很多货还没有搬)路径
   *
   * @param from 机器人位置
   * @param berth_id 返回的港口id
   * @param founded  是否找到路径
   * @return std::vector<Point> 返回的路径
   */
  std::vector<Point> get_rich_berth_path(const Point &from, int &berth_id,
                                         bool &founded) {
    std::vector<Point> path;
    float final_wight = -200.f; // 越大越好
    int min_berth_id = 0;
    bool found_path = false;
    // 只要能到达任意一个港口就行
    for (int i = 0; i < BERTH_NUM; i++) {
      if (berth_is_baned(i)) {
        continue;
      }

      Point berth_pos = Point(berths[i].pos.x + 1, berths[i].pos.y + 1);
      auto cur_path = get_path_from_point_to_berth(i, from, found_path);
      auto &cur_berth = berths[i];
      if (found_path) {
        const int near_goods_num = cur_berth.near_goods_num;
        const int near_goods_value = cur_berth.near_goods_value;
        const int near_goods_distance =
            cur_berth.near_goods_distance * 2 + cur_path.size();

        if (near_goods_value < 4000) {
          continue;
        }

        const float money_per_distance =
            static_cast<float>(near_goods_value) / (near_goods_distance + 1);
        const float money_per_goods = static_cast<float>(near_goods_value) /
                                      static_cast<float>(near_goods_num + 1);

        // TODO: 有更好的方法吗? 越大越好
        float cur_wight = near_goods_value / (near_goods_distance + 1.0);

        if (cur_wight > final_wight) {
          final_wight = cur_wight;
          min_berth_id = i;
          path = cur_path;
          founded = true;
        }
      }
    }
    berth_id = min_berth_id;

    log_info("will go to "
             "berth[%d],near_goods_num:%d,near_goods_value:%d,near_goods_"
             "distance:%d",
             min_berth_id, berths[min_berth_id].near_goods_num,
             berths[min_berth_id].near_goods_value,
             berths[min_berth_id].near_goods_distance);

    return path;
  }

  std::vector<Point> get_near_berth_path_exclude(const Point &from,
                                                 int &berth_id, bool &founded,
                                                 std::vector<int> &visit) {
    std::vector<Point> path;
    int min_dis = 999999;
    int min_berth_id = std::rand() % BERTH_NUM;
    bool found_path = false;

    // 只要能到达任意一个港口就行
    for (int i = 0; i < ROBOT_NUM; i++) {
      if (std::any_of(visit.begin(), visit.end(),
                      [i](int v) { return v == i; })) {
        continue;
      }

      Point berth_pos = Point(berths[i].pos.x + 1, berths[i].pos.y + 1);
      auto cur_path = get_path_from_point_to_berth(i, from, found_path);
      if (found_path) {
        if (cur_path.size() < min_dis) {
          min_dis = cur_path.size();
          min_berth_id = i;
          path = cur_path;
          founded = true;
        }
      }
    }

    berth_id = min_berth_id;
    visit.push_back(min_berth_id);
    log_info("from(%d,%d) to berth[%d] (%d,%d) size:%d", from.x, from.y,
             min_berth_id, berths[min_berth_id].pos.x + 1,
             berths[min_berth_id].pos.y + 1, path.size());
    return path;
  }

  std::optional<int> in_berth_area(const Point &p) {
    for (auto &berth : berths) {
      if (berth.in_berth_area(p)) {
        return berth.id;
      }
    }
    return std::nullopt;
  }

  std::optional<int> in_berth_search_area(const Point &p) {

    for (auto &berth : berths) {
      if (berth.in_berth_search_area(p)) {
        return berth.id;
      }
    }
    return std::nullopt;
  }

  void robot_dead_list_init() {
    for (int i = 0; i < ROBOT_NUM; i++) {
      robots_dead[i] = false;
    }
  }

  void init_done() {
    char okk[10];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
  }

  void init_ships() {
    int ship_capacity;
    scanf("%d", &ship_capacity);
    for (int i = 0; i < SHIP_NUM; i++) {
      ships[i].id = i;
      ships[i].capacity = ship_capacity;
      ships[i].cur_capacity = 0;
      ships[i].cur_value = 0;
      ships[i].inst_remine_cycle = 0;
      ships[i].goods_wait_cycle = 0;
      ships[i].berth_wait_cycle = 0;
      ships[i].status = 1;
      ships[i].berth_id = -1;
      log_info("ship[%d] capacity:%d", i, ships[i].capacity);
    }
    log_info("Ships initialized");
  }
  void init() {
    auto start = std::chrono::high_resolution_clock::now();
    init_game_map();
    init_berths();
    game_map.init_robot_pos();
    berths_come_from_init();
    robots_first_pos_from_init();
    init_ships();
    auto end = std::chrono::high_resolution_clock::now();
    log_info("IoLayer init time:%d ms",
             std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                 .count());
    init_done();
  }

  void input_cycle() {
    // 读取当前周期的信息
    scanf("%d%d", &cur_cycle, &cur_money);
    log_info("cycle[%d],money:%d", cur_cycle, cur_money);
    // 读取新增货物信息
    scanf("%d", &new_goods_num);
    for (int i = 0; i < new_goods_num; i++) {
      int x, y, money;
      scanf("%d%d%d", &x, &y, &money);
      new_goods_list[i].pos = Point(x, y);
      new_goods_list[i].money = money;
      new_goods_list[i].end_cycle = cur_cycle + 998;
      new_goods_list[i].status = GoodsStatus::Normal;
      log_trace("new goods[%d]:(%d,%d),money:%d,end_cycle:%d", i, x, y, money,
                new_goods_list[i].end_cycle);

      total_goods_money += money;
    }
    total_goods_num += new_goods_num;

    // 读取机器人信息
    for (int i = 0; i < ROBOT_NUM; i++) {
      int goods, x, y, status;
      scanf("%d%d%d%d", &goods, &x, &y, &status);
      robots[i].had_goods = goods;
      robots[i].pos = Point(x, y);
      robots[i].status = status;
      log_info("robot[%d](%d,%d),goods:%d,status:%d", i, x, y, goods, status);
    }
    // 船只状态
    for (int i = 0; i < SHIP_NUM; i++) {
      scanf("%d%d", &ships[i].status, &ships[i].berth_id);
      log_trace("ship [%d] status:%d,berth_id:%d", i, ships[i].status,
                ships[i].berth_id);
    }

    char okk[10];
    scanf("%s", okk);
  }
  void output_cycle() {
    log_trace("cycle[%d],inst_num:%d", cur_cycle, commands.size());
    for (const auto &command : commands) {
      char fmt_buf[50] = {};
      switch (command.inst) {
      case ROBOT_MOVE:
        std::sprintf(fmt_buf, "move %d %d\n", command.arg1, command.arg2);
        break;
      case ROBOT_GET:
        std::sprintf(fmt_buf, "get %d\n", command.arg1);
        break;
      case ROBOT_PULL:
        std::sprintf(fmt_buf, "pull %d\n", command.arg1);
        break;
      case SHIP:
        std::sprintf(fmt_buf, "ship %d %d\n", command.arg1, command.arg2);
        break;
      case GO:
        std::sprintf(fmt_buf, "go %d\n", command.arg1);
        break;
      default:
        log_fatal("unknown command inst:%d", command.inst);
        break;
      }
      printf("%s", fmt_buf);
      log_trace("command:%s", fmt_buf);
    }
    printf("OK\n");
    fflush(stdout);
    commands.clear();
  }

  // 机器人指令
  void robot_move(const int robot_id, const RobotDrirection direction) {
    commands.push_back({ROBOT_MOVE, robot_id, static_cast<int>(direction)});
  }
  void robot_move(int robot_id, Point to) {
    this->robot_move(robot_id, calc_direction(robots[robot_id].pos, to));
  }

  void robot_get(int robot_id) { commands.push_back({ROBOT_GET, robot_id, 0}); }
  void robot_pull(int robot_id) {
    commands.push_back({ROBOT_PULL, robot_id, 0});
  }

  // 船只指令
  void ship(const int ship_id, const int berth_id) {
    commands.push_back({SHIP, ship_id, berth_id});
  }
  void go(int ship_id) { commands.push_back({GO, ship_id, 0}); }

  bool is_valid_move(const Point &from, const Point &to) {
    if (from.x < 0 || from.x >= 200 || from.y < 0 || from.y >= 200) {
      log_trace("from(%d,%d) out of range", from.x, from.y);
      return false;
    }
    if (to.x < 0 || to.x >= 200 || to.y < 0 || to.y >= 200) {
      log_trace("to(%d,%d) out of range", to.x, to.y);
      return false;
    }
    if (abs(from.x - to.x) + abs(from.y - to.y) != 1) {
      log_trace("from(%d,%d) to(%d,%d) not valid", from.x, from.y, to.x, to.y);
      return false;
    }
    return true;
  }

  RobotDrirection calc_direction(const Point &from, const Point &to) {

    // to 只能是 from 的上下左右，其余情况不合法
    if (abs(from.x - to.x) + abs(from.y - to.y) != 1) {
      log_fatal("from(%d,%d) to(%d,%d) not valid", from.x, from.y, to.x, to.y);
      assert(false);
    }

    if (from.x == to.x) {
      if (from.y > to.y) {
        return RobotDrirection::LEFT;
      } else {
        return RobotDrirection::RIGHT;
      }
    } else {
      if (from.x > to.x) {
        return RobotDrirection::UP;
      } else {
        return RobotDrirection::DOWN;
      }
    }
  }

  //-----------------test----------------------------------

  void test_berths_come_from() {
    // game_map.print_map_with_point();

    for (int i = 100; i < 120; i++) {
      game_map.print_map_line(i);
    }

    for (int i = 0; i < BERTH_NUM; i++) {
      // (117,3) (36, 173);
      // (117,3) to (107,122)
      Point cur = Point(107, 122);
      bool founded = false;
      auto path = get_path_from_point_to_berth(i, cur, founded);
      if (founded) {
        log_info("berth[%d] (%d,%d)path size:%d", i, berths[i].pos.x + 1,
                 berths[i].pos.y + 1, path.size());
        for (const auto &p : path) {
          log_info("(%d,%d)", p.x, p.y);
        }
      } else {
        log_info("berth[%d] path not found", i);
      }
    }
  }
};