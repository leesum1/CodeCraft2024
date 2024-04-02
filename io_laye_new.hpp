#pragma once

#include "berth.hpp"
#include "come_from_map.hpp"
#include "direction.hpp"
#include "game_map.hpp"
#include "goods.hpp"
#include "log.h"
#include "path_helper.hpp"
#include "point.hpp"
#include "robot.hpp"
#include "ship.hpp"
#include <algorithm>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// This is the IoLayer class. It is responsible for all input/output operations.
class IoLayerNew {

private:
  enum CommandInst {
    // 机器人指令
    ROBOT_MOVE,
    ROBOT_GET,
    ROBOT_PULL,
    ROBOT_LBOT,
    // 船只指令
    SHIP_MOVE,
    SHIP_ROT,
    SHIP_LBOAT,
    SHIP_DEPT,
    SHIP_BERTH
  };

  struct Command {
    CommandInst inst;
    int arg1;
    int arg2;
  };

public:
  GameMap game_map; // 初始地图
  std::unordered_map<Point, Goods> map_goods_list;

  /* 每帧更新的信息*/
  std::vector<Berth> berths{};         // 靠泊点信息
  std::vector<Ship> ships{};           // 船只信息
  std::vector<Robot> robots{};         // 机器人信息
  std::vector<Goods> new_goods_list{}; // 新增货物信息

  const int robot_price = 2000;         // 机器人价格
  const int ship_price = 8000;          // 船只价格
  std::vector<Point> robot_shops{};     // 机器人商店
  std::vector<Point> ship_shops{};      // 船只商店
  Point delivery_point = invalid_point; // 交付点

  int ship_capacity = 0;

  int new_goods_num = 0; // 新增货物数量
  int cur_cycle = 0;     // 当前周期
  int cur_money = 0;     // 当前金钱

  bool final_time = false; // 到最后的时间后,机器人之往指定的港口运输货物

  // 港口打表
  // std::array<std::unordered_map<Point, PointCost>, 10>
  //     berths_come_from_for_robot;
  // std::array<std::unordered_set<Point>, 10> berths_come_from_set_for_robot;

  std::vector<ComeFromMap> berths_come_from_for_robot{};
  std::vector<ComeFromMap> berths_come_from_for_ship{};

  int total_goods_num = 0;    // 总货物数量
  int total_goods_money = 0;  // 总货物价值
  int goted_goods_num = 0;    // 已经获取的货物数量
  int goted_goods_money = 0;  // 已经获取的货物价值
  int selled_goods_num = 0;   // 已经卖出的货物数量
  int selled_goods_money = 0; // 已经卖出的货物价值

  /* 每帧的输出指令 */
  std::vector<Command> commands;

  explicit IoLayerNew() {}
  ~IoLayerNew(){};

  bool berth_is_baned(const int berth_id) {
    // 到了最后时刻,只能往指定的港口运输货物
    if (final_time) {
      std::vector<int> final_berth;
      // 将还能动的船的目的地加入
      for (int i = 0; i < ships.size(); i++) {
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
    for (int i = 0; i < berths.size(); i++) {
      if (berth_is_baned(i)) {
        continue;
      }

      // auto cur_cost = get_cost_from_berth_to_point(i, p);
      auto cur_cost = berths_come_from_for_robot[i].get_point_cost(p);
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
    for (int i = 0; i < berths.size(); i++) {

      // auto cur_cost = get_cost_from_berth_to_point(i, p);
      auto cur_cost = berths_come_from_for_robot[i].get_point_cost(p);
      if (cur_cost.has_value()) {
        if (cur_cost.value() < min_cost) {
          min_cost = cur_cost.value();
          min_berth_id = i;
        }
      }
    }
    return {min_berth_id, min_cost};
  }

  // std::optional<int> get_cost_from_berth_to_point(const int berth_id,
  //                                                 const Point &to) {
  //   const Point &berth_pos =
  //       Point(berths[berth_id].pos.x, berths[berth_id].pos.y);

  //   auto &cur_berth_com_from_set = berths_come_from_set_for_robot[berth_id];
  //   auto &cur_berth_com_from = berths_come_from_for_robot[berth_id];

  //   // 如何确定查找正确？
  //   if (cur_berth_com_from_set.find(to) != cur_berth_com_from_set.end()) {
  //     return cur_berth_com_from[to].cost;
  //   }
  //   return std::nullopt;
  // }

  // std::vector<Point> get_path_from_berth_to_point(const int berth_id,
  //                                                 const Point &to,
  //                                                 bool &founded) {
  //   const Point &berth_pos =
  //       Point(berths[berth_id].pos.x, berths[berth_id].pos.y);
  //   auto path = PATHHelper::get_path(
  //       berth_pos, to, berths_come_from_for_robot[berth_id], founded);
  //   return path;
  // }

  // std::vector<Point> get_path_from_point_to_berth(const int berth_id,
  //                                                 const Point &from,
  //                                                 bool &founded) {
  //   if (berth_id < 0 || berth_id >= berths.size()) {
  //     log_fatal("berth id out of range, expect 0~%d, actual:%d",
  //     berths.size(),
  //               berth_id);
  //     assert(false);
  //   }
  //   // find the path
  //   const Point &berth_pos =
  //       Point(berths[berth_id].pos.x, berths[berth_id].pos.y);
  //   const Point &goal_pos = from;

  //   auto path = PATHHelper::get_path_reverse(
  //       berth_pos, goal_pos, berths_come_from_for_robot[berth_id], founded);

  //   log_info("berth[%d] (%d,%d) to (%d,%d) size:%d", berth_id,
  //   P_ARG(berth_pos),
  //            P_ARG(goal_pos), path.size());

  //   return path;
  // }

  int minimal_transport_time() {

    int minimal_time = 999999;
    for (int i = 0; i < berths.size(); i++) {
      if (berths[i].transport_time < minimal_time) {
        minimal_time = berths[i].transport_time;
      }
    }
    return minimal_time;
  }

  void print_goods_info() {
    for (int i = 0; i < berths.size(); i++) {
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

  void berths_come_from_init() {
    for (int i = 0; i < berths.size(); i++) {
      const Point &start1 = Point(berths[i].pos.x, berths[i].pos.y);

      auto is_barrier_for_robot = [&](const Point &p) {
        return game_map.is_barrier_for_robot(p);
      };
      auto neighbors_for_robot = [&](const Point &p) {
        return game_map.neighbors_for_robot(p);
      };

      auto is_barrier_for_ship = [&](const Point &p) {
        return game_map.is_barrier_for_ship(p);
      };
      auto is_neighbor_for_ship = [&](const Point &p) {
        return game_map.neighbors_for_ship(p);
      };

      // 主航道（非碰撞区域）的花费为 2
      auto ship_cost = [&](const Point &p) {
        if (game_map.has_collison_effect_for_ship(p)) {
          return 1;
        } else {
          return 2;
        }
      };

      // berths_come_from_for_robot[i] = PATHHelper::bfs_search(
      //     start1, ([&](Point p) { return false; }), is_barrier_for_robot,
      //     neighbors_for_robot, PATHHelper::default_cost, 300000);
      // berths_come_from_set_for_robot[i] =
      //     Tools::map_to_set(berths_come_from_for_robot[i]);

      berths_come_from_for_robot.emplace_back(ComeFromMap());
      berths_come_from_for_robot.back().init(
          "berth[" + std::to_string(i) + "]_come_from_for_robot", start1,
          is_barrier_for_robot, neighbors_for_robot, PATHHelper::default_cost);

      berths_come_from_for_ship.emplace_back(ComeFromMap());
      berths_come_from_for_ship.back().init(
          "berth[" + std::to_string(i) + "]_come_from_for_ship", start1,
          is_barrier_for_ship, is_neighbor_for_ship, ship_cost);
    }
    log_info("berths_come_from initialized");
    for (int i = 0; i < berths.size(); i++) {
      log_info("berth[%d] robot_come_from size:%d,ship_come_from_size:%d", i,
               berths_come_from_for_robot[i].map_size(),
               berths_come_from_for_ship[i].map_size());
    }
  }

  void map_process() {
    std::chrono::high_resolution_clock::time_point start =
        std::chrono::high_resolution_clock::now();

    auto update_berth_area_info = [&](Area &a) {
      auto berth =
          std::find_if(berths.begin(), berths.end(),
                       [&](const Berth &b) { return a.contain(b.pos); });
      log_assert(berth != berths.end(), "berth not found");
      log_assert(!berth->berth_area.valid(), "already set berth area");
      log_assert(a.valid(), "invalid area:%s", a.to_string().c_str());
      berth->set_berth_area(a);
      log_info("berth[%d] area:%s", berth->id, a.to_string().c_str());
    };
    auto update_dock_area_info = [&](Area &a) {
      auto berth =
          std::find_if(berths.begin(), berths.end(),
                       [&](const Berth &b) { return a.contain(b.pos); });

      log_assert(berth != berths.end(), "berth not found");
      log_assert(!berth->berth_area.valid(), "already set dock area");

      log_assert(a.valid(), "invalid area:%s", a.to_string().c_str());
      berth->set_dock_area(a);
      log_info("berth[%d] dock area:%s", berth->id, a.to_string().c_str());
    };

    bool dock_visited[200][200];
    bool berth_visited[200][200];
    std::memset(dock_visited, 0, sizeof(dock_visited));
    std::memset(berth_visited, 0, sizeof(berth_visited));

    for (int i = 0; i < 200; i++) {
      for (int j = 0; j < 200; j++) {
        const auto &pos_type = game_map.get_pos_type({i, j});
        switch (pos_type) {
        case GameMap::ROBOT_SHOP: {
          robot_shops.push_back(Point(i, j));
          log_trace("robot_shop:(%d,%d)", P_ARG(Point(i, j)));
          break;
        };
        case GameMap::SHIP_SHOP: {
          ship_shops.push_back(Point(i, j));
          log_trace("ship_shop:(%d,%d)", P_ARG(Point(i, j)));
          break;
        };
        case GameMap::DELIVERY: {
          delivery_point = Point(i, j);
          log_trace("delivery_point:(%d,%d)", P_ARG(delivery_point));
          break;
        }
        case GameMap::BERTH: {
          // 检测矩形区域
          if (!berth_visited[i][j]) {
            Point left_top = Point(i, j);
            Point right_bottom = Point(i, j);
            while (right_bottom.x < 200 &&
                   game_map.is_berth_pos({right_bottom.x, j})) {
              berth_visited[right_bottom.x][j] = true;
              right_bottom.x++;
            }
            while (right_bottom.y < 200 &&
                   game_map.is_berth_pos({i, right_bottom.y})) {
              berth_visited[i][right_bottom.y] = true;
              right_bottom.y++;
            }
            Area berth_area_tmp =
                Area(left_top, {right_bottom.x - 1, right_bottom.y - 1});
            update_berth_area_info(berth_area_tmp);

            for (int x = left_top.x; x < right_bottom.x; x++) {
              for (int y = left_top.y; y < right_bottom.y; y++) {
                berth_visited[x][y] = true;
              }
            }
          }
          break;
        }
        case GameMap::DOCK: {
          // 检测矩形区域
          if (!dock_visited[i][j]) {
            Point left_top = Point(i, j);
            Point right_bottom = Point(i, j);
            while (right_bottom.x < 200 &&
                   game_map.is_dock_pos({right_bottom.x, j})) {
              right_bottom.x++;
            }
            while (right_bottom.y < 200 &&
                   game_map.is_dock_pos({i, right_bottom.y})) {
              right_bottom.y++;
            }
            Area dock_area_tmp =
                Area(left_top, {right_bottom.x - 1, right_bottom.y - 1});
            update_dock_area_info(dock_area_tmp);
            for (int x = left_top.x; x < right_bottom.x; x++) {
              for (int y = left_top.y; y < right_bottom.y; y++) {
                dock_visited[x][y] = true;
              }
            }
          }
          break;
        }
        default:
          break;
        }
      }
    }

    for (auto &berth : berths) {
      log_assert(berth.dock_area.contain(berth.pos),
                 "berth[%d] dock_area not contain pos:(%d,%d)", berth.id,
                 P_ARG(berth.pos));
      log_assert(berth.berth_area.contain(berth.pos),
                 "berth[%d] berth_area not contain pos:(%d,%d)", berth.id,
                 P_ARG(berth.pos));
      log_assert(berth.dock_area.contain(berth.berth_area),
                 "berth[%d] dock_area not contain berth_area", berth.id);
    }

    std::chrono::high_resolution_clock::time_point end =
        std::chrono::high_resolution_clock::now();
    log_info("map_process done, time:%d ms",
             std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                 .count());
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
    for (int i = 0; i < berths.size(); i++) {
      if (berth_is_baned(i)) {
        continue;
      }
      Point berth_pos = Point(berths[i].pos.x, berths[i].pos.y);
      // auto cur_path = get_path_from_point_to_berth(i, from, found_path);

      auto cur_path = berths_come_from_for_robot[i].get_path_from_point(
          from, found_path);

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
             min_berth_id, berths[min_berth_id].pos.x,
             berths[min_berth_id].pos.y, path.size());
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
    for (int i = 0; i < berths.size(); i++) {
      if (berth_is_baned(i)) {
        continue;
      }

      Point berth_pos = Point(berths[i].pos.x, berths[i].pos.y);
      // auto cur_path = get_path_from_point_to_berth(i, from, found_path);
      auto cur_path = berths_come_from_for_robot[i].get_path_from_point(
          from, found_path);

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
    int min_berth_id = std::rand() % berths.size();
    bool found_path = false;

    // 只要能到达任意一个港口就行
    for (int i = 0; i < berths.size(); i++) {
      if (berth_is_baned(i)) {
        continue;
      }
      if (std::any_of(visit.begin(), visit.end(),
                      [i](int v) { return v == i; })) {
        continue;
      }

      Point berth_pos = Point(berths[i].pos.x, berths[i].pos.y);
      // auto cur_path = get_path_from_point_to_berth(i, from, found_path);
      auto cur_path = berths_come_from_for_robot[i].get_path_from_point(
          from, found_path);

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
             min_berth_id, berths[min_berth_id].pos.x,
             berths[min_berth_id].pos.y, path.size());
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

  // void robot_dead_list_init() {
  //   for (int i = 0; i < berths.size(); i++) {
  //     robots_dead[i] = false;
  //   }
  // }

  // ------------------------------------------
  // 初始化
  // ------------------------------------------
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
    int berth_num = 0;
    scanf("%d", &berth_num);
    for (int i = 0; i < berth_num; i++) {
      int berth_id = -1;
      int x = -1;
      int y = -1;
      int loadingspeed = -1;
      scanf("%d%d%d%d", &berth_id, &x, &y, &loadingspeed);
      berths.emplace_back(Berth{berth_id, Point{x, y}, 0, loadingspeed});
    }
    std::sort(berths.begin(), berths.end(),
              [](const Berth &a, const Berth &b) { return a.id < b.id; });

    for (int i = 0; i < berths.size(); i++) {
      log_assert(berths[i].id == i, "berth id:%d", berths[i].id);
      log_info("berth[%d](%d,%d),loading_speed:%d", i, berths[i].pos.x,
               berths[i].pos.y, berths[i].loading_speed);
    }

    // 计算 transport_time 平均数
    int loading_speed_sum = 0;
    for (int i = 0; i < berths.size(); i++) {
      loading_speed_sum += berths[i].loading_speed;
    }

    float loading_speed_avg =
        static_cast<float>(loading_speed_sum) / berths.size();

    for (int i = 0; i < berths.size(); i++) {
      berths[i].avg_berth_loading_speed = loading_speed_avg;
    }
    log_info("Berths initialized,loading_speed_avg:%f", loading_speed_avg);
  }

  void init_done() {
    char okk[10];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
  }

  void init_ships() {
    scanf("%d", &ship_capacity);
    log_info("Ships initialized,ship_capacity:%d", ship_capacity);
  }

  void init() {
    auto start = std::chrono::high_resolution_clock::now();
    init_game_map();
    init_berths();
    // game_map.init_robot_pos();
    berths_come_from_init();
    init_ships();
    auto end = std::chrono::high_resolution_clock::now();
    log_info("IoLayer init time:%d ms",
             std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                 .count());
    map_process();
    init_done();
  }

  // ------------------------------------------
  // 输入输出
  // ------------------------------------------
  void input_cycle() {
    // 读取当前周期的信息
    scanf("%d%d", &cur_cycle, &cur_money);
    log_info("cycle[%d],money:%d", cur_cycle, cur_money);
    // 读取新增货物信息
    new_goods_list.clear();
    scanf("%d", &new_goods_num);
    for (int i = 0; i < new_goods_num; i++) {
      int x, y, money;
      scanf("%d%d%d", &x, &y, &money);
      if (money != 0) {
        new_goods_list.push_back(
            Goods{Point(x, y), money, cur_cycle + 1000, GoodsStatus::Normal});
        log_trace("new goods[%d]:(%d,%d),money:%d,end_cycle:%d", i, x, y, money,
                  cur_cycle + 1000);
        total_goods_money += money;
        total_goods_num++;
      }
    }

    // 读取机器人信息
    int robots_num = -1;
    scanf("%d", &robots_num);

    for (int i = 0; i < robots_num; i++) {
      int id, goods, x, y;
      scanf("%d%d%d%d", &id, &goods, &x, &y);
      // 添加新的机器人
      if (std::find_if(robots.begin(), robots.end(), [&id](Robot &r) {
            return r.id == id;
          }) == robots.end()) {
        robots.emplace_back(Robot{id, Point(x, y), goods == 1, 1});
        log_info("new robot[%d](%d,%d),goods:%d", i, x, y, goods);
        continue;
      }
      // 更新老的机器人
      robots[i].had_goods = goods == 1;
      robots[i].pos = Point(x, y);
      robots[i].status = 1;
      log_info("robot[%d](%d,%d),goods:%d", i, x, y, goods);
    }

    for (int i = 0; i < robots.size(); i++) {
      log_info("robot[%d](%d,%d),goods:%d, robots.size():%d", i,
               robots[i].pos.x, robots[i].pos.y, robots[i].had_goods,
               robots.size());
      log_assert(robots[i].id == i, "robot id:%d,i:%d", robots[i].id, i);
    }

    // 船只状态
    int ship_nums = -1;
    scanf("%d", &ship_nums);
    log_info("ship_nums:%d", ship_nums);
    for (int i = 0; i < ship_nums; i++) {
      int ship_id, goods_num, x, y, direction, status;
      scanf("%d%d%d%d%d%d", &ship_id, &goods_num, &x, &y, &direction, &status);
      if (std::find_if(ships.begin(), ships.end(), [&ship_id](Ship &ship) {
            return ship.id == ship_id;
          }) == ships.end()) {
        ships.push_back(Ship{ship_id, goods_num, ship_capacity, Point(x, y),
                             Direction::int_to_direction(direction), status});
        log_info("new ship[%d](%d,%d)", i, x, y);
        continue;
      }
      ships[i].cur_capacity = goods_num;
      ships[i].pos.x = x;
      ships[i].pos.y = y;
      ships[i].direction = Direction::int_to_direction(direction);
      ships[i].status = status;
      log_info("ship[%d](%d,%d),direction:%d", i, x, y, direction);
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
      case ROBOT_LBOT:
        std::sprintf(fmt_buf, "lbot %d %d\n", command.arg1, command.arg2);
        break;
      case SHIP_MOVE:
        std::sprintf(fmt_buf, "ship %d\n", command.arg1);
        break;
      case SHIP_ROT:
        std::sprintf(fmt_buf, "rot %d %d\n", command.arg1, command.arg2);
        break;
      case SHIP_LBOAT:
        std::sprintf(fmt_buf, "lboat %d %d\n", command.arg1, command.arg2);
        break;
      case SHIP_DEPT:
        std::sprintf(fmt_buf, "dept %d\n", command.arg1);
        break;
      case SHIP_BERTH:
        std::sprintf(fmt_buf, "berth %d\n", command.arg1);
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

  // ------------------------------------------
  // 机器人指令
  // ------------------------------------------
  void robot_move(const int robot_id, const Direction::Direction direction) {
    commands.push_back({ROBOT_MOVE, robot_id, static_cast<int>(direction)});
  }
  void robot_move(int robot_id, Point to) {
    this->robot_move(robot_id,
                     Direction::calc_direction(robots[robot_id].pos, to));
  }

  void robot_get(int robot_id) { commands.push_back({ROBOT_GET, robot_id, 0}); }
  void robot_pull(int robot_id) {
    commands.push_back({ROBOT_PULL, robot_id, 0});
  }

  void robot_lbot(const Point &pos) {
    commands.push_back({ROBOT_LBOT, pos.x, pos.y});
  }

  // ------------------------------------------
  // 船只指令
  // ------------------------------------------
  void ship_move(const int ship_id) {
    commands.push_back({SHIP_MOVE, ship_id, 0});
  }

  void ship_dept(const int ship_id) {
    commands.push_back({SHIP_DEPT, ship_id, 0});
  }
  void ship_berth(const int ship_id) {
    commands.push_back({SHIP_BERTH, ship_id, 0});
  }
  // 0表示顺时针方向，1表示逆时针方向
  void ship_rot(const int ship_id, const int rot_direction) {
    log_assert(rot_direction == 0 || rot_direction == 1, "rot_direction:%d",
               rot_direction);
    commands.push_back({SHIP_ROT, ship_id, rot_direction});
  }
  void ship_lboat(const Point &pos) {
    commands.push_back({SHIP_LBOAT, pos.x, pos.y});
  }

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

  //-----------------test----------------------------------

  // void test_berths_come_from() {
  //   // game_map.print_map_with_point();

  //   for (int i = 100; i < 120; i++) {
  //     game_map.print_map_line(i);
  //   }

  //   for (int i = 0; i < berths.size(); i++) {
  //     // (117,3) (36, 173);
  //     // (117,3) to (107,122)
  //     Point cur = Point(107, 122);
  //     bool founded = false;
  //     auto path = get_path_from_point_to_berth(i, cur, founded);
  //     if (founded) {
  //       log_info("berth[%d] (%d,%d)path size:%d", i, berths[i].pos.x,
  //                berths[i].pos.y, path.size());
  //       for (const auto &p : path) {
  //         log_info("(%d,%d)", p.x, p.y);
  //       }
  //     } else {
  //       log_info("berth[%d] path not found", i);
  //     }
  //   }
  // }
};