#pragma once

#include "berth.hpp"
#include "bfs.hpp"
#include "game_map.hpp"
#include "goods.hpp"
#include "log.h"
#include "point.hpp"
#include "robot.hpp"
#include "ship.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <queue>
#include <unordered_map>
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

  /* 每帧更新的信息*/
  std::array<Berth, BERTH_NUM> berths;             // 靠泊点信息
  std::array<Ship, SHIP_NUM> ships;                // 船只信息
  std::array<Robot, ROBOT_NUM> robots;             // 机器人信息
  std::array<Goods, NEW_GOODS_NUM> new_goods_list; // 新增货物信息
  int new_goods_num = 0;                           // 新增货物数量
  int cur_cycle = 0;                               // 当前周期
  int cur_money = 0;                               // 当前金钱

  // 港口打表
  std::array<std::unordered_map<Point, PointCost>, 10> berths_come_from;
  std::array<bool, ROBOT_NUM> robots_dead;

  int total_goods_num = 0;   // 总货物数量
  int total_goods_money = 0; // 总货物价值
  int goted_goods_num = 0;   // 已经获取的货物数量
  int goted_goods_money = 0; // 已经获取的货物价值

  /* 每帧的输出指令 */
  std::vector<Command> commands;

  explicit IoLayer() {}
  ~IoLayer(){};

  void print_final_info() {
    log_info("total_goods_num:%d,total_goods_money:%d,goted_goods_num:%d,"
             "goted_goods_money:%d",
             total_goods_num, total_goods_money, goted_goods_num,
             goted_goods_money);
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
    log_info("Berths initialized");
  }
  void berths_come_from_init() {
    for (int i = 0; i < BERTH_NUM; i++) {
      Point start = Point(berths[i].pos.x + 1, berths[i].pos.y + 1);
      std::queue<Point> q;
      q.push(start);
      int search_lvl = 0;
      berths_come_from[i][start] = PointCost(start, search_lvl);
      while (!q.empty()) {
        int level_size = q.size();
        for (int j = 0; j < level_size; j++) {
          Point cur = q.front();
          q.pop();
          for (const auto &next : game_map.neighbors(cur)) {
            if (berths_come_from[i].find(next) == berths_come_from[i].end()) {
              berths_come_from[i][next] = PointCost(cur, search_lvl);
              q.push(next);
            }
          }
        }
        search_lvl++;
      }
    }
    log_info("berths_come_from initialized");
    for (int i = 0; i < BERTH_NUM; i++) {
      log_info("berth[%d] come from map size: %d", i,
               berths_come_from[i].size());
    }
  }
  std::vector<Point> get_berth_path(const int berth_id, const Point &from,
                                    bool &founded) {
    if (berth_id < 0 || berth_id >= BERTH_NUM) {
      log_fatal("berth id out of range, expect 0~%d, actual:%d", BERTH_NUM,
                berth_id);
      assert(false);
    }
    // find the path
    Point berth_pos =
        Point(berths[berth_id].pos.x + 1, berths[berth_id].pos.y + 1);
    Point goal_pos = from;

    auto path =
        BFS::get_path(berth_pos, goal_pos, berths_come_from[berth_id], founded);

    log_info("berth[%d] (%d,%d) to (%d,%d) size:%d", berth_id, P_ARG(berth_pos),
             P_ARG(goal_pos), path.size());
    if (founded) {
      if (path.front() != from) {
        log_fatal("path.front() != from, path.front():(%d,%d), from:(%d,%d)",
                  path.front().x, path.front().y, from.x, from.y);
        assert(false);
      }
    }
    path.emplace_back(berth_pos);
    std::reverse(path.begin(), path.end());
    path.pop_back();

    return path;
  }

  std::vector<Point> get_near_berth_path(const Point &from, int &berth_id,
                                         bool &founded) {
    std::vector<Point> path;
    int min_dis = 999999;
    int min_berth_id = 0;
    bool found_path = false;
    // 只要能到达任意一个港口就行
    for (int i = 0; i < BERTH_NUM; i++) {
      Point berth_pos = Point(berths[i].pos.x + 1, berths[i].pos.y + 1);
      auto cur_path = get_berth_path(i, from, found_path);
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
    log_info("from(%d,%d) to berth[%d] (%d,%d) size:%d", from.x, from.y,
             min_berth_id, berths[min_berth_id].pos.x + 1,
             berths[min_berth_id].pos.y + 1, path.size());
    return path;
  }

  bool in_berth_area(const Point &p) {
    for (auto &berth : berths) {
      if (berth.in_berth_area(p)) {
        return true;
      }
    }
    return false;
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
      log_info("ship[%d] capacity:%d", i, ships[i].capacity);
    }
    log_info("Ships initialized");
  }
  void init() {
    auto start = std::chrono::high_resolution_clock::now();
    init_game_map();
    init_berths();
    berths_come_from_init();
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
      scanf("%d%d", &ships[i].status, &ships[i].berch_id);
      log_trace("ship [%d] status:%d,berth_id:%d", i, ships[i].status,
                ships[i].berch_id);
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
      auto path = get_berth_path(i, cur, founded);
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