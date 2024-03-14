#pragma once

#include "berth.h"
#include "game_map.h"
#include "goods.h"
#include "point.h"
#include "robot.h"
#include "ship.h"
#include <array>
#include <memory>
#include <optional>
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
  std::shared_ptr<GameMap> game_map_original; // 初始地图

  /* 每帧更新的信息*/
  std::array<Berth, BERTH_NUM> berths;              // 靠泊点信息
  std::array<Ship, SHIP_NUM> ships;                 // 船只信息
  std::array<Robot, ROBOT_NUM> robots;              // 机器人信息
  std::array<GoodsC, NEW_GOODS_NUM> new_goods_list; // 新增货物信息
  int new_goods_num = 0;                            // 新增货物数量
  int cur_cycle = 0;                                // 当前周期
  int cur_money = 0;                                // 当前金钱
  std::array<std::unordered_map<Point, PointCost>, 10> berths_come_from;

  int total_goods_num = 0;   // 总货物数量
  int total_goods_money = 0; // 总货物价值
  int goted_goods_num = 0;   // 已经获取的货物数量
  int goted_goods_money = 0; // 已经获取的货物价值

  /* 每帧的输出指令 */
  std::vector<Command> commands;

  explicit IoLayer();
  bool init_game_map();
  void init_berths();
  void berths_come_from_init();
  std::optional< std::vector<Point>> get_berth_path(int berth_id, Point from);
  void init_ships();
  void init();

  void input_cycle();
  void output_cycle();

  // 机器人指令
  void robot_move(int robot_id, RobotDrirection direction);
  void robot_get(int robot_id);
  void robot_pull(int robot_id);
  void robot_move_get(int robot_id, RobotDrirection direction) {
    robot_move(robot_id, direction);
    robot_get(robot_id);
  }
  void robot_move_pull(int robot_id, RobotDrirection direction) {
    robot_move(robot_id, direction);
    robot_pull(robot_id);
  }

  // 船只指令
  void ship(int ship_id, int berth_id);
  void go(int ship_id);
};