#pragma once

#include "berth.h"
#include "game_map.h"
#include "goods.h"
#include "robot.h"
#include "ship.h"
#include <array>
#include <optional>
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
  std::optional<GameMap> game_map_original; // 初始地图

  /* 每帧更新的信息*/
  std::array<Berth, BERTH_NUM> berths;             // 靠泊点信息
  std::array<Ship, SHIP_NUM> ships;                // 船只信息
  std::array<Robot, ROBOT_NUM> robots;             // 机器人信息
  std::array<Goods, NEW_GOODS_NUM> new_goods_list; // 新增货物信息
  int new_goods_num;                               // 新增货物数量
  int cur_cycle;                                   // 当前周期
  int cur_money;                                   // 当前金钱

  /* 每帧的输出指令 */
  std::vector<Command> commands;

  explicit IoLayer();
  bool init_game_map();
  void init_berths();
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