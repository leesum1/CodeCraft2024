#pragma once

#include "berth.h"
#include "io_layer.h"
#include "point.h"
#include <unordered_map>
#include <vector>
class Manager {


  std::vector<RobotC> robots;
  std::vector<Ship> ships;
  std::shared_ptr<GameMap> game_map;
  std::unordered_map<Point, GoodsC> goods_list;
  std::vector<Berth> berth_list;



public:
  IoLayer io_layer;

  Manager();
  void init_game();
  void run_game();
  void bfs_search(RobotC &robot);
  void find_new_goods(RobotC &robot);
  void find_new_berth(RobotC &robot);
  void go_to_goods(RobotC &robot);

  void robot_ctrl_fsm(RobotC &robot);
  void robot_move();
  RobotDrirection calc_direction(Point from, Point to);
  void print_goods_list();
  void collision_detect();
  ~Manager();
};
