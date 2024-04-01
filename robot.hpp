#pragma once

#include "goods.hpp"
#include "point.hpp"
#include <vector>

class Robot {

public:
  int id;                   // 机器人编号
  Point pos;                // 机器人位置
  bool had_goods;           // 机器人是否有货物
  int status;               // 机器人状态
  int target_berth_id = -1; // 机器人目标泊位编号

  // 额外信息
  Point next_pos_before_collision_check = invalid_point; // 下一次移动的位置
  Point next_pos_final = invalid_point;  // 下下次移动的位置
  std::vector<Point> path_list{};        // 机器人路径
  bool will_goods_in_this_cycle = false; // 本周期是否拿货
  bool has_pass_collision_check = false; // 是否已经通过碰撞检测
  Goods target_goods = invalid_goods;    // 机器人目标货物
  int idle_cycle = 0;

  explicit Robot() {
    this->id = 0;
    this->pos = Point(0, 0);
    this->had_goods = false;
    this->status = 0;
    this->target_berth_id = -1;
  }

  Point get_next_pos(){
    if (next_pos_final != invalid_point) {
      return next_pos_final;
    }
    return next_pos_before_collision_check;
  }

  explicit Robot(int id, Point pos, bool had_goods, int status)
      : id(id), pos(pos), had_goods(had_goods), status(status) {}

  void clear_flags() {
    has_pass_collision_check = false;
    will_goods_in_this_cycle = false;
    next_pos_before_collision_check = invalid_point;
    next_pos_final = invalid_point;
  }

  bool target_berth_is_valid() {
    return target_berth_id >= 0 && target_berth_id < 10;
  }

  ~Robot() {}
};