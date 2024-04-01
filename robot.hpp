#pragma once

#include "point.hpp"



class Robot {

public:
  enum RobotNextPointType { GOODS, BERTH, EMPTY, COLLISION, RECOVER };

  int id;                   // 机器人编号
  Point pos;                // 机器人位置
  bool had_goods;           // 机器人是否有货物
  int status;               // 机器人状态
  int target_berth_id = -1; // 机器人目标泊位编号

  explicit Robot() {
    this->id = 0;
    this->pos = Point(0, 0);
    this->had_goods = false;
    this->status = 0;
    this->target_berth_id = -1;
  }
  explicit Robot(int id, Point pos, bool had_goods, int status)
      : id(id), pos(pos), had_goods(had_goods), status(status) {}

  bool target_berth_is_valid() {
    return target_berth_id >= 0 && target_berth_id < 10;
  }

  ~Robot() {}
};