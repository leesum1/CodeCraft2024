#pragma once

#include "direction.hpp"
#include "log.h"
#include "point.hpp"
#include <array>
#include <utility>

class Ship {

public:
  int id;
  int capacity;
  // 正常行驶状态（状态0）恢复状态（状态1）装载状态（状态2)
  int status;
  int berth_id;
  Point pos; // 当前位置
  Direction::Direction direction;

  // 一些状态位置
  int berth_wait_cycle = 0;       // 等待进入泊位的周期数
  int goods_wait_cycle = 0;       // 在泊位等待货物的周期数
  bool is_last_transport = false; // 是否是最后一次运输
  int last_berth_id = -1;         // 上一次泊位id
  bool is_dead = false;           // 无法再接受指令

  bool has_change_berth = false; // 是否已经换泊位,即在泊位中移动

  int cur_capacity = 0; // 当前载重量
  int cur_value = 0;    // 当前价值

  int inst_remine_cycle = 0; // 当前指令剩余周期
  int spend_cycle = 0;       // 当前周期花费

  bool good_wait_tolong() { return this->goods_wait_cycle > 50; }
  int capacity_percent() { return this->cur_capacity * 100 / this->capacity; }

  bool can_accpet_inst() { return this->inst_remine_cycle <= 0; }

  std::array<Point, 2> get_ship_head() {
    switch (this->direction) {
    case Direction::RIGHT: {
      return {Point(this->pos.x, this->pos.y + 2),
              Point(this->pos.x + 1, this->pos.y + 2)};
      break;
    }
    case Direction::LEFT: {
      return {Point(this->pos.x, this->pos.y - 2),
              Point(this->pos.x - 1, this->pos.y - 2)};
      break;
    }
    case Direction::UP: {
      return {Point(this->pos.x - 2, this->pos.y),
              Point(this->pos.x - 2, this->pos.y + 1)};
      break;
    }
    case Direction::DOWN:
      return {Point(this->pos.x + 2, this->pos.y),
              Point(this->pos.x + 2, this->pos.y - 1)};
      break;
    }

    log_assert(false, "get_ship_head error");
  }

  Area get_ship_area() { return calc_ship_area(this->pos, this->direction); }

  static Area calc_ship_area(const Point &pos, Direction::Direction dir) {
    switch (dir) {
    case Direction::RIGHT: {
      return Area(pos, Point(pos.x + 1, pos.y + 2));
      break;
    }
    case Direction::LEFT: {
      return Area({pos.x - 1, pos.y - 2}, pos);
      break;
    }
    case Direction::UP: {
      return Area({pos.x - 2, pos.y}, {pos.x, pos.y + 1});
      break;
    }
    case Direction::DOWN:
      return Area({pos.x, pos.y - 1}, {pos.x + 2, pos.y});
      break;
    }

    log_assert(false, "get_ship_area error");
  }

  static std::pair<Point, Direction::Direction>
  calc_rot_action(const Point &pos, const Direction::Direction dir,
                  bool clockwise_direction) {
    switch (dir) {
    case Direction::RIGHT: {
      if (clockwise_direction) {
        return std::make_pair(Point(pos.x, pos.y + 2), Direction::DOWN);
      } else {
        return std::make_pair(Point(pos.x + 1, pos.y + 1), Direction::UP);
      }
      break;
    }
    case Direction::LEFT: {
      if (clockwise_direction) {
        return std::make_pair(Point(pos.x, pos.y - 2), Direction::UP);
      } else {
        return std::make_pair(Point(pos.x - 1, pos.y - 1), Direction::DOWN);
      }
      break;
    }
    case Direction::UP: {
      if (clockwise_direction) {
        return std::make_pair(Point(pos.x - 2, pos.y), Direction::RIGHT);
      } else {
        return std::make_pair(Point(pos.x - 1, pos.y + 1), Direction::LEFT);
      }
      break;
    }
    case Direction::DOWN:
      if (clockwise_direction) {
        return std::make_pair(Point(pos.x + 2, pos.y), Direction::LEFT);
      } else {
        return std::make_pair(Point(pos.x + 1, pos.y - 1), Direction::RIGHT);
      }
      break;
    }

    log_assert(false, "calc_rot_action error");
    return std::make_pair(Point(-1, -1), Direction::UP);
  }

  void new_inst(int inst_cycle) {
    log_assert(this->inst_remine_cycle == 0, "inst_remine_cycle is not 0 ,%d",
               this->inst_remine_cycle);
    log_assert(inst_cycle > 0, "inst_cycle is not positive");

    this->inst_remine_cycle = inst_cycle;
  }
  void load(int value) {
    log_assert(value > 0 && value <= 200, "value is not positive, %d", value);
    this->cur_capacity++;
    this->cur_value += value;
  }
  void unload() {
    log_assert(this->cur_capacity >= 0, "cur_capacity is not positive, %d",
               this->cur_capacity);
    log_assert(this->cur_value >= 0, "cur_value is not positive %d",
               this->cur_value);

    log_assert(this->berth_id == -1, "berth_id is not -1, %d", this->berth_id);

    log_trace("Ship %d unload, cur_capacity: %d, cur_value: %d", this->id,
              this->capacity_percent(), this->cur_value);
    this->cur_capacity = 0;
    this->cur_value = 0;
  }
  void in_virtual_point() { this->status = -1; }
  bool full() { return this->cur_capacity >= this->capacity; }

  explicit Ship(int id, int cur_capacity, int max_capacity, const Point &pos,
                Direction::Direction direction, int status)
      : id(id), cur_capacity(cur_capacity), capacity(max_capacity),
        status(status), pos(pos), direction(direction) {}

  explicit Ship() {
    this->id = 0;
    this->capacity = 0;
    this->status = 0;
    this->berth_id = -1;
    this->berth_wait_cycle = 0;
    this->cur_capacity = 0;
    this->cur_value = 0;
    this->goods_wait_cycle = 0;
    this->inst_remine_cycle = 0;
    this->spend_cycle = 0;
    this->is_last_transport = false;
    this->is_dead = false;
  }
};
