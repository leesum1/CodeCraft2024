#pragma once

#include "log.h"
class Ship {

public:
  int id;
  int capacity;
  int status;
  int berth_id;

  // 一些状态位置
  int berth_wait_cycle; // 等待进入泊位的周期数
  int goods_wait_cycle; // 在泊位等待货物的周期数

  int cur_capacity; // 当前载重量
  int cur_value;    // 当前价值

  int inst_remine_cycle; // 当前指令剩余周期

  bool good_wait_tolong() { return this->goods_wait_cycle > 50; }

  bool can_accpet_inst() { return this->inst_remine_cycle <= 0; }
  void new_inst(int inst_cycle) {
    log_assert(this->inst_remine_cycle == 0, "inst_remine_cycle is not 0");
    log_assert(inst_cycle > 0, "inst_cycle is not positive");

    this->inst_remine_cycle = inst_cycle;
  }
  void load(int value) {
    log_assert(value > 0 && value <= 200, "value is not positive, %d", value);
    this->cur_capacity++;
    this->cur_value += value;
  }
  void unload() {
    log_assert(this->cur_capacity > 0, "cur_capacity is not positive, %d",
               this->cur_capacity);
    log_assert(this->cur_value > 0, "cur_value is not positive %d",
               this->cur_value);

    log_assert(this->berth_id == -1, "berth_id is not -1, %d", this->berth_id);

    log_trace("Ship %d unload, cur_capacity: %d, cur_value: %d", this->id,
              this->cur_capacity, this->cur_value);
    this->cur_capacity = 0;
    this->cur_value = 0;
  }
  void in_virtual_point() { this->status = -1; }
  bool full() { return this->cur_capacity >= this->capacity; }

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
  }
};