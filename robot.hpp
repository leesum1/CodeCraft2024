#pragma once

#include "goods.hpp"
#include "point.hpp"
#include <vector>

class Robot {
  Point next_pos_final = invalid_point; // 下下次移动的位置
  bool next_pos_final_is_valid = false;

public:
  int id; // 机器人编号
  Point pos; // 机器人位置
  bool had_goods; // 机器人是否有货物
  int status; // 机器人状态
  int target_berth_id = -1; // 机器人目标泊位编号

  // 额外信息
  Point next_pos_before_collision_check = invalid_point; // 下一次移动的位置

  std::vector<Point> path_list{}; // 机器人路径
  bool will_goods_in_this_cycle = false; // 本周期是否拿货
  bool has_pass_collision_check = false; // 是否已经通过碰撞检测
  Goods target_goods = invalid_goods; // 机器人目标货物
  int priority = 0; // 机器人优先级
  int idle_cycle = 0;
  int collision_cycle = 0;
  int goted_goods_count = 0;
  int goted_goods_value = 0;
  std::vector<Goods> goods_list{}; // 机器人拥有的货物列表

  explicit Robot() {
    this->id = 0;
    this->pos = Point(0, 0);
    this->had_goods = false;
    this->status = 0;
    this->target_berth_id = -1;
  }

  void pull_goods_statistic(const Goods& goods) {
    goted_goods_count++;
    goted_goods_value += goods.money;
    goods_list.push_back(goods);
  }

  void printf_goods_statistic() {
        // 以 20 为单位,统计货物价值
        std::array<int, 10> goods_value_step_by_20{};

        for (auto& goods : goods_list) {
            int step = goods.money / 20;
            if (step >= 10) {
                step = 9;
            }
            goods_value_step_by_20[step]++;
        }
        log_info("robot[%d] goted goods count:%d, value:%d", id, goted_goods_count, goted_goods_value);
        for (int i = 0; i < 10; i++) {
            log_info("robot[%d] goods->[%d-%d]:%d",id, i * 20, (i + 1) * 20 - 1,
                     goods_value_step_by_20[i]);
        }
  }

  Point get_next_pos() {
    if (next_pos_final_is_valid) {
      return next_pos_final;
    }
    return next_pos_before_collision_check;
  }

  void set_final_next_pos(const Point& p) {
    next_pos_final = p;
    next_pos_final_is_valid = true;
  }

  explicit Robot(int id, Point pos, bool had_goods, int status)
    : id(id), pos(pos), had_goods(had_goods), status(status) {}

  void clear_flags() {
    has_pass_collision_check = false;
    will_goods_in_this_cycle = false;
    next_pos_final_is_valid = false;
    next_pos_final = invalid_point;
    next_pos_before_collision_check = invalid_point;
  }


  void book_new_goods(const Goods& goods, const std::vector<Point>& path) {
    target_goods = goods;
    target_goods.status = GoodsStatus::Booked;
    path_list = path;
    next_pos_before_collision_check = path_list.back();
  }

  void update_next_pos() {
    log_assert(!path_list.empty(), "path_list is empty");
    next_pos_before_collision_check = path_list.back();
  }

  ~Robot() = default;
};
