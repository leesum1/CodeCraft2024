#pragma once

#include "point.h"
#include <vector>
struct Goods {
  int x, y, money;
  Goods() {}
};

class GoodsC {
public:
  Point pos;            // 货物位置
  int money;            // 货物价值
  int start_cycle;      // 货物开始出现的周期
  int end_cycle;        // 货物消失的周期
  bool is_dead = false; // 货物是否已经被机器人拿走,或者消失
  bool be_selected = false;

  int calc_goods_value(const Point robot_pos);
  GoodsC(Point pos, int money, int start_cycle);
  GoodsC();

  // 拷贝构造函数
  GoodsC(const GoodsC &other)
      : pos(other.pos), money(other.money), start_cycle(other.start_cycle),
        end_cycle(other.end_cycle), is_dead(other.is_dead) {}
  ~GoodsC();
};