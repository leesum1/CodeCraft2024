#pragma once

#include "point.hpp"

class Goods {
public:
  Point pos;     // 货物位置
  int money;     // 货物价值
  int end_cycle; // 货物消失的周期
  bool be_selected = false;

  Goods(Point pos, int money, int end_cycle)
      : pos(pos), money(money), end_cycle(end_cycle) {}

  Goods() : pos(Point(-1, -1)), money(0), end_cycle(0) {}

  // 拷贝构造函数
  Goods(const Goods &other)
      : pos(other.pos), money(other.money), end_cycle(other.end_cycle) {}
  ~Goods() {}
};