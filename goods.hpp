#pragma once

#include "point.hpp"

class Goods {
public:
  Point pos;     // 货物位置
  int money;     // 货物价值
  int end_cycle; // 货物消失的周期
  bool be_selected = false;

  bool is_disappeared(int cur_cycle) {
    return (cur_cycle > end_cycle) || pos != invalid_point;
  }

  Goods(Point pos, int money, int end_cycle)
      : pos(pos), money(money), end_cycle(end_cycle) {}

  Goods() : pos(Point(-1, -1)), money(0), end_cycle(0) {}

  // 拷贝构造函数
  Goods(const Goods &other)
      : pos(other.pos), money(other.money), end_cycle(other.end_cycle) {}
  ~Goods() {}

  bool operator==(const Goods &other) const {
    return pos == other.pos && money == other.money &&
           end_cycle == other.end_cycle && be_selected == other.be_selected;
  }
  bool operator!=(const Goods &other) const { return !(*this == other); }
};

const static Goods invalid_goods = Goods(Point(-1, -1), 0, 0);