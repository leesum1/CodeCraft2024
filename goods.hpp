#pragma once

#include "point.hpp"
#include <string>

enum class GoodsStatus {
  Normal = 0, // 正常状态, 未被预定
  Booked,     // 被一个机器人预定
  Got,        // 已经被一个机器人拿走
  Dead,       // 消失状态
};

class Goods {
public:
  Point pos;          // 货物位置
  int money;          // 货物价值
  int end_cycle;      // 货物消失的周期
  GoodsStatus status; // 货物状态
  int to_near_berth_cost = 20000; // 到最近的停靠点的距离

   std::string to_string() const {
    return "Goods: pos: (" + std::to_string(pos.x) + ", " +
           std::to_string(pos.y) + "), money: " + std::to_string(money) +
           ", end_cycle: " + std::to_string(end_cycle) +
           ", status: " + std::to_string(static_cast<int>(status));
  }

  bool is_disappeared(const int cur_cycle) {
    return (cur_cycle > end_cycle) || (status == GoodsStatus::Dead);
  }

  Goods(Point pos, int money, int end_cycle, GoodsStatus status)
      : pos(pos), money(money), end_cycle(end_cycle), status(status) {}

  Goods()
      : pos(Point(-1, -1)), money(0), end_cycle(0), status(GoodsStatus::Dead) {}

  // 拷贝构造函数
  Goods(const Goods &other)
      : pos(other.pos), money(other.money), end_cycle(other.end_cycle),
        status(other.status) {
    to_near_berth_cost = other.to_near_berth_cost;
  }
  ~Goods() = default;

  bool operator==(const Goods &other) const {
    return pos == other.pos && money == other.money &&
           end_cycle == other.end_cycle && status == other.status;
  }
  bool operator!=(const Goods &other) const { return !(*this == other); }
};

const static Goods invalid_goods =
    Goods(Point(-1, -1), 0, 0, GoodsStatus::Dead);