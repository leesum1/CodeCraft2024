#include "goods.h"

int GoodsC::calc_goods_value(const Point robot_pos) {
  const int dis = (this->pos.manhattan_distance(robot_pos));
  if (dis == 0) {
    return money;
  }

  return money / dis;
}

GoodsC::GoodsC(Point pos, int money, int start_cycle) {
  this->pos = pos;
  this->money = money;
  this->start_cycle = start_cycle;
  this->end_cycle = start_cycle + 20 * 50;

  this->is_dead = false;
}
GoodsC::GoodsC() {
  this->pos = Point(0, 0);
  this->money = 0;
  this->start_cycle = 0;
  this->end_cycle = 0;

  this->is_dead = false;
}

GoodsC::~GoodsC() {}
