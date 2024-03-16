#pragma once

#include "point.hpp"
struct Berth {
  Point pos;
  int transport_time;
  int loading_speed;
  int id;
  Berth() {}
  bool in_berth_area(const Point &p) {
    const auto& left_top = pos;
    const auto right_bottom = Point{pos.x + 3, pos.y + 3};
    return p.x >= left_top.x && p.x <= right_bottom.x && p.y >= left_top.y &&
           p.y <= right_bottom.y;
  }
};