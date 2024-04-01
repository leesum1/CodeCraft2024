#pragma once

#include "point.hpp"
namespace Direction {
enum Direction { RIGHT, LEFT, UP, DOWN };

inline Direction calc_direction(const Point &from, const Point &to) {

  // to 只能是 from 的上下左右，其余情况不合法
  if (abs(from.x - to.x) + abs(from.y - to.y) != 1) {
    log_fatal("from(%d,%d) to(%d,%d) not valid", from.x, from.y, to.x, to.y);
    assert(false);
  }

  if (from.x == to.x) {
    if (from.y > to.y) {
      return Direction::LEFT;
    } else {
      return Direction::RIGHT;
    }
  } else {
    if (from.x > to.x) {
      return Direction::UP;
    } else {
      return Direction::DOWN;
    }
  }
}

} // namespace Direction
