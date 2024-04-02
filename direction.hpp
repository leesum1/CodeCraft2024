#pragma once

#include "log.h"
#include "point.hpp"
#include <map>
#include <optional>
namespace Direction {
enum Direction { RIGHT, LEFT, UP, DOWN };
enum Rotate { CLOCKWISE = 0, COUNTERCLOCKWISE = 1 }; // 顺时针，逆时针

inline std::optional<Rotate> calc_rotate_direction(const Direction &from,
                                                   const Direction &to) {
  if (from == to) {
    return std::nullopt;
  }

  static const std::map<std::pair<Direction, Direction>, Rotate> rotate_map = {
      {{Direction::RIGHT, Direction::UP}, Rotate::COUNTERCLOCKWISE},
      {{Direction::RIGHT, Direction::DOWN}, Rotate::CLOCKWISE},
      {{Direction::LEFT, Direction::UP}, Rotate::CLOCKWISE},
      {{Direction::LEFT, Direction::DOWN}, Rotate::COUNTERCLOCKWISE},
      {{Direction::UP, Direction::RIGHT}, Rotate::CLOCKWISE},
      {{Direction::UP, Direction::LEFT}, Rotate::COUNTERCLOCKWISE},
      {{Direction::DOWN, Direction::RIGHT}, Rotate::COUNTERCLOCKWISE},
      {{Direction::DOWN, Direction::LEFT}, Rotate::CLOCKWISE}};

  auto it = rotate_map.find({from, to});
  if (it == rotate_map.end()) {
    return std::nullopt;
  }
  return it->second;
}

inline Direction int_to_direction(int x) {
  switch (x) {
  case 0:
    return Direction::RIGHT;
  case 1:
    return Direction::LEFT;
  case 2:
    return Direction::UP;
  case 3:
    return Direction::DOWN;
  default:
    log_fatal("invalid direction %d", x);
    assert(false);
  }
}

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

inline Direction calc_direction(Area &from, const Point &to) {
  log_assert(from.valid(), "invalid area %s", from.to_string().c_str());
  log_assert(!from.contain(to), "from:%s to(%d,%d)", from.to_string().c_str(),
             P_ARG(to));

  if (to.y > from.right_bottom.y) {
    return Direction::RIGHT;
  } else if (to.y < from.left_top.y) {
    return Direction::LEFT;
  } else if (to.x < from.left_top.x) {
    return Direction::UP;
  } else if (to.x > from.right_bottom.x) {
    return Direction::DOWN;
  } else {
    log_fatal("from:%s to(%d,%d)", from.to_string().c_str(), P_ARG(to));
  }
}

inline Direction calc_direction_nocheck(const Point &from, const Point &to) {

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
