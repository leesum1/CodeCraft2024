#pragma once

#include "log.h"
#include "point.hpp"
#include <map>
#include <optional>
#include <vector>
namespace Direction {
enum Direction { RIGHT, LEFT, UP, DOWN };
enum Rotate { CLOCKWISE = 0, COUNTERCLOCKWISE = 1 }; // 顺时针，逆时针

inline Point move(const Point &p, const Direction &dir) {
  switch (dir) {
  case Direction::RIGHT:
    return Point(p.x, p.y + 1);
  case Direction::LEFT:
    return Point(p.x, p.y - 1);
  case Direction::UP:
    return Point(p.x - 1, p.y);
  case Direction::DOWN:
    return Point(p.x + 1, p.y);
  default:
    log_fatal("invalid direction %d", dir);
    assert(false);
  }
}

inline Rotate opposite_rotate(const Rotate &rot) {
  switch (rot) {
  case Rotate::CLOCKWISE:
    return Rotate::COUNTERCLOCKWISE;
  case Rotate::COUNTERCLOCKWISE:
    return Rotate::CLOCKWISE;
  default:
    log_fatal("invalid rotate %d", rot);
    assert(false);
  }
}

inline Direction opposite_direction(const Direction &dir) {
  switch (dir) {
  case Direction::RIGHT:
    return Direction::LEFT;
  case Direction::LEFT:
    return Direction::RIGHT;
  case Direction::UP:
    return Direction::DOWN;
  case Direction::DOWN:
    return Direction::UP;
  default:
    log_fatal("invalid direction %d", dir);
    assert(false);
  }
}

inline std::optional<Rotate> calc_rotate_direction(const Direction &from,
                                                   const Direction &to) {
  if (from == to) {
    log_fatal("from:%d to:%d", from, to);
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
    log_fatal("from:%d to:%d", from, to);
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

inline std::vector<Direction> calc_direction(Area &from, const Point &to) {
  log_assert(from.valid(), "invalid area %s", from.to_string().c_str());
  log_assert(!from.contain(to), "from:[%s] to(%d,%d)", from.to_string().c_str(),
             P_ARG(to));

  if (to.x >= from.left_top.x && to.x <= from.right_bottom.x) {
    if (to.y > from.right_bottom.y) {
      return {Direction::RIGHT};
    } else if (to.y < from.left_top.y) {
      return {Direction::LEFT};
    }
  } else if (to.y >= from.left_top.y && to.y <= from.right_bottom.y) {
    if (to.x < from.left_top.x) {
      return {Direction::UP};
    } else if (to.x > from.right_bottom.x) {
      return {Direction::DOWN};
    }
  } else if (to.x < from.left_top.x) {
    if (to.y < from.left_top.y) {
      return {Direction::UP, Direction::LEFT};
    } else if (to.y > from.right_bottom.y) {
      return {Direction::UP, Direction::RIGHT};
    }
  } else if (to.x > from.right_bottom.x) {
    if (to.y < from.left_top.y) {
      return {Direction::DOWN, Direction::LEFT};
    } else if (to.y > from.right_bottom.y) {
      return {Direction::DOWN, Direction::RIGHT};
    }
  }

  log_fatal("from:%s to(%d,%d)", from.to_string().c_str(), P_ARG(to));

  return {};
}

inline std::vector<Direction> calc_direction_nocheck(const Point &from,
                                                     const Point &to) {

  log_assert(from != to, "from(%d,%d) eq to(%d,%d)", from.x, from.y, to.x,
             to.y);

  if (to.x == from.x) {
    if (to.y < from.y) {
      return {Direction::LEFT};
    } else {
      return {Direction::RIGHT};
    }
  } else if (to.y == from.y) {
    if (to.x < from.x) {
      return {Direction::UP};
    } else {
      return {Direction::DOWN};
    }
  } else if (to.x > from.x) {
    if (to.y > from.y) {
      return {Direction::RIGHT, Direction::DOWN};
    } else {
      return {Direction::LEFT, Direction::DOWN};
    }
  } else if (to.x < from.x) {
    if (to.y > from.y) {
      return {Direction::RIGHT, Direction::UP};
    } else {
      return {Direction::LEFT, Direction::UP};
    }
  }

  log_assert(false, "from(%d,%d) to(%d,%d)", from.x, from.y, to.x, to.y);
  return {};
}

} // namespace Direction
