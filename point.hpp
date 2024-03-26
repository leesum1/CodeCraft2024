
#pragma once

#include "log.h"
#include <cstddef>
#include <functional>

#define P_ARG(p) (p).x, (p).y

struct Point {
  int x;
  int y;
  Point() : x(-1), y(-1) {}
  Point(int x, int y) : x(x), y(y) {}

  static bool at_same_row_or_col(const Point &p1, const Point &p2) {
    log_assert(!is_same(p1, p2), "is_same");
    return at_same_row(p1, p2) || at_same_col(p1, p2);
  }

  static bool at_same_row(const Point &p1, const Point &p2) {
    log_assert(!is_same(p1, p2), "is_same");
    return p1.x == p2.x;
  }
  static bool at_same_col(const Point &p1, const Point &p2) {
    log_assert(!is_same(p1, p2), "is_same");
    return p1.y == p2.y;
  }
  static bool is_adjacent(const Point &p1, const Point &p2) {
    log_assert(!is_same(p1, p2), "is_same");
    return (at_same_row(p1, p2) && abs(p1.x - p2.x) == 1) ||
           (at_same_col(p1, p2) && abs(p1.y - p2.y) == 1);
  }
  static bool is_same(const Point &p1, const Point &p2) {
    return p1.x == p2.x && p1.y == p2.y;
  }

  Point &operator=(const Point &other) {
    if (this != &other) {
      x = other.x;
      y = other.y;
    }
    return *this;
  }

  // 重载比较运算符==
  bool operator==(const Point &other) const {
    return x == other.x && y == other.y;
  }
  // 重载比较运算符!=
  bool operator!=(const Point &other) const { return !(*this == other); }

  int manhattan_distance(const Point other) {
    return abs(x - other.x) + abs(y - other.y);
  }
};

struct PointCost {
  Point pos;
  int cost;
  PointCost(){};
  PointCost(Point pos, int cost) : pos(pos), cost(cost) {}

  friend bool operator<(PointCost f1, PointCost f2) {
    return f1.cost > f2.cost;
  }
};

namespace std {
template <> struct hash<Point> {
  std::size_t operator()(const Point &p) const {
    using std::hash;
    using std::size_t;
    int new_val = (p.x << 8) | p.y;

    return hash<int>()(new_val);
  }
};
} // namespace std

struct PrioItem {
  Point pos;
  int cost;
  PrioItem(Point pos, int cost) {
    this->pos = pos;
    this->cost = cost;
  }
  friend bool operator<(PrioItem f1, PrioItem f2) { return f1.cost > f2.cost; }
};

static const Point invalid_point;
static const Point stop_point(-2, -2);

#define INVALID_POINT invalid_point
