
#pragma once

#include "log.h"
#include <cstddef>
#include <functional>
#include <string>
#include <string_view>

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

  static bool is_stop_point(const Point &p) {
    return p.x == -2 && p.y == -2 || p.x == -1 && p.y == -1;
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

struct Area {
  Point left_top;
  Point right_bottom;
  Area(Point left_top, Point right_bottom)
      : left_top(left_top), right_bottom(right_bottom) {}
  Area() : left_top(Point(-1, -1)), right_bottom(Point(-1, -1)) {}

  std::string to_string() {
    return std::string("left_top:") + std::to_string(left_top.x) + "," +
           std::to_string(left_top.y) +
           " right_bottom:" + std::to_string(right_bottom.x) + "," +
           std::to_string(right_bottom.y);
  }

  bool valid() {
    bool area_valid1 = left_top.x != -1 && left_top.y != -1 &&
                       right_bottom.x != -1 && right_bottom.y != -1;
    bool area_valid2 =
        left_top.x < right_bottom.x && left_top.y < right_bottom.y;

    return area_valid1 && area_valid2;
  }

  void reset() {
    left_top = Point(-1, -1);
    right_bottom = Point(-1, -1);
  }
  bool contain(Area &a) {
    log_assert(a.valid(), "invalid area (%d,%d) (%d,%d)", P_ARG(a.left_top),
               P_ARG(a.right_bottom));
    return contain(a.left_top) && contain(a.right_bottom);
  }

  bool contain(const Point &p) {
    log_assert(valid(), "invalid area (%d,%d) (%d,%d)", P_ARG(left_top),
               P_ARG(right_bottom));
    bool x_in = p.x >= left_top.x && p.x <= right_bottom.x;
    bool y_in = p.y >= left_top.y && p.y <= right_bottom.y;
    return x_in && y_in;
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
