
#pragma once

#include <cstddef>
#include <functional>

struct Point {
  int x;
  int y;
  Point(){};
  Point(int x, int y) {
    this->x = x;
    this->y = y;
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
  PointCost() {};
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

    return ((hash<int>()(p.x) ^ (hash<int>()(p.y) << 1)) >> 1);
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