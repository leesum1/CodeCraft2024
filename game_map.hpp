#pragma once

// This is the GameMap class. It is responsible for all game map operations.

#include "log.h"
#include "point.hpp"
#include <array>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>
class GameMap {
public:
  char map[200][200] = {'#'};
  enum PosType {
    LAND,
    MAIN_LAND,
    SEA,
    MAIN_SEA,
    BARRIER,
    ROBOT_SHOP,
    SHIP_SHOP,
    BERTH,
    DOCK,
    SEA_LAND,
    MAIN_SEA_LAND,
    DELIVERY
  };

private:
  // '.' : 空地
  // '>' : 陆地主干道
  // '*' : 海洋
  // '~' : 海洋主干道
  // '#' : 障碍
  // 'R' : 机器人购买地块，同时该地块也是主干道
  // 'S' : 船舶购买地块，同时该地块也是主航道
  // 'B' : 泊位
  // 'K' : 靠泊区
  // 'C' : 海陆立体交通地块
  // 'c' : 海陆立体交通地块，同时为主干道和主航道
  // 'T' : 交货点

  struct PosTypeAttr {
    bool is_barrier_for_robot;
    bool is_barrier_for_ship;
    bool has_collison_effect_for_robot;
    bool has_collison_effect_for_ship;
  };

  const PosTypeAttr land_attr = {false, true, true, true};
  const PosTypeAttr main_land_attr = {false, true, false, true};
  const PosTypeAttr sea_attr = {true, false, true, true};
  const PosTypeAttr main_sea_attr = {true, false, true, false};
  const PosTypeAttr barrier_attr = {true, true, true, true};
  const PosTypeAttr sea_land_attr = {false, false, true, true};
  const PosTypeAttr main_sea_land_attr = {false, false, false, false};

  const std::unordered_map<char, std::pair<PosType, PosTypeAttr>> lookuptable =
      {{'#', {PosType::BARRIER, barrier_attr}},
       {'.', {PosType::LAND, land_attr}},
       {'>', {PosType::MAIN_LAND, main_land_attr}},
       {'*', {PosType::SEA, sea_attr}},
       {'~', {PosType::MAIN_SEA, main_sea_attr}},
       {'R', {PosType::ROBOT_SHOP, main_land_attr}},
       {'S', {PosType::SHIP_SHOP, main_sea_attr}},
       {'B', {PosType::BERTH, main_sea_land_attr}},
       {'K', {PosType::DOCK, main_sea_attr}},
       {'C', {PosType::SEA_LAND, sea_land_attr}},
       {'c', {PosType::MAIN_SEA_LAND, main_sea_land_attr}},
       {'T', {PosType::DELIVERY, main_sea_attr}}};

  std::random_device rd;
  std::vector<std::pair<int, int>> directions = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}};

public:
  std::array<Point, 10> robots_first_pos; // 机器人初始位置

  Point find_random_barrier() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 199);
    Point pos;
    do {
      pos.x = dis(gen);
      pos.y = dis(gen);
    } while (map[pos.x][pos.y] != '#');
    return pos;
  }

  void init_robot_pos() {
    int cnt = 0;
    for (int i = 0; i < 200; i++) {
      for (int j = 0; j < 200; j++) {
        if (map[i][j] == 'A') {
          robots_first_pos[cnt++] = Point(i, j);
        }
      }
    }

    for (int i = 0; i < 10; i++) {
      log_raw("robot %d pos:(%d,%d)\n", i, robots_first_pos[i].x,
              robots_first_pos[i].y);
    }
  }

  PosType get_pos_type(const Point &pos) {
    if (is_valid_pos(pos)) {
      // if (lookuptable.find(map[pos.x][pos.y]) != lookuptable.end()) {
      return lookuptable.at(map[pos.x][pos.y]).first;
      // }
    }
    return PosType::BARRIER;
  }
  bool is_dock_pos(const Point &pos) {
    return get_pos_type(pos) == PosType::DOCK || is_berth_pos(pos);
  }
  bool is_berth_pos(const Point &pos) {
    return get_pos_type(pos) == PosType::BERTH;
  }

  bool is_valid_pos(const Point &pos) {
    return pos.x >= 0 && pos.x < 200 && pos.y >= 0 && pos.y < 200;
  }
  bool is_barrier_for_robot(const Point &pos) {
    if (is_valid_pos(pos)) {
      return lookuptable.at(map[pos.x][pos.y]).second.is_barrier_for_robot;
    }
    return true;
  }
  bool is_barrier_for_ship(const Point &pos) {
    if (is_valid_pos(pos)) {
      return lookuptable.at(map[pos.x][pos.y]).second.is_barrier_for_ship;
    }
    return true;
  }

  bool has_collision_effect_for_robot(const Point &pos) {
    if (is_valid_pos(pos)) {
      return lookuptable.at(map[pos.x][pos.y])
          .second.has_collison_effect_for_robot;
    }
    return true;
  }
  bool has_collision_effect_for_ship(const Point &pos) {
    if (is_valid_pos(pos)) {
      return lookuptable.at(map[pos.x][pos.y])
          .second.has_collison_effect_for_ship;
    }
    return true;
  }

  void rand_neighber_again() {
    std::shuffle(directions.begin(), directions.end(), rd);
  }

  std::vector<Point> neighbors_for_ship(const Point &pos) {
    std::vector<Point> result;
    for (int i = 0; i < 4; i++) {
      const int nx = pos.x + directions[i].first;
      const int ny = pos.y + directions[i].second;
      if (is_valid_pos(pos) && !is_barrier_for_ship(pos)) {
        result.emplace_back(nx, ny);
      }
    }

    if (std::rand() % 1024 < 200) {
      rand_neighber_again();
    }

    return result;
  }

  std::vector<Point> neighbors_for_robot(const Point &pos) {
    std::vector<Point> result;
    for (int i = 0; i < 4; i++) {
      const int nx = pos.x + directions[i].first;
      const int ny = pos.y + directions[i].second;
      if (is_valid_pos(pos) && !is_barrier_for_robot(pos)) {
        result.emplace_back(nx, ny);
      }
    }

    if (std::rand() % 1024 < 100) {
      rand_neighber_again();
    }

    return result;
  }


  void write_line(const char *buf, int x) {
    for (int i = 0; i < 200; i++) {
      map[x][i] = buf[i];
    }
  }
  void print_map() {
    for (int i = 0; i < 200; i++) {
      for (int j = 0; j < 200; j++) {
        log_raw("%c", map[i][j]);
      }
      log_raw("\n");
    }
  }

  void print_map_with_point() {
    for (int i = 0; i < 200; i++) {
      for (int j = 0; j < 200; j++) {
        log_raw("(%d,%d)%c", i, j, map[i][j]);
      }
      log_raw("\n");
    }
  }
  void print_map_line(int x) {
    for (int i = 0; i < 200; i++) {
      log_raw("%c", map[x][i]);
    }
    log_raw("line:%d\n", x);
  }

  explicit GameMap() = default;
  ~GameMap() = default;
};