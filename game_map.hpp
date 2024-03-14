#pragma once

// This is the GameMap class. It is responsible for all game map operations.

#include "log.h"
#include "point.hpp"
#include <vector>
class GameMap {

private:
  char map[200][200] = {'#'};
  // ‘.’ : 空地
  // ‘*’ : 海洋
  // ‘#’ : 障碍
  // ‘A’ : 机器人起始位置,总共 10 个。
  // ‘B’ : 大小为 4*4,表示泊位的位置,泊位标号在后泊位处初始化。
  enum PosType { SPACE = 0, OCEAN = 1, ROBOT = 2, BARRIER = 3, BERTH = 4 };

public:
  PosType get_pos_type(Point &pos) {
    if (is_valid_pos(pos)) {
      switch (map[pos.x][pos.y]) {
      case '.':
        return PosType::SPACE;
      case '*':
        return PosType::OCEAN;
      case 'A':
        return PosType::ROBOT;
      case '#':
        return PosType::BARRIER;
      case 'B':
        return PosType::BERTH;
      default:
        return PosType::BARRIER;
      }
    }
    return PosType::BARRIER;
  }

  bool is_valid_pos(Point &pos) {
    return pos.x >= 0 && pos.x < 200 && pos.y >= 0 && pos.y < 200;
  }
  bool is_barrier(Point &pos) {
    PosType type = get_pos_type(pos);
    return type == PosType::BARRIER || type == PosType::OCEAN;
  }
  bool is_space(Point &pos) { return get_pos_type(pos) == PosType::SPACE; }

  std::vector<Point> neighbors(Point &pos) {
    std::vector<Point> result;
    constexpr int dx[4] = {1, 0, -1, 0};
    constexpr int dy[4] = {0, 1, 0, -1};
    for (int i = 0; i < 4; i++) {
      int nx = pos.x + dx[i];
      int ny = pos.y + dy[i];
      if (is_valid_pos(pos) && !is_barrier(pos)) {
        result.push_back(Point(nx, ny));
      }
    }
    return result;
  }
  void write_line(char *buf, int x) {
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

  explicit GameMap() {

  }
  ~GameMap() {};
};