#pragma once

// This is the GameMap class. It is responsible for all game map operations.

#include "point.h"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

class GameMap {

private:
  char *map = nullptr;
  bool map_is_owned = false;
  // ‘.’ : 空地
  // ‘*’ : 海洋
  // ‘#’ : 障碍
  // ‘A’ : 机器人起始位置,总共 10 个。
  // ‘B’ : 大小为 4*4,表示泊位的位置,泊位标号在后泊位处初始化。
  enum PosType { SPACE = 0, OCEAN = 1, ROBOT = 2, BARRIER = 3, BERTH = 4 };

public:
  std::optional<PosType> get_pos_type(uint64_t x, uint64_t y);
  bool is_valid_pos(uint64_t x, uint64_t y);
  bool is_valid_type(char val);
  bool is_barrier(uint64_t x, uint64_t y) {
    auto pos_type = this->get_pos_type(x, y);
    if (!pos_type.has_value()) {
      return false;
    }
    return pos_type.value() == PosType::BARRIER ||
           pos_type.value() == PosType::OCEAN;
  }
  bool is_space(uint64_t x, uint64_t y) { return !this->is_barrier(x, y); }

  uint64_t width;
  uint64_t height;

  explicit GameMap(uint64_t width, uint64_t height);
  explicit GameMap(int width, int height, char *map);
  explicit GameMap();
  ~GameMap();
  void write_pos(uint64_t x, uint64_t y, char val);
  void print_map();
  char *get_map_ptr();
  void set_map(char *map);
  std::vector<Point> neighbors(Point pos);
};