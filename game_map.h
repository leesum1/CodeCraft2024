#pragma once

// This is the GameMap class. It is responsible for all game map operations.
#include <cstdint>
#include <optional>

class GameMap {

private:
  uint64_t width;
  uint64_t height;
  char *map = nullptr;
  // ‘.’ : 空地
  // ‘*’ : 海洋
  // ‘#’ : 障碍
  // ‘A’ : 机器人起始位置,总共 10 个。
  // ‘B’ : 大小为 4*4,表示泊位的位置,泊位标号在后泊位处初始化。
  enum PosType { SPACE = 0, OCEAN = 1, ROBOT = 2, BARRIER = 3, BERTH = 4 };

  std::optional<PosType> get_pos_type(uint64_t x, uint64_t y);
  bool is_valid_pos(uint64_t x, uint64_t y);
  bool is_valid_type(char val);

public:
  explicit GameMap(uint64_t width, uint64_t height);
  ~GameMap();
  void write_pos(uint64_t x, uint64_t y, char val);
  void print_map();
  void set_map(char *map);
};