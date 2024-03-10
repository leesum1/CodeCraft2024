
#include "game_map.h"
#include "log.h"

/**
 * @brief Constructs a GameMap object with the specified width and height.
 *
 * @param width The width of the game map.
 * @param height The height of the game map.
 */
GameMap::GameMap(uint64_t width, uint64_t height) {
  this->width = width;
  this->height = height;
  this->map = new char[width * height];
}

GameMap::~GameMap() {
  if (this->map != nullptr) {
    delete[] this->map;
  }
}

void GameMap::print_map() {
  if (this->map == nullptr) {
    log_fatal("Map is not initialized");
    return;
  }

  for (uint64_t i = 0; i < this->height; i++) {
    for (uint64_t j = 0; j < this->width; j++) {
      log_raw("%c", this->map[i * this->width + j]);
    }
    log_raw("\n");
  }
}

void GameMap::set_map(char *map) {
  if (this->map != nullptr) {
    delete[] this->map;
  }
  this->map = map;
}

void GameMap::write_pos(uint64_t x, uint64_t y, char val) {
  if (!this->is_valid_pos(x, y)) {
    return;
  }
  if (!this->is_valid_type(val)) {
    return;
  }
  if (this->map == nullptr) {
    log_fatal("Map is not initialized");
    return;
  }

  this->map[y * this->width + x] = val;
}

/**
 * Checks if a given character is a valid type for the game map.
 *
 * @param val The character to be checked.
 * @return True if the character is a valid type, false otherwise.
 */
bool GameMap::is_valid_type(char val) {
  switch (val) {
  case '.':
  case '*':
  case '#':
  case 'A':
  case 'B':
    return true;
  default:
    log_fatal("Invalid character: %c", val);
    return false;
  }
}

/**
 * Checks if the given position is valid within the game map.
 *
 * @param x The x-coordinate of the position.
 * @param y The y-coordinate of the position.
 * @return True if the position is valid, false otherwise.
 */
bool GameMap::is_valid_pos(uint64_t x, uint64_t y) {

  if (this->map == nullptr) {
    log_fatal("Map is not initialized");
    return false;
  }

  if (x < 0 || x >= this->width || y < 0 || y >= this->height) {
    log_fatal("Invalid position: x=%d, y=%d in width:%d,hight:%d", x, y,
              this->width, this->height);
    return false;
  }

  return true;
}

/**
 * @brief Retrieves the position type at the specified coordinates.
 *
 * @param x The x-coordinate of the position.
 * @param y The y-coordinate of the position.
 * @return An optional value representing the position type. If the coordinates
 * are invalid or the character at the position is invalid, std::nullopt is
 * returned.
 */
std::optional<GameMap::PosType> GameMap::get_pos_type(uint64_t x, uint64_t y) {
  if (!this->is_valid_pos(x, y)) {
    return std::nullopt;
  }

  char c = this->map[y * this->width + x];
  switch (c) {
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
    log_fatal("Invalid character: %c", c);
    return std::nullopt;
  }
}