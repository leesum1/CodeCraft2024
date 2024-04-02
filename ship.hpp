#pragma once

#include "direction.hpp"
#include "log.h"
#include "point.hpp"
#include <array>
#include <optional>
#include <utility>

enum class ShipFSM {
  GO_TO_BERTH,    // 前往泊位
  GO_TO_DELIVERY, // 前往交货点
};

enum class ShipCommand {
  IDLE,                    // 空闲
  GO,                      // 前进
  ROTATE_CLOCKWISE,        // 顺时针旋转
  ROTATE_COUNTERCLOCKWISE, // 逆时针旋转

};

class Ship {

public:
  int id;
  int capacity;
  // 正常行驶状态（状态0）恢复状态（状态1）装载状态（状态2)
  int status;
  int berth_id;
  Point pos; // 当前中心位置
  Direction::Direction direction;

  // 规划的路径
  std::vector<Point> path{};
  int target_berth_id = -1; // 目标泊位id
  ShipFSM fsm = ShipFSM::GO_TO_BERTH;

  // 碰撞检测前计算出来的信息
  Point next_pos_before_collison = invalid_point;
  Direction::Direction next_direction_before_collison = Direction::UP;
  ShipCommand next_command_before_collison = ShipCommand::IDLE;

  // 碰撞检测后计算出来的信息
  Point next_pos_after_collison = invalid_point;
  Direction::Direction next_direction_after_collison = Direction::UP;
  ShipCommand next_command_after_collison = ShipCommand::IDLE;

  // 一些状态位置
  int cur_capacity = 0; // 当前载重量
  int cur_value = 0;    // 当前价值

  void clear_flags() {
    this->next_pos_before_collison = invalid_point;
    this->next_direction_before_collison = Direction::UP;
    this->next_command_before_collison = ShipCommand::IDLE;

    this->next_pos_after_collison = invalid_point;
    this->next_direction_after_collison = Direction::UP;
    this->next_command_after_collison = ShipCommand::IDLE;
  }

  Point get_next_pos() {
    if (this->next_pos_after_collison == invalid_point) {
      return this->pos;
    }
    return this->next_pos_after_collison;
  }
  Direction::Direction get_next_direction() {
    if (this->next_pos_after_collison == invalid_point) {
      return this->direction;
    }
    return this->next_direction_after_collison;
  }
  ShipCommand get_next_command() {
    if (this->next_pos_after_collison == invalid_point) {
      return ShipCommand::IDLE;
    }
    return this->next_command_after_collison;
  }

  void update_ship_next_pos() {

    log_assert(path.size() > 0, "path is empty");

    auto ship_head = get_ship_head();

    // 下一个点相对于当前船头的方向
    const auto pos_dir = Direction::calc_direction(ship_head, path.back());

    auto update_func = [&](const Point &next_pos,
                           const Direction::Direction &next_dir,
                           const ShipCommand &next_command) {
      this->next_pos_after_collison = next_pos;
      this->next_direction_after_collison = next_dir;
      this->next_command_after_collison = next_command;
    };

    if (pos_dir == this->direction) {
      // 1.方向相同,前进

      update_func(Direction::move(this->pos, this->direction), this->direction,
                  ShipCommand::GO);

    } else if (pos_dir == Direction::opposite(this->direction)) {
      // 2. 方向相反,随便找一个方向旋转
      bool clockwise = rand() % 2 == 0;

      const auto [next_pos, next_dir] =
          calc_rot_action(this->pos, this->direction, clockwise);
      update_func(next_pos, next_dir,
                  clockwise ? ShipCommand::ROTATE_CLOCKWISE
                            : ShipCommand::ROTATE_COUNTERCLOCKWISE);

    } else {
      // 3. 方向相差 90 度,旋转
      const auto rot_dir =
          Direction::calc_rotate_direction(this->direction, pos_dir);

      const auto [next_pos, next_dir] = calc_rot_action(
          this->pos, this->direction, rot_dir.value() == Direction::CLOCKWISE);

      update_func(next_pos, next_dir,
                  rot_dir.value() == Direction::CLOCKWISE
                      ? ShipCommand::ROTATE_CLOCKWISE
                      : ShipCommand::ROTATE_COUNTERCLOCKWISE);
    }
  }

  int capacity_percent() { return this->cur_capacity * 100 / this->capacity; }

  bool normal_status() { return this->status == 0; }
  bool recover_status() { return this->status == 1; }
  bool load_status() { return this->status == 2; }

  /**
   * @brief 得到船头的位置
   *
   * @return
   */
  Area get_ship_head() { return calc_ship_head(this->pos, this->direction); }
  /**
   * @brief 得到下一个位置的船头位置,如果下一个位置是停止点,返回std::nullopt
   *
   * @return
   */
  std::optional<Area> get_ship_next_head() {
    const auto next_pos = this->get_next_pos();
    const auto next_dir = this->get_next_direction();
    if (Point::is_stop_point(next_pos)) {
      return std::nullopt;
    }
    return calc_ship_head(next_pos, next_dir);
  }

  /**
   * @brief 计算船头的位置
   *
   * @param pos 船中心点位置
   * @param dir 船朝向
   * @return Area
   */
  static Area calc_ship_head(const Point &pos, const Direction::Direction dir) {

    std::array<Point, 2> head_pos{};
    Area head_area;
    switch (dir) {
    case Direction::RIGHT: {
      head_pos = {Point(pos.x, pos.y + 2), Point(pos.x + 1, pos.y + 2)};
      break;
    }
    case Direction::LEFT: {
      head_pos = {Point(pos.x, pos.y - 2), Point(pos.x - 1, pos.y - 2)};
      break;
    }
    case Direction::UP: {
      head_pos = {Point(pos.x - 2, pos.y), Point(pos.x - 2, pos.y + 1)};
      break;
    }
    case Direction::DOWN:
      head_pos = {Point(pos.x + 2, pos.y), Point(pos.x + 2, pos.y - 1)};
      break;
    }

    if (head_pos[0].x <= head_pos[1].x && head_pos[0].y <= head_pos[1].y) {
      head_area = Area(head_pos[0], head_pos[1]);
    } else {
      head_area = Area(head_pos[1], head_pos[0]);
    }
    log_assert(head_area.valid(), "head_area is invalid");
    log_assert(Point::is_adjacent(head_pos[0], head_pos[1]),
               "head_pos is not adjacent,(%d,%d),(%d,%d)", P_ARG(head_pos[0]),
               P_ARG(head_pos[1]));

    return head_area;
  }

  /**
   * @brief 得到船的区域
   *
   * @return Area
   */
  Area get_ship_area() { return calc_ship_area(this->pos, this->direction); }

  /**
   * @brief 得到下一个位置的区域,如果下一个位置是停止点,返回std::nullopt
   *
   * @return std::optional<Area>
   */
  std::optional<Area> get_ship_next_area() {
    const auto next_pos = this->get_next_pos();
    const auto next_dir = this->get_next_direction();

    if (Point::is_stop_point(next_pos)) {
      return std::nullopt;
    }
    return calc_ship_area(next_pos, next_dir);
  }

  /**
   * @brief 计算旋转后船的区域位置
   *
   * @param pos
   * @param dir
   * @return Area
   */
  static Area calc_ship_area(const Point &pos, Direction::Direction dir) {
    switch (dir) {
    case Direction::RIGHT: {
      return Area(pos, Point(pos.x + 1, pos.y + 2));
      break;
    }
    case Direction::LEFT: {
      return Area({pos.x - 1, pos.y - 2}, pos);
      break;
    }
    case Direction::UP: {
      return Area({pos.x - 2, pos.y}, {pos.x, pos.y + 1});
      break;
    }
    case Direction::DOWN:
      return Area({pos.x, pos.y - 1}, {pos.x + 2, pos.y});
      break;
    }

    log_assert(false, "get_ship_area error");
  }

  /**
   * @brief 计算旋转后的位置和方向
   *
   * @param pos 当前位置
   * @param dir 当前方向
   * @param clockwise_direction 是否顺时针旋转
   * @return std::pair<Point, Direction::Direction> 旋转后的位置和方向
   */
  static std::pair<Point, Direction::Direction>
  calc_rot_action(const Point &pos, const Direction::Direction dir,
                  bool clockwise_direction) {
    switch (dir) {
    case Direction::RIGHT: {
      if (clockwise_direction) {
        return std::make_pair(Point(pos.x, pos.y + 2), Direction::DOWN);
      } else {
        return std::make_pair(Point(pos.x + 1, pos.y + 1), Direction::UP);
      }
      break;
    }
    case Direction::LEFT: {
      if (clockwise_direction) {
        return std::make_pair(Point(pos.x, pos.y - 2), Direction::UP);
      } else {
        return std::make_pair(Point(pos.x - 1, pos.y - 1), Direction::DOWN);
      }
      break;
    }
    case Direction::UP: {
      if (clockwise_direction) {
        return std::make_pair(Point(pos.x - 2, pos.y), Direction::RIGHT);
      } else {
        return std::make_pair(Point(pos.x - 1, pos.y + 1), Direction::LEFT);
      }
      break;
    }
    case Direction::DOWN:
      if (clockwise_direction) {
        return std::make_pair(Point(pos.x + 2, pos.y), Direction::LEFT);
      } else {
        return std::make_pair(Point(pos.x + 1, pos.y - 1), Direction::RIGHT);
      }
      break;
    }

    log_assert(false, "calc_rot_action error");
    return std::make_pair(Point(-1, -1), Direction::UP);
  }

  void load(int value) {
    log_assert(value > 0 && value <= 200, "value is not positive, %d", value);
    this->cur_capacity++;
    this->cur_value += value;
  }
  void unload() {
    log_assert(this->cur_capacity >= 0, "cur_capacity is not positive, %d",
               this->cur_capacity);
    log_assert(this->cur_value >= 0, "cur_value is not positive %d",
               this->cur_value);

    log_assert(this->berth_id == -1, "berth_id is not -1, %d", this->berth_id);

    log_trace("Ship %d unload, cur_capacity: %d, cur_value: %d", this->id,
              this->capacity_percent(), this->cur_value);
    this->cur_capacity = 0;
    this->cur_value = 0;
  }
  bool full() { return this->cur_capacity >= this->capacity; }

  explicit Ship(int id, int cur_capacity, int max_capacity, const Point &pos,
                Direction::Direction direction, int status)
      : id(id), cur_capacity(cur_capacity), capacity(max_capacity),
        status(status), pos(pos), direction(direction) {}

  explicit Ship() {
    this->id = 0;
    this->capacity = 0;
    this->status = 0;
    this->berth_id = -1;
    this->cur_capacity = 0;
    this->cur_value = 0;
  }
};
