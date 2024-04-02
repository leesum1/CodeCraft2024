#pragma once

#include "io_laye_new.hpp"
#include "log.h"
#include "point.hpp"
#include "ship.hpp"
#include <cassert>
#include <optional>

class ShipControl {

  IoLayerNew *io_layer = nullptr;

public:
  ShipControl(IoLayerNew *io_layer) : io_layer(io_layer) {}
  ~ShipControl() = default;

  auto get_is_barrier_lambda(const int ship_id) {

    auto is_barrier_func = [this, ship_id](const Point &p) {
      bool is_barrier1 = io_layer->game_map.is_barrier_for_ship(p);
      bool is_barrier2 = false;
      bool is_barrier3 = false;

      if (!io_layer->game_map.has_collison_effect_for_robot(p)) {
        return is_barrier1;
      }

      for (auto &s : io_layer->ships) {
        if (s.id == ship_id) {
          continue;
        }

        auto s_cur_area = s.get_ship_area();
        auto s_next_area = s.get_ship_next_area();
        if (s_cur_area.contain(p)) {
          is_barrier2 = true;
        }
        if (s_next_area.has_value() && s_next_area.value().contain(p)) {
          is_barrier3 = true;
        }
      }
      return is_barrier1 || is_barrier2 || is_barrier3;
    };
    return is_barrier_func;
  }

  auto get_find_neighbor_lambda(IoLayerNew &io_layer) {
    io_layer.game_map.rand_neighber_again();
    auto find_neighbor = [&](const Point &p) {
      return io_layer.game_map.neighbors_for_ship(p);
    };
    return find_neighbor;
  }

  /**
   * @brief 找到最有价值的泊位
   *
   * @param ship
   * @return std::optional<int>
   */
  std::optional<int> get_rich_berth_id() {
    std::optional<int> best_berth_id = std::nullopt;
    int max_value = -1;
    for (auto &berth : io_layer->berths) {
      if (berth.goods_value() > max_value) {
        max_value = berth.goods_value();
        best_berth_id = berth.id;
      }
    }
    return best_berth_id;
  }

  int get_best_deliver_id() { return 0; }

  // bool update_ship_next_pos() {
  // Area head_area;
  // auto ship_head = get_ship_head();
  // if (ship_head[0].x <= ship_head[1].x && ship_head[0].y <= ship_head[1].y) {
  //   head_area = Area(ship_head[0], ship_head[1]);
  // } else {
  //   head_area = Area(ship_head[1], ship_head[0]);
  // }
  // log_assert(head_area.valid(), "head_area is invalid");
  //
  //// 下一个点相对于当前船头的方向
  // const auto pos_dir = Direction::calc_direction(head_area, path.back());
  //
  // if (pos_dir == this->direction) {
  //  // 前进
  //} else if (pos_dir == Direction::opposite(this->direction)) {
  //  // 随便找一个方向旋转
  //} else {
  //  // 旋转
  //  const auto rot_dir =
  //      Direction::calc_rotate_direction(this->direction, pos_dir);
  //
  //  const auto [next_pos, next_dir] = calc_rot_action(
  //      this->pos, this->direction, rot_dir.value() == Direction::CLOCKWISE);
  //
  //  this->next_pos = next_pos;
  //  this->next_direction = next_dir;
  //}

  // const auto rot_ret =  calc_rot_action(const Point &pos, const
  // Direction::Direction dir, bool clockwise_direction)
  //}

  void go_to_berth(Ship &ship) {
    if (!ship.normal_status()) {
      return;
    }
    if (ship.fsm != ShipFSM::GO_TO_BERTH) {
      return;
    }
    if (ship.path.size() > 0) {
      ship.update_ship_next_pos();
      return;
    }

    auto rich_berth_id = get_rich_berth_id();

    if (!rich_berth_id.has_value()) {
      log_info("ship[%d] no rich berth", ship.id);
      return;
    }

    auto ship_head = ship.get_ship_head();
    auto &berth_come_from =
        io_layer->berths_come_from_for_ship.at(rich_berth_id.value());

    bool berth_path_founded = false;
    auto berth_path = berth_come_from.get_path_from_point(ship_head.left_top,
                                                          berth_path_founded);

    if (!berth_path_founded) {
      log_info("ship[%d] no berth[%d] path founded", ship.id,
               rich_berth_id.value());
      return;
    }
    log_debug("ship_area:%s", ship_head.to_string().c_str());
    for (auto &p : berth_path) {
      log_info("ship[%d] berth_path(%d,%d)", ship.id, P_ARG(p));
    }

    ship.path = berth_path;
    ship.update_ship_next_pos();
  }

  void go_to_deliver(Ship &ship) {
    if (!ship.normal_status()) {
      return;
    }
    if (ship.fsm != ShipFSM::GO_TO_DELIVERY) {
      return;
    }

    if (ship.path.size() > 0) {
      ship.update_ship_next_pos();
      return;
    }

    auto best_deliver_id = get_best_deliver_id();

    const auto ship_head = ship.get_ship_head();

    bool deliver_path_founded = false;
    auto deliver_path =
        io_layer->delivery_point_come_from.at(best_deliver_id)
            .get_path_from_point(ship_head.left_top, deliver_path_founded);

    if (!deliver_path_founded) {
      log_info("ship[%d] no berth[%d] path founded", ship.id, best_deliver_id);
      return;
    }

    ship.path = deliver_path;
    ship.update_ship_next_pos();
  }

  void ouput_command(Ship &ship) {
    if (!ship.normal_status()) {
      return;
    }

    switch (ship.get_next_command()) {
    case ShipCommand::IDLE: {
      break;
    }
    case ShipCommand::GO: {
      io_layer->ship_move(ship.id);
      break;
    }
    case ShipCommand::ROTATE_CLOCKWISE: {
      io_layer->ship_rot(ship.id, 0);
      break;
    }
    case ShipCommand::ROTATE_COUNTERCLOCKWISE: {
      io_layer->ship_rot(ship.id, 1);
      break;
    }
    }

    auto ship_next_area = ship.get_ship_next_area();

    if (ship_next_area.has_value()) {
      while (ship.path.size() > 0) {
        Point p = ship.path.back();
        if (ship_next_area.value().contain(p) == false) {
          break;
        } else {
          log_info("ship[%d] pop path.back(%d,%d)", ship.id, P_ARG(p));
          ship.path.pop_back();
        }
      }
    }

    if (ship.path.size() == 0) {
      switch (ship.fsm) {
      case ShipFSM::GO_TO_BERTH: {
        ship.fsm = ShipFSM::GO_TO_DELIVERY;
        log_trace("ship[%d] fsm change to GO_TO_DELIVERY", ship.id);
        break;
      }
      case ShipFSM::GO_TO_DELIVERY: {
        ship.fsm = ShipFSM::GO_TO_BERTH;
        log_trace("ship[%d] fsm change to GO_TO_BERTH", ship.id);
        break;
      }
      }
    }
  }
};
