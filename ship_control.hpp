#pragma once

#include "io_laye_new.hpp"
#include "log.h"
#include "point.hpp"
#include "ship.hpp"
#include <optional>

class ShipControl {

  IoLayerNew *io_layer = nullptr;

public:
  ShipControl(IoLayerNew *io_layer) : io_layer(io_layer) {}
  ~ShipControl() = default;

  auto get_is_barrier_lambda(const int ship_id, bool care_other_ship_cur_pos,
                             bool care_other_ship_next_pos) {

    auto is_barrier_func = [this, ship_id, care_other_ship_cur_pos,
                            care_other_ship_next_pos](const Point &p) {
      bool is_barrier1 = io_layer->game_map.is_barrier_for_ship(p);
      bool is_barrier2 = false;
      bool is_barrier3 = false;

      // 如果没有碰撞效果，直接返回
      if (!io_layer->game_map.has_collison_effect_for_robot(p)) {
        return is_barrier1;
      }

      // 如果不关心其他船的位置，直接返回
      if (!care_other_ship_cur_pos && !care_other_ship_next_pos) {
        return is_barrier1;
      };

      for (auto &s : io_layer->ships) {
        if (s.id == ship_id) {
          continue;
        }

        auto s_cur_area = s.get_ship_area();
        auto s_next_area = s.get_ship_next_area();

        if (care_other_ship_cur_pos && s_cur_area.contain(p)) {
          is_barrier2 = true;
        }
        if (care_other_ship_next_pos && s_next_area.has_value() &&
            s_next_area.value().contain(p)) {
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

  int get_random_berth_id() {
    int max_berth_id = io_layer->berths.size() - 1;
    return std::rand() % max_berth_id;
  }

  int get_best_deliver_id() { return 0; }

  int get_random_deliver_id() {
    int max_deliver_id = io_layer->delivery_points.size() - 1;
    // return std::rand() % max_deliver_id;
    return 1;
  }

  void go_to_berth(Ship &ship) {
    if (!ship.normal_status()) {
      return;
    }
    if (ship.fsm != ShipFSM::GO_TO_BERTH) {
      return;
    }
    if (ship.path.size() > 0) {
      ship.update_ship_next_pos(get_is_barrier_lambda(ship.id, false, false));
      return;
    }

    auto rich_berth_id = get_random_berth_id();



    auto ship_head = ship.get_ship_head();
    auto &berth_come_from =
        io_layer->berths_come_from_for_ship.at(rich_berth_id);

    bool berth_path_founded = false;
    auto berth_path = berth_come_from.get_path_from_point(ship_head.left_top,
                                                          berth_path_founded);

    if (!berth_path_founded) {
      log_info("ship[%d] no berth[%d] path founded", ship.id,
               rich_berth_id);
      return;
    }
    log_debug("ship_area:%s", ship_head.to_string().c_str());
    for (auto &p : berth_path) {
      log_info("ship[%d] berth_path(%d,%d)", ship.id, P_ARG(p));
    }

    ship.path = berth_path;
    drop_path_point_if_reach(ship);
    ship.update_ship_next_pos(get_is_barrier_lambda(ship.id, false, false));
  }

  void go_to_deliver(Ship &ship) {
    if (!ship.normal_status()) {
      return;
    }
    if (ship.fsm != ShipFSM::GO_TO_DELIVERY) {
      return;
    }

    if (ship.path.size() > 0) {
      ship.update_ship_next_pos(get_is_barrier_lambda(ship.id, false, false));
      return;
    }

    auto best_deliver_id = get_random_deliver_id();

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
    drop_path_point_if_reach(ship);
    ship.update_ship_next_pos(get_is_barrier_lambda(ship.id, false, false));
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

    drop_path_point_if_reach(ship);

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

  void drop_path_point_if_reach(Ship &ship) {

    auto ship_cur_area = ship.get_ship_area();

    while (ship.path.size() > 0) {
      Point p = ship.path.back();
      if (ship_cur_area.contain(p) == false) {
        break;
      } else {
        log_info("ship[%d] cur_area reach, pop path.back(%d,%d)", ship.id,
                 P_ARG(p));
        ship.path.pop_back();
      }
    }

    auto ship_next_area = ship.get_ship_next_area();
    if (!ship_next_area.has_value()) {
      return;
    }

    while (ship.path.size() > 0) {
      Point p = ship.path.back();
      if (ship_next_area.value().contain(p) == false) {
        break;
      } else {
        log_info("ship[%d] next_area reach pop path.back(%d,%d)", ship.id,
                 P_ARG(p));
        ship.path.pop_back();
      }
    }
  }
};
