#pragma once

#include "io_laye_new.hpp"
#include "log.h"
#include "path_helper.hpp"
#include "point.hpp"
#include <functional>
#include <optional>
#include <vector>

#include "tree_map.hpp"

class RobotCollisionAvoid {
  IoLayerNew* io_layer = nullptr;
  // first: 子节点 high_priority_robot_id, second: 父节点 low_priority_robot_id
  std::vector<std::pair<int, int>> collision_pair_list{};

public:
  explicit RobotCollisionAvoid(IoLayerNew* io_layer) : io_layer(io_layer) {}
  ~RobotCollisionAvoid() = default;

  void check_collision_pair() {
    // key: 子节点 high_priority_robot_id, val: 父节点 low_priority_robot_id
    std::unordered_map<int, int> collision_pair_filter{};
    for (const auto collision_pair : collision_pair_list) {
      if (Point::is_stop_point(io_layer->robots.at(collision_pair.second).get_next_pos())) {
        collision_pair_filter[collision_pair.first] = collision_pair.second;
      }
    }
    auto tree_map = TreeMap(collision_pair_filter);
    int cur_priority = 0;
    tree_map.trees_bfs_for_each([&](const TreeMap::TreeNode* node) {
        io_layer->robots.at(node->id).priority = cur_priority++;
        log_trace("robot_id[%d] priority change to %d", node->id, io_layer->robots.at(node->id).priority);
      }
    );
    collision_pair_list.clear();
  }

  /**
   * @brief 高优先级机器人的下一步位置是否与低优先级机器人当前位置发生碰撞
   *
   * @param robot_id 高优先级机器人id
   * @return std::optional<int>  低优先级机器人id
   */
  std::optional<int> high_next_pos2low_cur_pos_collision(const int robot_id) const {
    const Point& robot_next_pos = io_layer->robots.at(robot_id).get_next_pos();

    if (Point::is_stop_point(robot_next_pos)) {
      return std::nullopt;
    }
    if (!io_layer->game_map.has_collision_effect_for_robot(robot_next_pos)) {
      return std::nullopt;
    }

    for (const auto& r : io_layer->robots) {
      if (r.id == robot_id || r.has_pass_collision_check) {
        // 1. 不需要与自己检查
        // 2. 只和优先级低的机器人检查
        continue;
      }
      if (robot_next_pos == r.pos) {
        return r.id;
      }
    }
    return std::nullopt;
  }

  /**
   * @brief 高优先级机器人的下一步位置是否与低优先级机器人的下一步位置发生碰撞
   *
   * @param robot_id
   * @return std::optional<int>
   */
  std::optional<int> high_next_pos2low_next_pos_collision(const int robot_id) const {
    const Point& robot_next_pos = io_layer->robots.at(robot_id).get_next_pos();
    if (Point::is_stop_point(robot_next_pos)) {
      return std::nullopt;
    }
    if (!io_layer->game_map.has_collision_effect_for_robot(robot_next_pos)) {
      return std::nullopt;
    }
    for (auto& r : io_layer->robots) {
      if (r.id == robot_id || r.has_pass_collision_check) {
        // 1. 不需要与自己检查
        // 2. 只和优先级低的机器人检查
        continue;
      }
      if (robot_next_pos == r.get_next_pos()) {
        return r.id;
      }
    }
    return std::nullopt;
  }

  /**
   * @brief 低优先级机器人的下一步位置是否与高优先级机器人当前位置发生碰撞
   *
   * @param robot_id
   * @return std::optional<int>
   */
  std::optional<int> low_next_pos2high_cur_pos_collision(const int robot_id) const {
    const Point& robot_next_pos = io_layer->robots.at(robot_id).get_next_pos();
    if (Point::is_stop_point(robot_next_pos)) {
      return std::nullopt;
    }
    if (!io_layer->game_map.has_collision_effect_for_robot(robot_next_pos)) {
      return std::nullopt;
    }
    for (auto& r : io_layer->robots) {
      if (r.id == robot_id || !r.has_pass_collision_check) {
        // 1. 不需要与自己检查
        // 2. 只和优先级高的机器人检查
        continue;
      }
      if (robot_next_pos == r.pos) {
        return r.id;
      }
    }
    return std::nullopt;
  }

  /**
   * @brief 低优先级机器人的下一步位置是否与高优先级机器人的下一步位置发生碰撞
   *
   * @param robot_id
   * @return std::optional<int>
   */
  std::optional<int> low_next_pos2high_next_pos_collision(const int robot_id) const {
    const Point& robot_next_pos = io_layer->robots.at(robot_id).get_next_pos();
    if (Point::is_stop_point(robot_next_pos)) {
      return std::nullopt;
    }
    if (!io_layer->game_map.has_collision_effect_for_robot(robot_next_pos)) {
      return std::nullopt;
    }
    for (auto& r : io_layer->robots) {
      if (r.id == robot_id || !r.has_pass_collision_check) {
        // 1. 不需要与自己检查
        // 2. 只和优先级高的机器人检查
        continue;
      }
      if (robot_next_pos == r.get_next_pos()) {
        return r.id;
      }
    }
    return std::nullopt;
  }


  void collision_avoid_step1(const int robot_id) {
    const auto& robot_next_pos_before =
      io_layer->robots.at(robot_id).get_next_pos();
    const auto& robot_cur_pos = io_layer->robots.at(robot_id).pos;
    auto& cur_robot = io_layer->robots.at(robot_id);

    cur_robot.has_pass_collision_check = true;
    if (Point::is_stop_point(robot_next_pos_before)) {
      log_trace("robot_id:%d next_pos is stop_point,no need collision_avoid ",
                robot_id);
      return;
    }
    log_trace("robot_id:%d before collision_avoid_step1, cur_pos(%d,%d) "
              "next_pos:(%d,%d)",
              robot_id, P_ARG(robot_cur_pos), P_ARG(robot_next_pos_before));

    // 1. 低优先级机器人的下一步位置是否与高优先级机器人的下一步位置发生碰撞
    const auto low_next_pos2high_next_pos_collision_id =
      low_next_pos2high_next_pos_collision(robot_id);

    if (low_next_pos2high_next_pos_collision_id.has_value()) {
      // 1. 低优先级机器人需要让出位置
      log_trace("robot_id:%d low_next_pos2high_next_pos_collision_id:%d",
                robot_id, low_next_pos2high_next_pos_collision_id.value());
      cur_robot.collision_cycle++;
      bool cut_success = false;
      auto cut_come_from = PATHHelper::cut_path(
        robot_cur_pos, io_layer->get_is_barrier_for_robot_lambda(robot_id, true, true, false),
        io_layer->get_find_neighbor_for_robot_lambda(), cur_robot.path_list, 20, cut_success);
      if (cut_success) {
        log_trace("robot_id:%d cut_success", robot_id);
        cur_robot.set_final_next_pos(cur_robot.path_list.back());
      }
      else {
        log_trace("robot_id:%d cut_failed", robot_id);
        std::vector<Point> avoid_points{};
        avoid_points.push_back(
          io_layer->robots.at(low_next_pos2high_next_pos_collision_id.value())
                  .get_next_pos());
        auto fail_callback = [&] {
          cur_robot.set_final_next_pos(invalid_point);
        };

        fallback_to_bt_path(robot_id, cut_come_from, avoid_points,
                            fail_callback);
      }
    }

    // 2. 低优先级机器人的下一步位置是否与高优先级机器人当前位置发生碰撞
    auto low_next_pos2high_cur_pos_collision_id =
      low_next_pos2high_cur_pos_collision(robot_id);
    if (low_next_pos2high_cur_pos_collision_id.has_value()) {
      cur_robot.collision_cycle++;
      // 2. 低优先级机器人需要让出位置
      log_trace("robot_id:%d low_next_pos2high_cur_pos_collision_id:%d",
                robot_id, low_next_pos2high_cur_pos_collision_id.value());

      bool cut_success = false;
      auto cut_come_from = PATHHelper::cut_path(
        robot_cur_pos, io_layer->get_is_barrier_for_robot_lambda(robot_id, true, true, false),
        io_layer->get_find_neighbor_for_robot_lambda(), cur_robot.path_list, 20, cut_success);
      if (cut_success) {
        log_trace("robot_id:%d cut_success", robot_id);
        cur_robot.set_final_next_pos(cur_robot.path_list.back());
      }
      else {
        std::vector<Point> avoid_points{};
        avoid_points.push_back(
          io_layer->robots.at(low_next_pos2high_cur_pos_collision_id.value())
                  .pos);
        auto fail_callback = [&] {
          cur_robot.set_final_next_pos(invalid_point);
        };

        fallback_to_bt_path(robot_id, cut_come_from, avoid_points,
                            fail_callback);
      }
    }

    // 3. 高优先级机器人的下一步位置是否与低优先级机器人当前位置发生碰撞
    auto high_next_pos2low_cur_pos_collision_id =
      high_next_pos2low_cur_pos_collision(robot_id);
    if (high_next_pos2low_cur_pos_collision_id.has_value()) {
      log_trace("robot_id:%d high_next_pos2low_cur_pos_collision_id:%d",
                robot_id, high_next_pos2low_cur_pos_collision_id.value());
      cur_robot.collision_cycle++;
      collision_pair_list.emplace_back(robot_id, high_next_pos2low_cur_pos_collision_id.value());
      // 停止移动
      cur_robot.set_final_next_pos(invalid_point);
    }

    // 4. 高优先级机器人的下一步位置是否与低优先级机器人的下一步位置发生碰撞
    const auto high_next_pos2low_next_pos_collision_id =
      high_next_pos2low_next_pos_collision(robot_id);
    if (high_next_pos2low_next_pos_collision_id.has_value()) {
      // 继续执行,不做处理
      log_trace("robot_id:%d high_next_pos2low_next_pos_collision_id:%d",
                robot_id, high_next_pos2low_next_pos_collision_id.value());
    }

    const auto& robot_next_pos_after =
      io_layer->robots.at(robot_id).get_next_pos();
    log_trace("robot_id:%d after collision_avoid_step1, cur_pos(%d,%d) "
              "next_pos:(%d,%d)",
              robot_id, P_ARG(robot_cur_pos), P_ARG(robot_next_pos_after));
  }

  bool
  fallback_to_bt_path(const int robot_id,
                      const std::unordered_map<Point, PointCost>& come_from,
                      const std::vector<Point>& avoid_points,
                      const std::function<void()>& fail_callback) const {
    auto& cur_robot = io_layer->robots.at(robot_id);
    const auto bt_point = PATHHelper::get_bt_point(
      cur_robot.pos, come_from, io_layer->get_is_barrier_for_robot_lambda(robot_id, true, true, false),
      avoid_points);
    if (!bt_point.has_value()) {
      log_trace("robot_id:%d fallback_to_bt_path failed", robot_id);
      fail_callback();
      return false;
    }
    log_trace("robot_id:%d fallback_to_bt_path success", robot_id);

    bool bt_path_found = false;
    auto bt_path = PATHHelper::get_path(cur_robot.pos, bt_point.value(),
                                        come_from, bt_path_found);
    PATHHelper::add_backtrace_path(cur_robot.pos, cur_robot.path_list, bt_path);

    log_assert(bt_path_found, "robot_id:%d get_path failed", robot_id);
    cur_robot.set_final_next_pos(cur_robot.path_list.back());

    return true;
  }
};
