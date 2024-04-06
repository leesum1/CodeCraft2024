#pragma once

#include "goods.hpp"
#include "io_laye_new.hpp"
#include "log.h"
#include "point.hpp"
#include "robot.hpp"
#include "robot_collision_avoid.hpp"
#include "robot_control.hpp"
#include "ship.hpp"
#include "ship_control.hpp"
#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

class ManagerNew {
public:
    IoLayerNew io_layer;
    RobotCollisionAvoid robot_collision_avoid{&io_layer};
    ShipControl ship_control{&io_layer};
    RobotControl robot_control{&io_layer};

    ManagerNew() = default;

    ~ManagerNew() = default;

    void init_game() { io_layer.init(); }


    std::function<int(const RobotControl::GoodsInfo&)> get_goods_strategy_lambda(const int robot_id) const {
        // if (robot_id < io_layer.robots.size() / 2) {
        //     auto time_strategy = [](const RobotControl::GoodsInfo& goods_info) -> int {
        //         return RobotControl::goods_strategy_remain_time_first(goods_info, true);
        //     };
        //     return time_strategy;
        // }
        //
        // auto quality_strategy = [](const RobotControl::GoodsInfo& goods_info) -> int {
        //     return RobotControl::goods_strategy_quality_first(goods_info, true);
        // };
        // return quality_strategy;


        auto time_strategy = [](const RobotControl::GoodsInfo& goods_info) -> int {
            return RobotControl::goods_strategy_remain_time_first(goods_info, true);
        };
        return time_strategy;
    }

    void goods_list_cycle() {
        // 将新货物添加到货物列表中
        for (auto& i : io_layer.new_goods_list) {
            io_layer.map_goods_list[i.pos] =
                i;
            log_assert(i.pos != invalid_point,
                       "invalid goods");
        }

        const bool update_goods_info = io_layer.cur_cycle % 10 == 0;
        // const bool update_goods_info = false;

        if (update_goods_info) {
            for (auto& berth : io_layer.berths) {
                berth.clear_goods_info();
                berth.tmp_baned = false;
            }
        }

        // 删除 goods_list 中已经消失的货物
        for (auto it = io_layer.map_goods_list.begin();
             it != io_layer.map_goods_list.end();) {
            if (it->second.end_cycle < io_layer.cur_cycle &&
                it->second.status != GoodsStatus::Got) {
                it = io_layer.map_goods_list.erase(it);
            }
            else {
                if (update_goods_info) {
                    for (int i = 0; i < io_layer.berths.size(); i++) {
                        auto& cur_berth = io_layer.berths[i];
                        // auto cur_cost = io_layer.get_cost_from_berth_to_point(i,
                        // it->first);
                        auto cur_cost =
                            io_layer.berths_come_from_for_robot[i].get_point_cost(
                                it->first);

                        if (!cur_cost.has_value()) {
                            continue;
                        }
                        auto minimum_cost = io_layer.get_minimum_berth_cost_2(it->first);

                        if (minimum_cost.first != i) {
                            continue;
                        }

                        // if (cur_cost.value() <= 120 &&
                        //     it->second.money >= io_layer.total_goods_avg_money()) {
                        //   cur_berth.near_goods_num++;
                        //   cur_berth.near_goods_value += it->second.money;
                        //   cur_berth.near_goods_distance += cur_cost.value();
                        // }
                    }
                }

                ++it;
            }
        }
        log_info("map_goods_list size:%d", io_layer.map_goods_list.size());

        // if (update_goods_info) {

        //   std::vector<std::pair<int, int>> berths_sort;
        //   for (int i = 0; i < BERTH_NUM; i++) {
        //     berths_sort.emplace_back(i, io_layer.berths[i].goods_num());
        //   }
        //   std::sort(berths_sort.begin(), berths_sort.end(),
        //             [&](const auto &p1, const auto &p2) {
        //               return p1.second < p2.second;
        //             });

        //   for (int i = 0; i < 10; i++) {
        //     io_layer.berths[i].tmp_baned = false;
        //   }

        //   std::for_each_n(berths_sort.begin(), 2, [&](const auto &p) {
        //     log_info("berth[%d] goods_num:%d", p.first, p.second);
        //     io_layer.berths[p.first].tmp_baned = true;
        //   });
        // }

        // for (int i = 0; i < BERTH_NUM; i++) {
        //   io_layer.berths[i].printf_near_goods_info();

        //   // // 动态禁用港口可能还是副作用
        //   if (update_goods_info) {
        //     if (io_layer.cur_cycle > 1000) {
        //       if (io_layer.berths[i].near_goods_num < 6) {

        //         if (!io_layer.berths[i].tmp_baned) {
        //           bool maybe_baned = true;
        //           io_layer.berths[i].tmp_baned = maybe_baned;
        //         }
        //       } else {
        //         io_layer.berths[i].tmp_baned = false;
        //       }
        //     }
        //   }

        //   log_info("berth[%d] is_baned:%d", i, io_layer.berth_is_baned(i));
        // }
    }

    void run_game() {
        for (int zhen = 1; zhen <= 15000; zhen++) {
            io_layer.input_cycle();

            // 更新货物信息
            goods_list_cycle();

            const int expect_robot_num = 18;
            const int expect_ship_num = 1;

            if (io_layer.robots.size() < expect_robot_num && io_layer.cur_money > io_layer.robot_price) {
                io_layer.robot_lbot(io_layer.robot_shops.front());
            }
            if (zhen == 1) {
                io_layer.ship_lboat(io_layer.ship_shops.back());
            }
            if (io_layer.ships.size() < expect_ship_num && io_layer.cur_money > io_layer.ship_price && io_layer.robots.
                size() == expect_robot_num) {
                io_layer.ship_lboat(io_layer.ship_shops.front());
            }


            for (auto& robot : io_layer.robots) {
                robot.clear_flags();
            }


            auto start = std::chrono::high_resolution_clock::now();
            for (auto& robot : io_layer.robots) {
                robot_control.robot_get_goods(robot);
                robot_control.find_new_goods(robot, get_goods_strategy_lambda(robot.id));
                robot_control.go_near_berth(robot);
            }
            auto end = std::chrono::high_resolution_clock::now();
            log_info(
                "robot move time:%d",
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                .count());
            start = std::chrono::high_resolution_clock::now();

            for (auto& robot : io_layer.robots) {
                robot_collision_avoid.collision_avoid_step1(robot.id);
            }
            end = std::chrono::high_resolution_clock::now();
            log_info(
                "collision avoid step1 time:%d",
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                .count());

            for (auto& robot : io_layer.robots) {
                robot_control.robots_move(robot);
                robot_control.robots_pull_cycle(robot);
            }

            for (auto& ship : io_layer.ships) {
                ship.clear_flags();
            }
            for (auto& ship : io_layer.ships) {
                ship_control.sell_goods_and_new_transport(ship);
                // log_assert(ship.cur_capacity == ship.goods_list.size(),
                //            "ship.cur_capacity:%d ship.goods_list.size():%d",
                //            ship.cur_capacity, ship.goods_list.size());
                // log_info("ship.cur_capacity:%d ship.goods_list.size():%d",
                //          ship.cur_capacity, ship.goods_list.size());
                ship_control.ship_control_fsm(ship);
                ship_control.drop_path_point_if_reach(ship);
                ship_control.ship_loading(ship);
                ship_control.go_to_berth(ship);
                ship_control.go_to_deliver(ship);
                ship_control.update_next_pos(ship);
                ship_control.output_command(ship);
            }


            io_layer.output_cycle();
            log_info("map_goods_list size:%d", io_layer.map_goods_list.size());
            io_layer.print_goods_info();

            check_robot_collision();
            if (abs(io_layer.cur_cycle) == 15000) {
                break;
            }
        }
        io_layer.print_final_info();
    }

    /**
     * @brief 检查机器人是否发生碰撞
     */
    void check_robot_collision() {
        std::unordered_set<Point> points_set{};
        bool has_collision = false;
        for (auto& robot : io_layer.robots) {
            const auto& cur_pos = robot.pos;
            const auto& next_pos = robot.get_next_pos();
            const bool cur_pos_has_collision_effect =
                io_layer.game_map.has_collision_effect_for_robot(cur_pos);
            const bool next_pos_has_collision_effect =
                io_layer.game_map.has_collision_effect_for_robot(next_pos);
            const bool cur_pos_is_stop = Point::is_stop_point(cur_pos);
            const bool next_pos_is_stop = Point::is_stop_point(next_pos);

            if (cur_pos_has_collision_effect && !cur_pos_is_stop) {
                if (points_set.find(cur_pos) != points_set.end()) {
                    has_collision = true;
                    log_info("(%d,%d)type:%d,%c\n", P_ARG(cur_pos),
                             io_layer.game_map.get_pos_type(cur_pos),
                             io_layer.game_map.map[cur_pos.x][cur_pos.y]);
                    break;
                }
                else {
                    points_set.insert(cur_pos);
                }
            }
            if (next_pos_has_collision_effect && !next_pos_is_stop) {
                if (points_set.find(next_pos) != points_set.end()) {
                    has_collision = true;
                    log_info("(%d,%d)type:%d,%c\n", P_ARG(next_pos),
                             io_layer.game_map.get_pos_type(next_pos),
                             io_layer.game_map.map[next_pos.x][next_pos.y]);
                    break;
                }
                else {
                    points_set.insert(next_pos);
                }
            }
        }

        if (has_collision) {
            for (auto& robot : io_layer.robots) {
                log_info("robot:%d pos:(%d,%d) next_pos:(%d,%d)", robot.id, robot.pos.x,
                         robot.pos.y, robot.get_next_pos().x, robot.get_next_pos().y);
            }
        }

        log_assert(!has_collision, "has_collision");
    }
};
