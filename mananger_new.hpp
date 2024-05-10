#pragma once

#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "config.h"
#include "goods.hpp"
#include "io_laye_new.hpp"
#include "log.h"
#include "point.hpp"
#include "robot.hpp"
#include "robot_collision_avoid.hpp"
#include "robot_control.hpp"
#include "ship.hpp"
#include "ship_control.hpp"

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
        // if (robot_id == 0) {

        //     return quality_strategy;
        // }
        static auto dis_strategy = [](const RobotControl::GoodsInfo &goods_info) -> int {
            return RobotControl::goods_strategy_distance_first(goods_info, false);
        };
        static auto quality_strategy = [](const RobotControl::GoodsInfo &goods_info) -> int {
            return RobotControl::goods_strategy_quality_first(goods_info, false);
        };
        static auto time_strategy = [](const RobotControl::GoodsInfo& goods_info) -> int {
            return RobotControl::goods_strategy_remain_time_first(goods_info, true);
        };


        
        if(io_layer.remain_cycle() < 1200){
            return quality_strategy;
        }
        return quality_strategy;
    }

    void goods_list_cycle() {
        // 将新货物添加到货物列表中
        for (auto& i : io_layer.new_goods_list) {
            const auto [_, mini_cost] = io_layer.get_minimum_berth_cost_for_robot(i.pos);
            // 无法到达的货物不添加到货物列表中
            if (mini_cost > 2000) {
                continue;
            }
            io_layer.map_goods_list[i.pos] = i;
            io_layer.map_goods_list[i.pos].to_near_berth_cost = mini_cost;
            io_layer.statistic.add_total_goods(io_layer.map_goods_list[i.pos]);
            log_assert(mini_cost == io_layer.map_goods_list[i.pos].to_near_berth_cost, "mini_cost:%d", mini_cost);
            log_assert(i.pos != invalid_point, "invalid goods");
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
        for (auto it = io_layer.map_goods_list.begin(); it != io_layer.map_goods_list.end();) {
            if (it->second.end_cycle < io_layer.cur_cycle || it->second.status == GoodsStatus::Got ||
                it->second.status == GoodsStatus::Dead) {
                it = io_layer.map_goods_list.erase(it);
            }
            else {
                if (update_goods_info) {
                    for (int i = 0; i < io_layer.berths.size(); i++) {
                        auto& cur_berth = io_layer.berths[i];
                        // auto cur_cost = io_layer.get_cost_from_berth_to_point(i,
                        // it->first);
                        auto cur_cost = io_layer.berths_come_from_for_robot[i].get_point_cost(it->first);

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

    void buy_robot_and_ship() {
        static int max_robot_num = io_layer.max_robot_num();
        static int max_ship_num = io_layer.calc_max_ship_num();
#if SCRIPT_MODE == 1
        max_robot_num = MAX_ROBOT_NUM;
        max_ship_num = MAX_SHIP_NUM;
#endif
        // if(io_layer.robot_shops_come_from.front().map_size() == 32552){
        // max_robot_num = 15;
        //     max_ship_num = 1;
        // }else if(io_layer.robot_shops_come_from.front().map_size() == 31518) {
        //  max_robot_num = 13;
        //     max_ship_num = 2;
        // }else{
        //     max_robot_num = 18;
        //     max_ship_num = 1;
        // }
 
        log_assert(io_layer.robots.size() <= max_robot_num, "robot num:%d", io_layer.robots.size());
        const int max_big_robot_num = 0;
        const int max_small_robot_num = max_robot_num - max_big_robot_num*2;
        const int max_all_robot_num = max_big_robot_num + max_small_robot_num;


        if (io_layer.robots.size() < max_small_robot_num && io_layer.cur_money > io_layer.robot_price) {
            const auto rand_robot_shop = io_layer.robot_shops.at(Tools::random(0ul, io_layer.robot_shops.size() - 1));
            io_layer.robot_lbot(rand_robot_shop,0);
            io_layer.last_robot_good_num = 1;
            if (io_layer.robots.size() + 1 == max_robot_num) {
                log_info("robot max num [%d] at cycle :%d", io_layer.robots.size()+1, io_layer.cur_cycle);
            }
        } else if (io_layer.robots.size() >= max_small_robot_num&&io_layer.robots.size() < max_all_robot_num && io_layer.cur_money >= 5000) {
            const auto rand_robot_shop = io_layer.robot_shops.at(Tools::random(0ul, io_layer.robot_shops.size() - 1));
            io_layer.robot_lbot(rand_robot_shop,1);
            io_layer.last_robot_good_num = 2;
            if (io_layer.robots.size() + 1 == max_robot_num) {
                log_info("robot max num [%d] at cycle :%d", io_layer.robots.size()+1, io_layer.cur_cycle);
            }
        }



        if (io_layer.cur_cycle == 1) {
            io_layer.ship_lboat(io_layer.ship_shops.back());
        }
        if (io_layer.ships.size() < max_ship_num && io_layer.cur_money > io_layer.ship_price &&
            io_layer.robots.size() == max_all_robot_num) {
            // if (io_layer.statistic.goted_goods_count() - io_layer.statistic.selled_goods_count() >
            //     io_layer.ship_capacity * (io_layer.ships.size() + 2)) {
                const auto rand_ship_shop = io_layer.ship_shops.at(Tools::random(0ul, io_layer.ship_shops.size() - 1));
                io_layer.ship_lboat(rand_ship_shop);
                if (io_layer.ships.size() == max_ship_num) {
                    log_info("ship max num at[%d] cycle :%d", io_layer.ships.size(), io_layer.cur_cycle);
                }
            // }
        }
    }

    bool robot_is_baned(const int robot_id) {
#ifdef TEST_MODE
        constexpr int base_line = 2400;
        constexpr int step = 20;
        const int baned_id = (io_layer.statistic.total_goods_count() - base_line) / step;

        if (robot_id == baned_id) {
            io_layer.robots[robot_id].priority = 0;
            return true;
        }
#endif
        return false;
    }

    void berth_cycle() {
        for (auto & ship : io_layer.ships) {
            for (auto& berth : io_layer.berths) {
                if(ship.in_berth_point_id_list(berth.id)){
                    continue;
                }
                if(io_layer.berth_had_other_ship(ship.id, berth.id)){
                    continue;
                }
                if (berth.max_robot_num == 0) {
                    continue;
                }
                if(io_layer.is_last_berth(berth.id)){
                    continue;
                }

            
                const auto ship_trap_info = io_layer.get_ship_trap_info(ship);

                const auto final_delivery_pos = io_layer.delivery_points.at(ship_trap_info.target_delivery_id);
                const auto to_berth_cost = io_layer.berths_come_from_for_ship[berth.id].get_point_cost(final_delivery_pos);
                if (!to_berth_cost.has_value()) {
                    continue;
                }
                const auto [_, to_delivery_cost] = io_layer.get_minimum_delivery_point_cost_for_ship(berth.pos);

                const int cur_berth_trap_cost = ship_trap_info.remained_cycle+to_berth_cost.value()+to_delivery_cost;
                if (io_layer.remain_cycle() < cur_berth_trap_cost+30) {
                    log_info("cycle[%d] berth[%d] will not arrive at next time, should close", io_layer.cur_cycle,
                             berth.id);
                    io_layer.reassign_robot_to_berth(berth.max_robot_num);
                             berth.max_robot_num = 0;
                }
            }
        }
    }



    void run_game() {
        for (int zhen = 1; zhen <= 15000; zhen++) {
            io_layer.input_cycle();

            // 更新货物信息
            goods_list_cycle();
            berth_cycle();
            buy_robot_and_ship();
            for (auto& robot : io_layer.robots) {
                robot.clear_flags();
            }


            for (auto& robot : io_layer.robots) {
                if (robot_is_baned(robot.id)) {
                    continue;
                }
                robot_control.robot_get_goods(robot);
                robot_control.find_new_goods(robot, get_goods_strategy_lambda(robot.id));
                robot_control.find_second_goods(robot);
                robot_control.go_near_berth(robot);
            }

            // 创建节点索引数组
            std::vector<size_t> indices(io_layer.robots.size());
            for (size_t i = 0; i < io_layer.robots.size(); ++i) {
                indices[i] = i;
            }

            // 按照节点值的大小顺序排序索引数组，而不改变原始节点的顺序
            std::sort(indices.begin(), indices.end(),
                      [&](const size_t a, const size_t b) {
                          return io_layer.robots[a].priority < io_layer.robots[b].priority;
                      });

            for (const size_t i : indices) {
                auto& robot = io_layer.robots[i];
                if (robot_is_baned(robot.id)) {
                    continue;
                }
                robot_collision_avoid.collision_avoid_step1(robot.id);
            }
            robot_collision_avoid.check_collision_pair();


            for (auto& robot : io_layer.robots) {
                if (robot_is_baned(robot.id)) {
                    continue;
                }
                robot_control.robots_move(robot);
                robot_control.robots_pull_cycle(robot);
            }

            for (auto& ship : io_layer.ships) {
                ship.clear_flags();
            }
            for (auto& ship : io_layer.ships) {
                if (io_layer.cur_cycle < io_layer.ship_capacity*10 || ship.can_operate() == false) {
                    continue;
                }
                ship_control.sell_goods_and_new_transport(ship);
                ship_control.must_go_to_delivery(ship);
                // log_assert(ship.cur_capacity == ship.goods_list.size(),
                //            "ship.cur_capacity:%d ship.goods_list.size():%d",
                //            ship.cur_capacity, ship.goods_list.size());
                // log_info("ship.cur_capacity:%d ship.goods_list.size():%d",
                //          ship.cur_capacity, ship.goods_list.size());
                ship_control.ship_control_fsm(ship);
                ship_control.drop_path_point_if_reach(ship);
                ship_control.must_go_to_delivery(ship);
                ship_control.ship_loading(ship);
                ship_control.go_to_berth(ship);
                ship_control.go_to_deliver(ship);
                ship_control.update_next_pos(ship);
                ship_control.output_command(ship);
            }
            check_robot_collision();
            io_layer.print_goods_info();
            io_layer.output_cycle();
            if (abs(io_layer.cur_cycle) == 15000) {
                io_layer.print_final_info();
                break;
            }
        }
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
            const bool cur_pos_has_collision_effect = io_layer.game_map.has_collision_effect_for_robot(cur_pos);
            const bool next_pos_has_collision_effect = io_layer.game_map.has_collision_effect_for_robot(next_pos);
            const bool cur_pos_is_stop = Point::is_stop_point(cur_pos);
            const bool next_pos_is_stop = Point::is_stop_point(next_pos);

            if (cur_pos_has_collision_effect && !cur_pos_is_stop) {
                if (points_set.find(cur_pos) != points_set.end()) {
                    has_collision = true;
                    log_info("(%d,%d)type:%d,%c\n", P_ARG(cur_pos), io_layer.game_map.get_pos_type(cur_pos),
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
                    log_info("(%d,%d)type:%d,%c\n", P_ARG(next_pos), io_layer.game_map.get_pos_type(next_pos),
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
                log_info("robot:%d pos:(%d,%d) next_pos:(%d,%d)", robot.id, robot.pos.x, robot.pos.y,
                         robot.get_next_pos().x, robot.get_next_pos().y);
            }
        }

        log_assert(!has_collision, "has_collision");
    }
};
