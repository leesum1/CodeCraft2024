#pragma once

#include <algorithm>
#include "berth.hpp"
#include "come_from_map.hpp"
#include "direction.hpp"
#include "game_map.hpp"
#include "goods.hpp"
#include "log.h"
#include "path_helper.hpp"
#include "point.hpp"
#include "robot.hpp"
#include "ship.hpp"
#include "statistic.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "tools.hpp"

class IoLayerNew {
private:
    enum CommandInst {
        // 机器人指令
        ROBOT_MOVE,
        ROBOT_GET,
        ROBOT_PULL,
        ROBOT_LBOT,
        // 船只指令
        SHIP_MOVE,
        SHIP_ROT,
        SHIP_LBOAT,
        SHIP_DEPT,
        SHIP_BERTH
    };

    struct Command {
        CommandInst inst;
        int arg1;
        int arg2;
    };

public:
    GameMap game_map; // 初始地图
    std::unordered_map<Point, Goods> map_goods_list;

    /* 每帧更新的信息*/
    std::vector<Berth> berths{}; // 靠泊点信息
    std::vector<Ship> ships{}; // 船只信息
    std::vector<Robot> robots{}; // 机器人信息
    std::vector<Goods> new_goods_list{}; // 新增货物信息

    const int robot_price = 2000; // 机器人价格
    const int ship_price = 8000; // 船只价格
    std::vector<Point> robot_shops{}; // 机器人商店
    std::vector<Point> ship_shops{}; // 船只商店
    std::vector<Point> delivery_points{}; // 交货点

    const int delivery_id_offset = 10;
    std::vector<std::vector<int>> delivery_points_berth_loop{};

    Statistic statistic{};

    int ship_capacity = 0;
    int berth_area_size_in_55_cost = 0; // 距离港口 55 距离以内的区域数量
    int berth_avg_delivery_point_cost = 0; // 港口到交货点的平均距离

    int new_goods_num = 0; // 新增货物数量
    int cur_cycle = 0; // 当前周期
    int cur_money = 0; // 当前金钱

    bool final_time = false; // 到最后的时间后,机器人之往指定的港口运输货物

    std::vector<ComeFromMap> berths_come_from_for_robot{};
    std::vector<ComeFromMap> berths_come_from_for_ship{};
    std::vector<ComeFromMap> robot_shops_come_from{};
    std::vector<ComeFromMap> delivery_point_come_from{};

    int total_goods_num = 0; // 总货物数量
    int total_goods_money = 0; // 总货物价值
    // int goted_goods_num = 0;    // 已经获取的货物数量
    // int goted_goods_money = 0;  // 已经获取的货物价值
    // int selled_goods_num = 0;   // 已经卖出的货物数量
    // int selled_goods_money = 0; // 已经卖出的货物价值

    /* 每帧的输出指令 */
    std::vector<Command> commands;

    explicit IoLayerNew() = default;

    ~IoLayerNew() = default;

    int remain_cycle() const { return 15000 - cur_cycle; }

    /**
     * @brief 港口是否可用
     * @param berth_id
     * @return
     */
    bool berth_is_ocuppied(const int berth_id) const {
        for (const auto& ship : ships) {
            if (std::any_of(ship.berth_point_id.begin(), ship.berth_point_id.end(), [&](const int v) {
                return v == berth_id;
            })) {
                return true;
            }

            if (ship.target_berth_id == berth_id) {
                return true;
            }
        }
        return false;
    }


    /**
     * @brief 计算船只最大数量
     * @return
     */
    int calc_max_ship_num() const {
        const double ship_max_num = berth_avg_delivery_point_cost / (ship_capacity * 1.4);
        return std::max(1, static_cast<int>(ship_max_num));
    }

    /**
     * @brief 计算机器人最大数量
     * @return
     */
    int max_robot_num() const {
        const double area_rate = static_cast<double>(berth_area_size_in_55_cost) / game_map.land_size();
        int robot_num = 12;
        if (area_rate > 0.65) {
            // 65% 以上
            robot_num += 0;
        }
        else if (area_rate > 0.55) {
            // 55%-65%
            robot_num += 1;
        }
        else if (area_rate > 0.45) {
            // 45%-55%
            robot_num += 3;
        }
        else if (area_rate > 0.30) {
            // 30%-45%
            robot_num += 6;
        }
        else {
            // 30% 以下
            robot_num += 7;
        }
        if (calc_max_ship_num() > 1) {
            robot_num += 1;
        }
        return robot_num;
    }

    /**
     * @brief 计算当前港口最大机器人数量
     * @param berth_id
     * @return
     */
    int calc_max_robots_in_berth(const int berth_id) const {
        const double robot_percent = static_cast<double>(berths.at(berth_id).near_points.size()) /
            berths_come_from_for_robot.
            at(berth_id).map_size();
        const int cur_berth_max_robot_num = std::max(static_cast<int>(robot_percent * max_robot_num()), 1) + 1;
        return cur_berth_max_robot_num;
    }

    /**
     * @brief 获取港口机器人的当前数量（当前机器人除外）
     * @param berth_id
     * @param robot_id
     * @return
     */
    int get_berth_robot_num(const int berth_id, const int robot_id) const {
        int num = 0;
        for (auto& robot : robots) {
            if (robot.id == robot_id) {
                continue;
            }
            if (robot.target_berth_id == berth_id) {
                num++;
            }
        }
        return num;
    }

    /**
     * @brief 获取港口船只的当前数量（当前船只除外）
     * @param ship_id
     * @param berth_id
     * @return
     */
    bool berth_had_other_ship(const int ship_id, const int berth_id) const {
        for (auto& ship : ships) {
            if (ship.id == ship_id) {
                continue;
            }
            if (ship.target_berth_id == berth_id) {
                return true;
            }
        }

        return false;
    }

    int calc_delivery_point_id(const int id) const {
        const int id_by_offset = id - delivery_id_offset;
        log_assert(id_by_offset < delivery_points.size(), "delivery_points size:%d, cur_id:%d", delivery_points.size(),
                   id_by_offset);
        return id_by_offset;
    }

    std::vector<int> get_last_rich_berth_list(const Ship& ship) {
        std::vector<int> berths_id{};

        for (int i = 0; i < berths.size(); i++) {
            if (berth_had_other_ship(ship.id, i)) {
                continue;
            }
            berths_id.push_back(i);
        }


        std::vector<int> best_id_list{};
        int best_weight = -1;
        int best_cost = 999999;
        while (std::next_permutation(berths_id.begin(), berths_id.end())) {
            int remine_capacity = ship.capacity; // 剩余容量
            int total_value = 0; // 总价值
            int total_cost = 0; // 总花费
            int select_weight = -1; // 总花费
            int select_cost = 999999;
            Point cur_pos = ship.pos; // 当前位置
            std::vector<int> cur_berths_id_list{};
            for (const auto& berth_id : berths_id) {
                // 获取是不是最后一次循环
                const auto iter_dis =
                    std::distance(berths_id.begin(), std::find(berths_id.begin(), berths_id.end(), berth_id));

                auto& cur_berth = berths[berth_id];
                const auto [goods_num, goods_value] = cur_berth.goods_first_n(remine_capacity);
                const int goods_cost = cur_berth.get_load_cost(goods_num);
                // 去往当前港口的花费
                const int to_berth_cost = berths_come_from_for_ship[berth_id].get_point_cost(cur_pos).value_or(20000);
                // 更新当前位置为当前港口位置
                cur_pos = cur_berth.pos;
                // 从当前港口到交货点的花费
                const int to_delivery_cost = get_minimum_delivery_point_cost_for_ship(cur_pos).second;
                // 减去当前港口的货物
                remine_capacity -= goods_num;
                // 更新总价值和总花费
                total_value += goods_value;
                total_cost = total_cost + to_berth_cost + goods_cost;

                // 假设从当前港口直接去交货点，计算性价比
                const int cur_cost = total_cost + to_delivery_cost + 1;
                const int cur_weight_tmp = total_value * 100;
                const bool have_enough_time = (cur_cost + 10) < remain_cycle();

                if (have_enough_time && (cur_weight_tmp >= select_weight)) {
                    if (cur_cost < select_cost) {
                        select_cost = cur_cost;
                        select_weight = cur_weight_tmp;
                        cur_berths_id_list = Tools::first_n(berths_id, iter_dis + 1);
                    }
                }
            }

            if (select_weight >= best_weight) {
                if (select_cost < best_cost) {
                    best_cost = select_cost;
                    best_weight = select_weight;
                    best_id_list = cur_berths_id_list;
                }
            }
        }

        if (best_id_list.empty()) {
            log_info("best_id_list is empty");
            return {};
        }
        return best_id_list;
    }

    /**
     * @brief 计算船最优的遍历港口顺序
     * @param ship
     * @return
     */
    std::vector<int> get_fit_berth_list(const Ship& ship) {
        std::vector<int> berths_id{};

        for (int i = 0; i < berths.size(); i++) {
            if (berth_had_other_ship(ship.id, i)) {
                continue;
            }
            berths_id.push_back(i);
        }
        const int remain_time = remain_cycle();
        std::vector<int> best_id_list{};
        double best_weight = -1.0;
        while (std::next_permutation(berths_id.begin(), berths_id.end())) {
            int remine_capacity = ship.capacity; // 剩余容量
            int total_value = 0; // 总价值
            int total_cost = 0; // 总花费
            double select_weight = -1; // 总花费
            Point cur_pos = ship.pos; // 当前位置
            std::vector<int> cur_berths_id_list{};
            for (const auto& berth_id : berths_id) {
                // 获取是不是最后一次循环
                const auto iter_dis =
                    std::distance(berths_id.begin(), std::find(berths_id.begin(), berths_id.end(), berth_id));

                auto& cur_berth = berths[berth_id];
                const auto [goods_num, goods_value] = cur_berth.goods_first_n(remine_capacity);
                const int goods_cost = cur_berth.get_load_cost(goods_num);
                // 去往当前港口的花费
                const int to_berth_cost = berths_come_from_for_ship[berth_id].get_point_cost(cur_pos).value_or(20000);
                // 更新当前位置为当前港口位置
                cur_pos = cur_berth.pos;
                // 从当前港口到交货点的花费
                const int to_delivery_cost = get_minimum_delivery_point_cost_for_ship(cur_pos).second;
                // 减去当前港口的货物
                remine_capacity -= goods_num;
                // 更新总价值和总花费
                total_value += goods_value;
                total_cost = total_cost + to_berth_cost + goods_cost;

                // 假设从当前港口直接去交货点，计算性价比
                const int cur_cost = total_cost + to_delivery_cost + 1;
                const double cur_weight_tmp = static_cast<double>(total_value * 100) / cur_cost;
                const bool have_enough_time = (cur_cost + 15) < remain_time;

                if (have_enough_time && (cur_weight_tmp > select_weight)) {
                    select_weight = cur_weight_tmp;
                    cur_berths_id_list = Tools::first_n(berths_id, iter_dis + 1);
                }
            }

            if (select_weight > best_weight) {
                best_weight = select_weight;
                best_id_list = cur_berths_id_list;
            }
        }


        if (best_id_list.empty()) {
            log_info("best_id_list is empty");
            int rand_berth_id = Tools::random(0ul, berths.size() - 1);
            return {rand_berth_id};
        }

        return best_id_list;
    }

    std::vector<int> get_best_berth_list(const Ship& ship) {
        // if (cur_cycle < 1000 || remain_cycle() < 500) {
        //     return get_last_rich_berth_list(ship);
        // }
        return get_fit_berth_list(ship);
    }


    std::vector<int> get_best_berth_list_v2(const Ship& ship) {
        std::vector<int> berths_id{};

        for (int i = 0; i < berths.size(); i++) {
            if (berth_had_other_ship(ship.id, i) || ship.target_berth_id == i) {
                continue;
            }
            berths_id.push_back(i);
        }
        const int cur_ship_value = ship.cur_value();
        const int cur_ship_cost = cur_cycle - ship.start_cycle;
        const int cur_ship_capacity = ship.remain_capacity();
        // 计算当前船的性价比，从当前位置直接去交货点的性价比
        double best_weight = static_cast<double>(cur_ship_value * 100) / (1 + cur_ship_cost +
            get_minimum_delivery_point_cost_for_ship(ship.pos).
            second);
        std::vector<int> best_id_list{};
        best_weight = -1.0;


        while (std::next_permutation(berths_id.begin(), berths_id.end())) {
            int remine_capacity = cur_ship_capacity; // 剩余容量
            int total_value = cur_ship_value; // 总价值
            int total_cost = cur_ship_cost; // 总花费
            double select_weight = -1; // 总花费
            Point cur_pos = ship.pos; // 当前位置
            std::vector<int> cur_berths_id_list{};
            for (const auto& berth_id : berths_id) {
                // 获取是不是最后一次循环
                const auto iter_dis = std::distance(berths_id.begin(),
                                                    std::find(berths_id.begin(), berths_id.end(), berth_id));

                auto& cur_berth = berths[berth_id];
                const auto [goods_num,goods_value] = cur_berth.goods_first_n(remine_capacity);
                const int goods_cost = cur_berth.get_load_cost(goods_num);
                // 去往当前港口的花费
                const int to_berth_cost =
                    berths_come_from_for_ship[berth_id].get_point_cost(cur_pos).value_or(20000);
                // 更新当前位置为当前港口位置
                cur_pos = cur_berth.pos;
                // 从当前港口到交货点的花费
                const int to_delivery_cost = get_minimum_delivery_point_cost_for_ship(cur_pos).second;
                // 减去当前港口的货物
                remine_capacity -= goods_num;
                // 更新总价值和总花费
                total_value += goods_value;
                total_cost = total_cost + to_berth_cost + goods_cost;

                // 假设从当前港口直接去交货点，计算性价比
                const int cur_cost = total_cost + to_delivery_cost + 1;
                const double cur_weight_tmp = static_cast<double>(total_value * 100) / cur_cost;
                const bool have_enough_time = (cur_cost + 20) < remain_cycle();

                if (have_enough_time && cur_weight_tmp >= select_weight) {
                    select_weight = cur_weight_tmp;
                    cur_berths_id_list = Tools::first_n(berths_id, iter_dis + 1);
                }
            }

            if (select_weight > best_weight) {
                best_weight = select_weight;
                best_id_list = cur_berths_id_list;
            }
        }


        if (best_id_list.empty()) {
            log_info("best_id_list is empty");
            return {};
        }

        return best_id_list;
    }


    /**
     * @brief 生成判断是否是障碍物的lambda函数
     *
     * @param robot_id 当前机器人id
     * @param only_care_high_priority 是否只关心高优先级机器人
     * @param only_care_neighbor 是否只关心邻居
     * @param only_care_cur_pos 是否只关心当前位置
     * @return auto
     */
    auto get_is_barrier_for_robot_lambda(const int robot_id, bool only_care_high_priority, bool only_care_neighbor,
                                         bool only_care_cur_pos) {
        auto is_barrier_func = [this, robot_id, only_care_high_priority, only_care_neighbor,
                only_care_cur_pos](const Point& p) {
            bool is_barrier1 = game_map.is_barrier_for_robot(p);
            bool is_barrier2 = false;
            bool is_barrier3 = false;
            const Point& cur_pos = robots.at(robot_id).pos;

            if (!game_map.has_collision_effect_for_robot(p)) {
                return is_barrier1;
            }

            for (auto& r : robots) {
                if (r.id == robot_id) {
                    continue;
                }
                if (only_care_high_priority && !r.has_pass_collision_check) {
                    continue;
                }
                if (only_care_neighbor && !Point::is_adjacent(cur_pos, p)) {
                    continue;
                }

                if (r.pos == p) {
                    is_barrier2 = true;
                }
                if (r.get_next_pos() == p && !only_care_cur_pos) {
                    is_barrier3 = true;
                }
            }
            return is_barrier1 || is_barrier2 || is_barrier3;
        };
        return is_barrier_func;
    }

    auto get_find_neighbor_for_robot_lambda() {
        game_map.rand_neighber_again();
        auto find_neighbor_func = [&](const Point& p) { return game_map.neighbors_for_robot(p); };
        return find_neighbor_func;
    }

    /**
     * @brief 获取是否是船的障碍物的lambda函数
     * @param ship_id 船的id
     * @param care_other_ship_cur_pos 是否关心其他船的当前位置
     * @param care_other_ship_next_pos 是否关心其他船的下一个位置
     * @return
     */
    auto get_is_barrier_for_ship_lambda(const int ship_id, bool care_other_ship_cur_pos,
                                        bool care_other_ship_next_pos) {
        auto is_barrier_func = [ship_id, care_other_ship_cur_pos, care_other_ship_next_pos, this](const Point& p) {
            const bool is_barrier1 = game_map.is_barrier_for_ship(p);
            bool is_barrier2 = false;
            bool is_barrier3 = false;

            // 如果没有碰撞效果，直接返回
            if (!game_map.has_collision_effect_for_ship(p)) {
                return is_barrier1;
            }

            // 如果不关心其他船的位置，直接返回
            if (!care_other_ship_cur_pos && !care_other_ship_next_pos) {
                return is_barrier1;
            };

            for (auto& s : ships) {
                if (s.id == ship_id) {
                    continue;
                }

                auto s_cur_area = s.get_ship_area();
                auto s_next_area = s.get_ship_next_area();

                if (care_other_ship_cur_pos && s_cur_area.contain(p)) {
                    is_barrier2 = true;
                }
                if (care_other_ship_next_pos && s_next_area.has_value() && s_next_area.value().contain(p)) {
                    is_barrier3 = true;
                }
            }
            return is_barrier1 || is_barrier2 || is_barrier3;
        };
        return is_barrier_func;
    }


    bool berth_is_baned(const int berth_id) {
        // 到了最后时刻,只能往指定的港口运输货物
        if (final_time) {
            std::vector<int> final_berth;
            // 将还能动的船的目的地加入
            for (auto& ship : ships) {
                if (ship.berth_id != -1) {
                    final_berth.push_back(ship.berth_id);
                }
            }
            if (!berths[berth_id].is_baned) {
                bool in_final_berth =
                    std::any_of(final_berth.begin(), final_berth.end(), [berth_id](int v) { return v == berth_id; });
                return !in_final_berth;
            }
            return true;
        }

        return (berths[berth_id].is_baned || berths[berth_id].tmp_baned);
    }


    // int goted_goods_avg_money() {
    //   if (goted_goods_num == 0) {
    //     return 0;
    //   }
    //   return goted_goods_money / goted_goods_num;
    // }
    // int selled_goods_avg_money() {
    //   if (selled_goods_num == 0) {
    //     return 0;
    //   }
    //   return selled_goods_money / selled_goods_num;
    // }

    /**
     * @brief Represents a pair of integers.
     *        first: berth id
     *        second: cost
     */
    std::pair<int, int> get_minimum_berth_cost_for_robot(const Point& p) {
        int min_cost = 999999;
        int min_berth_id = 0;
        for (int i = 0; i < berths.size(); i++) {
            if (berth_is_baned(i)) {
                continue;
            }

            // auto cur_cost = get_cost_from_berth_to_point(i, p);
            auto cur_cost = berths_come_from_for_robot[i].get_point_cost(p);
            if (cur_cost.has_value()) {
                if (cur_cost.value() < min_cost) {
                    min_cost = cur_cost.value();
                    min_berth_id = i;
                }
            }
        }
        return {min_berth_id, min_cost};
    }

    /**
 * @brief Represents a pair of integers.
 *        first: berth id
 *        second: cost
 */
    std::pair<int, int> get_minimum_berth_cost_for_ship(const Point& p) {
        int min_cost = 999999;
        int min_berth_id = 0;
        for (int i = 0; i < berths.size(); i++) {
            // auto cur_cost = get_cost_from_berth_to_point(i, p);
            auto cur_cost = berths_come_from_for_ship[i].get_point_cost(p);
            if (cur_cost.has_value()) {
                if (cur_cost.value() < min_cost) {
                    min_cost = cur_cost.value();
                    min_berth_id = i;
                }
            }
        }
        return {min_berth_id, min_cost};
    }

    /**
     * @brief 获取离指定点最近的交货点
     * @param p
     * @return std::pair<int, int> first: delivery point id, second: cost
     */
    std::pair<int, int> get_minimum_delivery_point_cost_for_ship(const Point& p) {
        int min_cost = 999999;
        int min_delivery_point_id = Tools::random(0ul, delivery_points.size() - 1);
        for (int i = 0; i < delivery_points.size(); i++) {
            auto cur_cost = delivery_point_come_from[i].get_point_cost(p);
            if (cur_cost.has_value()) {
                if (cur_cost.value() < min_cost) {
                    min_cost = cur_cost.value();
                    min_delivery_point_id = i;
                }
            }
        }
        return {min_delivery_point_id, min_cost};
    }


    std::pair<int, int> get_minimum_berth_cost_2(const Point& p) {
        int min_cost = 999999;
        int min_berth_id = 0;
        for (int i = 0; i < berths.size(); i++) {
            // auto cur_cost = get_cost_from_berth_to_point(i, p);
            auto cur_cost = berths_come_from_for_robot[i].get_point_cost(p);
            if (cur_cost.has_value()) {
                if (cur_cost.value() < min_cost) {
                    min_cost = cur_cost.value();
                    min_berth_id = i;
                }
            }
        }
        return {min_berth_id, min_cost};
    }

    void print_goods_info() {
        for (auto& berth : berths) {
            log_info("cur_goods_num:%d,cur_goods_value:%d", berth.goods_num(), berth.goods_value());
        }
        // log_info("total_goods_num:%d,total_goods_money:%d,goted_goods_num:%d,"
        //          "goted_goods_money:%d,selled_goods_num:%d,selled_goods_money:%d",
        //          total_goods_num, total_goods_money, goted_goods_num,
        //          goted_goods_money, selled_goods_num, selled_goods_money);
        // log_info("total_goods_avg_money:%d,goted_goods_avg_money:%d,selled_goods_"
        //          "avg_money:%d",
        //          total_goods_avg_money(), goted_goods_avg_money(),
        //          selled_goods_avg_money());
    }

    void print_final_info() {
        print_goods_info();

        for (auto& berth : berths) {
            berth.print();
        }

        // statistic.print_goted_goods_value();
        // statistic.print_selled_goods_value();
        // statistic.print_total_goods_value();
        for (auto& robot : robots) {
            robot.printf_goods_statistic();
        }
        for (auto& robot : robots) {
            log_info("robot[%d] collision_cycle:%d", robot.id, robot.collision_cycle);
        }
        // statistic.goods_statistic();

        for (auto& ship : ships) {
            ship.printf_cur_value();
        }
        fprintf(stderr,
                "{\"robot_num\":%lu,\"ship_num\":%lu,\"ship_capacity\":%d,\"total_goods_money\":%d,"
                "\"goted_goods_money\":%d,\"selled_goods_money\":%d,\"total_goods_num\":%d,"
                "\"goted_goods_num\":%d,\"selled_goods_num\":%d}\n",
                robots.size(), ships.size(), ship_capacity, statistic.total_goods_value(),
                statistic.goted_goods_value(), statistic.selled_goods_value(), statistic.total_goods_count(),
                statistic.goted_goods_count(), statistic.selled_goods_count());
        statistic.printf_goods_statistic();
    }

    void all_come_from_init() {
        auto is_barrier_for_robot = [&](const Point& p) { return game_map.is_barrier_for_robot(p); };
        auto is_neighbors_for_robot = [&](const Point& p) { return game_map.neighbors_for_robot(p); };

        auto is_barrier_for_ship = [&](const Point& p) { return game_map.is_barrier_for_ship(p); };
        auto is_neighbor_for_ship = [&](const Point& p) { return game_map.neighbors_for_ship(p); };

        // 主航道（非碰撞区域）的花费为 2
        auto ship_cost = [&](const Point& p) {
            if (game_map.has_collision_effect_for_ship(p)) {
                return 1;
            }
            else {
                return 2;
            }
        };

        for (int i = 0; i < berths.size(); i++) {
            game_map.rand_neighber_again();

            // 对于机器人来说，使用 berth_area 来进行打表
            const auto& start = berths[i].berth_area;
            berths_come_from_for_robot.emplace_back();
            berths_come_from_for_robot.back().init("berth[" + std::to_string(i) + "]_come_from_for_robot", start,
                                                   is_barrier_for_robot, is_neighbors_for_robot,
                                                   PATHHelper::default_cost);

            // 对于轮船来说，使用中心点 berths[i].pos 来进行打表
            berths_come_from_for_ship.emplace_back();
            berths_come_from_for_ship.back().init("berth[" + std::to_string(i) + "]_come_from_for_ship", berths[i].pos,
                                                  is_barrier_for_ship, is_neighbor_for_ship, ship_cost);
        }

        for (int i = 0; i < robot_shops.size(); i++) {
            game_map.rand_neighber_again();
            const Point& start1 = robot_shops[i];
            robot_shops_come_from.emplace_back();
            robot_shops_come_from.back().init("robot_shop[" + std::to_string(i) + "]_come_from", start1,
                                              is_barrier_for_robot, is_neighbors_for_robot, PATHHelper::default_cost);
        }

        for (int i = 0; i < delivery_points.size(); i++) {
            game_map.rand_neighber_again();
            const Point& start1 = delivery_points.at(i);
            delivery_point_come_from.emplace_back();
            delivery_point_come_from.back().init("delivery_point[" + std::to_string(i) + "]_come_from", start1,
                                                 is_barrier_for_ship, is_neighbor_for_ship, ship_cost);
        }
    }

    /**
     * @brief 提取地图中的有效信息,例如机器人商店,船只商店,交货点,港口信息
     *
     */
    void map_process() {
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

        auto update_berth_area_info = [&](Area& a) {
            auto berth = std::find_if(berths.begin(), berths.end(), [&](const Berth& b) { return a.contain(b.pos); });
            log_assert(berth != berths.end(), "berth not found");
            log_assert(!berth->berth_area.valid(), "already set berth area");
            log_assert(a.valid(), "invalid area:%s", a.to_string().c_str());
            berth->set_berth_area(a);
            log_info("berth[%d] area:%s", berth->id, a.to_string().c_str());
        };
        auto update_dock_area_info = [&](Area& a) {
            auto berth = std::find_if(berths.begin(), berths.end(), [&](const Berth& b) { return a.contain(b.pos); });

            log_assert(berth != berths.end(), "berth not found");
            log_assert(!berth->berth_area.valid(), "already set dock area");

            log_assert(a.valid(), "invalid area:%s", a.to_string().c_str());
            berth->set_dock_area(a);
            log_info("berth[%d] dock area:%s", berth->id, a.to_string().c_str());
        };

        bool dock_visited[200][200];
        bool berth_visited[200][200];
        std::memset(dock_visited, 0, sizeof(dock_visited));
        std::memset(berth_visited, 0, sizeof(berth_visited));

        for (int i = 0; i < 200; i++) {
            for (int j = 0; j < 200; j++) {
                const auto& pos_type = game_map.get_pos_type({i, j});
                switch (pos_type) {
                case GameMap::ROBOT_SHOP: {
                    robot_shops.emplace_back(i, j);
                    log_trace("robot_shop:(%d,%d)", P_ARG(Point(i, j)));
                    break;
                }
                case GameMap::SHIP_SHOP: {
                    ship_shops.emplace_back(i, j);
                    log_trace("ship_shop:(%d,%d)", P_ARG(Point(i, j)));
                    break;
                }
                case GameMap::DELIVERY: {
                    delivery_points.emplace_back(i, j);
                    log_trace("delivery_point:(%d,%d),size:%d", P_ARG(delivery_points.back()), delivery_points.size());
                    break;
                }
                case GameMap::BERTH: {
                    // 检测矩形区域
                    if (!berth_visited[i][j]) {
                        Point left_top = Point(i, j);
                        Point right_bottom = Point(i, j);
                        while (right_bottom.x < 200 && game_map.is_berth_pos({right_bottom.x, j})) {
                            berth_visited[right_bottom.x][j] = true;
                            right_bottom.x++;
                        }
                        while (right_bottom.y < 200 && game_map.is_berth_pos({i, right_bottom.y})) {
                            berth_visited[i][right_bottom.y] = true;
                            right_bottom.y++;
                        }
                        Area berth_area_tmp = Area(left_top, {right_bottom.x - 1, right_bottom.y - 1});
                        update_berth_area_info(berth_area_tmp);

                        for (int x = left_top.x; x < right_bottom.x; x++) {
                            for (int y = left_top.y; y < right_bottom.y; y++) {
                                berth_visited[x][y] = true;
                            }
                        }
                    }
                    break;
                }
                case GameMap::DOCK: {
                    // 检测矩形区域
                    if (!dock_visited[i][j]) {
                        Point left_top = Point(i, j);
                        Point right_bottom = Point(i, j);
                        while (right_bottom.x < 200 && game_map.is_dock_pos({right_bottom.x, j})) {
                            right_bottom.x++;
                        }
                        while (right_bottom.y < 200 && game_map.is_dock_pos({i, right_bottom.y})) {
                            right_bottom.y++;
                        }
                        Area dock_area_tmp = Area(left_top, {right_bottom.x - 1, right_bottom.y - 1});
                        update_dock_area_info(dock_area_tmp);
                        for (int x = left_top.x; x < right_bottom.x; x++) {
                            for (int y = left_top.y; y < right_bottom.y; y++) {
                                dock_visited[x][y] = true;
                            }
                        }
                    }
                    break;
                }
                default:
                    break;
                }
            }
        }

        for (auto& berth : berths) {
            log_assert(berth.dock_area.contain(berth.pos), "berth[%d] dock_area not contain pos:(%d,%d)", berth.id,
                       P_ARG(berth.pos));
            log_assert(berth.berth_area.contain(berth.pos), "berth[%d] berth_area not contain pos:(%d,%d)", berth.id,
                       P_ARG(berth.pos));
            log_assert(berth.dock_area.contain(berth.berth_area), "berth[%d] dock_area not contain berth_area",
                       berth.id);
        }

        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        log_info("map_process done, time:%d ms",
                 std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    }


    /**
     * @brief 获得最富有的港口(港口周围很多货还没有搬)路径
     *
     * @param from 机器人位置
     * @param berth_id 返回的港口id
     * @param founded  是否找到路径
     * @return std::vector<Point> 返回的路径
     */
    std::vector<Point> get_rich_berth_path(const Point& from, int& berth_id, bool& founded) {
        std::vector<Point> path;
        float final_wight = -200.f; // 越大越好
        int min_berth_id = 0;
        bool found_path = false;
        // 只要能到达任意一个港口就行
        for (int i = 0; i < berths.size(); i++) {
            if (berth_is_baned(i)) {
                continue;
            }

            Point berth_pos = Point(berths[i].pos.x, berths[i].pos.y);
            // auto cur_path = get_path_from_point_to_berth(i, from, found_path);
            auto cur_path = berths_come_from_for_robot[i].get_path_from_point(from, found_path);

            auto& cur_berth = berths[i];
            if (found_path) {
                const int near_goods_num = cur_berth.near_goods_num;
                const int near_goods_value = cur_berth.near_goods_value;
                const int near_goods_distance = cur_berth.near_goods_distance * 2 + cur_path.size();

                if (near_goods_value < 4000) {
                    continue;
                }

                const float money_per_distance = static_cast<float>(near_goods_value) / (near_goods_distance + 1);
                const float money_per_goods =
                    static_cast<float>(near_goods_value) / static_cast<float>(near_goods_num + 1);

                // TODO: 有更好的方法吗? 越大越好
                float cur_wight = near_goods_value / (near_goods_distance + 1.0);

                if (cur_wight > final_wight) {
                    final_wight = cur_wight;
                    min_berth_id = i;
                    path = cur_path;
                    founded = true;
                }
            }
        }
        berth_id = min_berth_id;

        log_info("will go to "
                 "berth[%d],near_goods_num:%d,near_goods_value:%d,near_goods_"
                 "distance:%d",
                 min_berth_id, berths[min_berth_id].near_goods_num, berths[min_berth_id].near_goods_value,
                 berths[min_berth_id].near_goods_distance);

        return path;
    }

    std::vector<Point> get_near_berth_path_exclude(const Point& from, int& berth_id, bool& founded,
                                                   std::vector<int>& visit) {
        std::vector<Point> path;
        int min_dis = 999999;
        int min_berth_id = std::rand() % berths.size();
        bool found_path = false;

        // 只要能到达任意一个港口就行
        for (int i = 0; i < berths.size(); i++) {
            if (berth_is_baned(i)) {
                continue;
            }
            if (std::any_of(visit.begin(), visit.end(), [i](int v) { return v == i; })) {
                continue;
            }

            Point berth_pos = Point(berths[i].pos.x, berths[i].pos.y);
            // auto cur_path = get_path_from_point_to_berth(i, from, found_path);
            auto cur_path = berths_come_from_for_robot[i].get_path_from_point(from, found_path);

            if (found_path) {
                if (cur_path.size() < min_dis) {
                    min_dis = cur_path.size();
                    min_berth_id = i;
                    path = cur_path;
                    founded = true;
                }
            }
        }

        berth_id = min_berth_id;
        visit.push_back(min_berth_id);
        log_info("from(%d,%d) to berth[%d] (%d,%d) size:%d", from.x, from.y, min_berth_id, berths[min_berth_id].pos.x,
                 berths[min_berth_id].pos.y, path.size());
        return path;
    }

    /**
     * @brief 检测点是否在港口区域
     *
     * @param p
     * @return std::optional<int> 港口id
     */
    std::optional<int> in_berth_area(const Point& p) {
        for (auto& berth : berths) {
            if (berth.in_berth_area(p)) {
                return berth.id;
            }
        }
        return std::nullopt;
    }

    /**
     * @brief 检测点是否在港口停泊区域
     *
     * @param p
     * @return std::optional<int> 港口id
     */
    std::optional<int> in_dock_area(const Point& p) {
        for (auto& berth : berths) {
            if (berth.in_dock_area(p)) {
                return berth.id;
            }
        }
        return std::nullopt;
    }

    std::optional<int> in_delivery_point_area(const Point& p) const {
        for (int i = 0; i < delivery_points.size(); i++) {
            if (delivery_points[i] == p) {
                return i;
            }
        }
        return std::nullopt;
    }

    /**
     * @brief 检测点是否在机器人商店区域
     *
     * @param p
     * @return std::optional<int> 机器人商店位置
     */
    std::optional<Point> in_robot_shop_area(const Point& p) const {
        for (int i = 0; i < robot_shops.size(); i++) {
            // TODO: 扩大范围
            if (robot_shops[i] == p) {
                return p;
            }
        }
        return std::nullopt;
    }

    // ------------------------------------------
    // 初始化
    // ------------------------------------------
    bool init_game_map() {
        char c_buf[210] = {0};
        for (int i = 0; i < 200; i++) {
            memset(c_buf, 0, sizeof(c_buf));
            scanf("%s", c_buf);
            log_raw("%s,len:%d\n", c_buf, strlen(c_buf));
            game_map.write_line(c_buf, i);
        }
        log_info("Game map initialized");
        game_map.print_map();
        return true;
    }

    void init_berths() {
        int berth_num = 0;
        scanf("%d", &berth_num);
        for (int i = 0; i < berth_num; i++) {
            int berth_id = -1;
            int x = -1;
            int y = -1;
            int loadingspeed = -1;
            scanf("%d%d%d%d", &berth_id, &x, &y, &loadingspeed);
            berths.emplace_back(Berth{berth_id, Point{x, y}, 0, loadingspeed});
        }
        std::sort(berths.begin(), berths.end(), [](const Berth& a, const Berth& b) { return a.id < b.id; });

        for (int i = 0; i < berths.size(); i++) {
            log_assert(berths[i].id == i, "berth id:%d", berths[i].id);
            log_info("berth[%d](%d,%d),loading_speed:%d", i, berths[i].pos.x, berths[i].pos.y, berths[i].loading_speed);
        }

        // 计算 transport_time 平均数
        int loading_speed_sum = 0;
        for (auto& berth : berths) {
            loading_speed_sum += berth.loading_speed;
        }

        float loading_speed_avg = static_cast<float>(loading_speed_sum) / berths.size();

        for (auto& berth : berths) {
            berth.avg_berth_loading_speed = loading_speed_avg;
        }
        log_info("Berths initialized,loading_speed_avg:%f", loading_speed_avg);
    }

    static void init_done() {
        char okk[10];
        scanf("%s", okk);
        printf("OK\n");
        fflush(stdout);
    }

    void init_ships() {
        scanf("%d", &ship_capacity);
        log_info("Ships initialized,ship_capacity:%d", ship_capacity);
    }

    void init() {
        auto start = std::chrono::high_resolution_clock::now();
        init_game_map();
        init_berths();
        // game_map.init_robot_pos();
        init_ships();
        auto end = std::chrono::high_resolution_clock::now();

        map_process();
        all_come_from_init();
        init_map_info();
        log_info("IoLayer init time:%d ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        init_done();
    }

    /**
     * @brief 初始化,但是不发送ok,用统计地图数据
     *
     */
    void init_but_not_send_ok() {
        auto start = std::chrono::high_resolution_clock::now();
        init_game_map();
        init_berths();
        // game_map.init_robot_pos();
        init_ships();
        auto end = std::chrono::high_resolution_clock::now();

        map_process();
        all_come_from_init();
        log_info("IoLayer init time:%d ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        print_final_info();
        // 等待 5 秒
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    void init_map_info() {
        std::unordered_set<Point> berth_robots_cost_set{};
        for (int i = 0; i < 200; i++) {
            for (int j = 0; j < 200; j++) {
                const auto pos = Point(i, j);
                const auto [berth_id,berth_cost] = get_minimum_berth_cost_for_robot(pos);
                if (berth_cost < 55) {
                    berth_robots_cost_set.insert(pos);
                }

                if (berth_cost < 2000) {
                    berths[berth_id].near_points.insert(pos);
                }
            }
        }
        berth_area_size_in_55_cost = berth_robots_cost_set.size();
        char map_copy[200][200] = {};
        std::memcpy(map_copy, game_map.map, sizeof(map_copy));

        for (int i = 0; i < 200; i++) {
            for (int j = 0; j < 200; j++) {
                for (const auto& berth : berths) {
                    if (berth.near_points.find(Point(i, j)) != berth.near_points.end()) {
                        // 数字转字符
                        map_copy[i][j] = '0' + berth.id;
                    }
                }
            }
        }

        for (int i = 0; i < 200; i++) {
            for (int j = 0; j < 200; j++) {
                log_raw("%c", map_copy[i][j]);
            }
            log_raw("\n");
        }

        for (int i = 0; i < berths.size(); i++) {
            log_info("berth[%d] near_points size:%d,pecentage %d, max_robot_num:%d", i, berths[i].near_points.size(),
                     berths[i].near_points.size()*100/berths_come_from_for_robot[i].map_size(),
                     calc_max_robots_in_berth(i));
            berths[i].max_robot_num = calc_max_robots_in_berth(i);
        }
        // log_assert(false,"");


        log_info("berth_robots_cost_set size:%d, land size:%d", berth_area_size_in_55_cost, game_map.land_size());

        int avg_delivery_point_cost = 0;
        for (auto& berth : berths) {
            const auto& berth_pos = berth.pos;
            const auto [delivery_point_id,delivery_point_cost] = get_minimum_delivery_point_cost_for_ship(berth_pos);
            berth.near_delivery_distance = delivery_point_cost;
            berth.near_delivery_id = delivery_point_id;
            avg_delivery_point_cost += delivery_point_cost;
        }
        avg_delivery_point_cost /= berths.size();
        berth_avg_delivery_point_cost = avg_delivery_point_cost;
        log_info("avg_delivery_point_cost:%d", berth_avg_delivery_point_cost);
    }

    // ------------------------------------------
    // 输入输出
    // ------------------------------------------
    void input_cycle() {
        // 读取当前周期的信息
        scanf("%d%d", &cur_cycle, &cur_money);
        log_info("cycle[%d],money:%d", cur_cycle, cur_money);
        // 读取新增货物信息
        new_goods_list.clear();
        scanf("%d", &new_goods_num);
        for (int i = 0; i < new_goods_num; i++) {
            int x, y, money;
            scanf("%d%d%d", &x, &y, &money);
            if (money != 0) {
                new_goods_list.emplace_back(Point(x, y), money, cur_cycle + 1000, GoodsStatus::Normal);
                log_trace("new goods[%d]:(%d,%d),money:%d,end_cycle:%d", i, x, y, money, cur_cycle + 1000);
            }
        }

        // 读取机器人信息
        int robots_num = -1;
        scanf("%d", &robots_num);

        for (int i = 0; i < robots_num; i++) {
            int id, goods, x, y;
            scanf("%d%d%d%d", &id, &goods, &x, &y);
            // 添加新的机器人
            if (std::find_if(robots.begin(), robots.end(), [&id](const Robot& r) { return r.id == id; }) == robots.
                end()) {
                robots.emplace_back(id, Point(x, y), goods == 1, 1);
                log_info("new robot[%d](%d,%d),goods:%d", i, x, y, goods);
                continue;
            }
            // 更新老的机器人
            robots[i].had_goods = goods == 1;
            robots[i].pos = Point(x, y);
            robots[i].status = 1;
            log_info("robot[%d](%d,%d),goods:%d", i, x, y, goods);
        }

        for (int i = 0; i < robots.size(); i++) {
            log_info("robot[%d](%d,%d),goods:%d, robots.size():%d", i, robots[i].pos.x, robots[i].pos.y,
                     robots[i].had_goods, robots.size());
            log_assert(robots[i].id == i, "robot id:%d,i:%d", robots[i].id, i);
        }

        // 船只状态
        int ship_nums = -1;
        scanf("%d", &ship_nums);
        log_info("ship_nums:%d", ship_nums);
        for (int i = 0; i < ship_nums; i++) {
            int ship_id, goods_num, x, y, direction, status;
            scanf("%d%d%d%d%d%d", &ship_id, &goods_num, &x, &y, &direction, &status);
            if (std::find_if(ships.begin(), ships.end(), [&ship_id](const Ship& ship) { return ship.id == ship_id; }) ==
                ships.end()) {
                ships.emplace_back(ship_id, goods_num, ship_capacity, Point(x, y),
                                   Direction::int_to_direction(direction), status, cur_cycle);
                log_info("new ship[%d](%d,%d)", i, x, y);
                continue;
            }
            ships[i].cur_capacity = goods_num;
            ships[i].pos.x = x;
            ships[i].pos.y = y;
            ships[i].direction = Direction::int_to_direction(direction);
            ships[i].status = status;
            log_info("ship[%d](%d,%d),direction:%d,status:%d,cur_capacity:%d", i, x, y, direction, status, goods_num);
        }

        char okk[10];
        scanf("%s", okk);
    }

    void output_cycle() {
        log_trace("cycle[%d],inst_num:%d", cur_cycle, commands.size());
        for (const auto& command : commands) {
            char fmt_buf[50] = {};
            switch (command.inst) {
            case ROBOT_MOVE:
                std::sprintf(fmt_buf, "move %d %d\n", command.arg1, command.arg2);
                break;
            case ROBOT_GET:
                std::sprintf(fmt_buf, "get %d\n", command.arg1);
                break;
            case ROBOT_PULL:
                std::sprintf(fmt_buf, "pull %d\n", command.arg1);
                break;
            case ROBOT_LBOT:
                std::sprintf(fmt_buf, "lbot %d %d\n", command.arg1, command.arg2);
                break;
            case SHIP_MOVE:
                std::sprintf(fmt_buf, "ship %d\n", command.arg1);
                break;
            case SHIP_ROT:
                std::sprintf(fmt_buf, "rot %d %d\n", command.arg1, command.arg2);
                break;
            case SHIP_LBOAT:
                std::sprintf(fmt_buf, "lboat %d %d\n", command.arg1, command.arg2);
                break;
            case SHIP_DEPT:
                std::sprintf(fmt_buf, "dept %d\n", command.arg1);
                break;
            case SHIP_BERTH:
                std::sprintf(fmt_buf, "berth %d\n", command.arg1);
                break;
            default:
                log_fatal("unknown command inst:%d", command.inst);
                break;
            }
            printf("%s", fmt_buf);
            log_trace("command:%s", fmt_buf);
        }
        printf("OK\n");
        fflush(stdout);
        commands.clear();
    }

    // ------------------------------------------
    // 机器人指令
    // ------------------------------------------
    void robot_move(const int robot_id, const Direction::Direction direction) {
        commands.push_back({ROBOT_MOVE, robot_id, static_cast<int>(direction)});
    }

    void robot_move(const int robot_id, const Point& to) {
        this->robot_move(robot_id, Direction::calc_direction(robots[robot_id].pos, to));
    }

    void robot_get(int robot_id) { commands.push_back({ROBOT_GET, robot_id, 0}); }

    void robot_pull(int robot_id) { commands.push_back({ROBOT_PULL, robot_id, 0}); }

    void robot_lbot(const Point& pos) { commands.push_back({ROBOT_LBOT, pos.x, pos.y}); }

    // ------------------------------------------
    // 船只指令
    // ------------------------------------------
    void ship_move(const int ship_id) { commands.push_back({SHIP_MOVE, ship_id, 0}); }

    void ship_dept(const int ship_id) { commands.push_back({SHIP_DEPT, ship_id, 0}); }

    void ship_berth(const int ship_id) { commands.push_back({SHIP_BERTH, ship_id, 0}); }

    // 0表示顺时针方向，1表示逆时针方向
    void ship_rot(const int ship_id, const int rot_direction) {
        log_assert(rot_direction == 0 || rot_direction == 1, "rot_direction:%d", rot_direction);
        commands.push_back({SHIP_ROT, ship_id, rot_direction});
    }

    void ship_lboat(const Point& pos) { commands.push_back({SHIP_LBOAT, pos.x, pos.y}); }

    bool is_valid_move(const Point& from, const Point& to) {
        if (from.x < 0 || from.x >= 200 || from.y < 0 || from.y >= 200) {
            log_trace("from(%d,%d) out of range", from.x, from.y);
            return false;
        }
        if (to.x < 0 || to.x >= 200 || to.y < 0 || to.y >= 200) {
            log_trace("to(%d,%d) out of range", to.x, to.y);
            return false;
        }
        if (abs(from.x - to.x) + abs(from.y - to.y) != 1) {
            log_trace("from(%d,%d) to(%d,%d) not valid", from.x, from.y, to.x, to.y);
            return false;
        }
        return true;
    }
};
