#pragma once

#include "direction.hpp"
#include "io_laye_new.hpp"
#include "log.h"
#include "point.hpp"
#include "ship.hpp"
#include <algorithm>
#include <optional>

class ShipControl {

    IoLayerNew *io_layer = nullptr;

    struct BerthInfo {
        std::optional<int> occupied_ship_id;
        int berth_id;
        int goods_value;
        int goods_num;
        int distance_to_ship;
        int distance_to_delivery;

        int all_distance() const { return distance_to_ship + distance_to_delivery; }
    };

public:
    explicit ShipControl(IoLayerNew *io_layer) : io_layer(io_layer) {}

    ~ShipControl() = default;

    /**
     * @brief 获取是否是船的障碍物的lambda函数
     * @param ship_id 船的id
     * @param care_other_ship_cur_pos 是否关心其他船的当前位置
     * @param care_other_ship_next_pos 是否关心其他船的下一个位置
     * @return
     */
    auto get_is_barrier_for_ship_lambda(const int ship_id, bool care_other_ship_cur_pos,
                                        bool care_other_ship_next_pos) {

        auto is_barrier_func = [this, ship_id, care_other_ship_cur_pos,
                care_other_ship_next_pos](const Point &p) {
            bool is_barrier1 = io_layer->game_map.is_barrier_for_ship(p);
            bool is_barrier2 = false;
            bool is_barrier3 = false;

            // 如果没有碰撞效果，直接返回
            if (!io_layer->game_map.has_collision_effect_for_ship(p)) {
                return is_barrier1;
            }

            // 如果不关心其他船的位置，直接返回
            if (!care_other_ship_cur_pos && !care_other_ship_next_pos) {
                return is_barrier1;
            };

            for (auto &s: io_layer->ships) {
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


    std::vector<BerthInfo> get_berth_infos_for_ship(Ship &ship) {
        std::vector<BerthInfo> berth_infos;
        for (auto &berth: io_layer->berths) {
            BerthInfo info{};
            const auto [goods_num, goods_value] = berth.goods_goods_first_n(ship.capacity);
            info.occupied_ship_id = berth.occupied_ship_id;
            info.berth_id = berth.id;
            info.goods_value = goods_value;
            info.goods_num = goods_num;
            info.distance_to_ship = io_layer->berths_come_from_for_ship.at(berth.id).get_point_cost(ship.pos).value_or(
                    2000);
            info.distance_to_delivery = io_layer->get_minimum_delivery_point_cost_for_ship(ship.pos).second;
            berth_infos.emplace_back(info);
        }
        return berth_infos;
    }

    /**
     * @brief 找到最有价值的泊位, 不考虑距离等因素，一般用于船只出生时，或者刚卸完货时
     *
     * @param ship
     * @return std::optional<int>
     */
    int get_rich_berth_id(Ship &ship) {
        std::vector<BerthInfo> berth_infos = get_berth_infos_for_ship(ship);
        int berth_id = get_random_berth_id(ship);
        int max_value = 0;
        for (auto &info: berth_infos) {
            if (info.occupied_ship_id.has_value() || info.berth_id == ship.target_berth_id) {
                continue;
            }
            if (info.goods_value >= max_value) {
                max_value = info.goods_value;
                berth_id = info.berth_id;
            }

        }
        return berth_id;
    }

    /**
     * @brief 找到最适合的泊位, 考虑距离等因素，用于船只装完货物但没有装满时
     *
     * @param ship
     * @return std::optional<int>
     */
    int get_fit_berth_id(Ship &ship) {
        std::vector<BerthInfo> berth_infos = get_berth_infos_for_ship(ship);
        const int remain_capacity = ship.remain_capacity();

        static const int MAX_DISTANCE = 20000;

        int full_berth_id = get_random_berth_id(ship);
        int full_min_distance = MAX_DISTANCE;
        int not_full_berth_id = get_random_berth_id(ship);
        int not_full_min_distance = MAX_DISTANCE;

        for (auto &info: berth_infos) {
            if (info.occupied_ship_id.has_value() || info.berth_id == ship.target_berth_id) {
                continue;
            }

            if (info.goods_num >= remain_capacity) {
                // 1. 可以装满
                if (info.all_distance() < full_min_distance) {
                    full_min_distance = info.all_distance();
                    full_berth_id = info.berth_id;
                }
            } else {
                // 2. 不能装满 (选择最近的，还是选择价值最高的？)
                if (info.all_distance() < not_full_min_distance) {
                    not_full_min_distance = info.all_distance();
                    not_full_berth_id = info.berth_id;
                }
            }
        }
        const int final_berth_id = full_min_distance < MAX_DISTANCE ? full_berth_id : not_full_berth_id;
        return final_berth_id;
    }


    int get_random_berth_id(Ship &ship) {
        int max_berth_id = io_layer->berths.size();
        int sel_berth_id = std::rand() % max_berth_id;

        while (sel_berth_id == ship.target_berth_id) {
            sel_berth_id = std::rand() % max_berth_id;
        }
        return sel_berth_id;
    }

    /**
     * @brief 获取最适合的交付点
     * @param ship
     * @return
     */
    int get_best_deliver_id(Ship &ship) {
        // 暂时只考虑距离因素
        int best_deliver_id;
        best_deliver_id = io_layer->get_minimum_delivery_point_cost_for_ship(ship.pos).first;
        return best_deliver_id;
    }

    int get_random_deliver_id() {
        int max_deliver_id = io_layer->delivery_points.size();
        return std::rand() % max_deliver_id;
    }

    /**
     * @brief 控制船的行为,让船去泊位
     * @param ship
     */
    void go_to_berth(Ship &ship) {
        if (!ship.normal_status()) {
            return;
        }
        if (ship.fsm != ShipFSM::GO_TO_BERTH) {
            return;
        }
        if (!ship.path.empty()) {
            return;
        }
        int sel_berth_id;
        if (ship.empty()) {
            sel_berth_id = get_rich_berth_id(ship);
        } else {
            sel_berth_id = get_fit_berth_id(ship);
        }


        auto ship_head = ship.get_ship_head();
        auto &berth_come_from =
                io_layer->berths_come_from_for_ship.at(sel_berth_id);

        bool berth_path_founded = false;
        auto berth_path = berth_come_from.get_path_from_point(ship_head.left_top,
                                                              berth_path_founded);

        if (!berth_path_founded) {
            log_info("ship[%d] no berth[%d] path founded", ship.id, sel_berth_id);
            return;
        }
        log_debug("ship_area:%s", ship_head.to_string().c_str());
        for (auto &p: berth_path) {
            log_info("ship[%d] berth_path(%d,%d)", ship.id, P_ARG(p));
        }

        ship.set_target_berth_id(sel_berth_id);
        ship.path = berth_path;
        drop_path_point_if_reach(ship);
        ship.update_ship_next_pos(get_is_barrier_for_ship_lambda(ship.id, true, true));
    }

    /**
     * @brief 控制船的行为,让船去交付点
     * @param ship
     */
    void go_to_deliver(Ship &ship) {

        if (ship.fsm != ShipFSM::GO_TO_DELIVERY) {
            return;
        }

        if (!(ship.normal_status() || ship.load_status())) {
            log_trace("ship[%d] go_to_deliver, not normal or not load,status:%d", ship.id, ship.status);
            return;
        }

        if (!ship.path.empty()) {
            log_trace("ship[%d] go_to_deliver, path not empty,size:%d", ship.id, ship.path.size());
            return;
        }

        log_trace("ship[%d] go_to_deliver", ship.id);

        auto best_deliver_id = get_best_deliver_id(ship);

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
        ship.set_target_delivery_id(best_deliver_id);
        drop_path_point_if_reach(ship);
        ship.update_ship_next_pos(get_is_barrier_for_ship_lambda(ship.id, true, true));
    }

    /**
     * @brief 船的行为控制,避免船一半在主航道一般在海上的情况
     * @param ship
     */
    void half_main_sea_avoid(Ship &ship) {
        auto head_area = ship.get_ship_head();

        const bool head1_in_main_sea =
                !io_layer->game_map.has_collision_effect_for_ship(head_area.left_top);
        const bool head2_in_main_sea =
                !io_layer->game_map.has_collision_effect_for_ship(
                        head_area.right_bottom);
        const int in_main_sea_count = head1_in_main_sea + head2_in_main_sea;

        if (in_main_sea_count != 1) {
            return;
        }

        log_trace("ship[%d] half_main_sea_avoid", ship.id);

        const Point new_pos_to =
                head1_in_main_sea ? head_area.right_bottom : head_area.left_top;
        const Point new_pos_from =
                head1_in_main_sea ? head_area.left_top : head_area.right_bottom;

        const auto dir_list =
                Direction::calc_direction_nocheck(new_pos_from, new_pos_to);
        log_assert(dir_list.size() == 1, "dir_list.size:%d", dir_list.size());
        log_debug("new_pos_from(%d,%d) new_pos_to(%d,%d) dir:%d",
                  P_ARG(new_pos_from), P_ARG(new_pos_to), dir_list.front());
        const auto rot_dir =
                Direction::calc_rotate_direction(ship.direction, dir_list.front());
        log_assert(rot_dir.has_value(), "rot_dir not valid");

        const auto [next_pos, next_dir] = Ship::calc_rot_action(
                ship.pos, ship.direction, rot_dir.value() == Direction::CLOCKWISE);

        auto new_ship_area = Ship::calc_ship_area(next_pos, next_dir);

        const auto new_ship_area_points = new_ship_area.to_points();

        auto is_barrier_for_ship = get_is_barrier_for_ship_lambda(ship.id, true, true);

        if (std::any_of(new_ship_area_points.begin(), new_ship_area_points.end(),
                        [&is_barrier_for_ship](const Point &p) {
                            return is_barrier_for_ship(p);
                        })) {
            log_info("ship[%d] half_main_sea_avoid failed", ship.id);
            return;
        }
        const auto next_command = rot_dir.value() == Direction::CLOCKWISE
                                  ? ShipCommand::ROTATE_CLOCKWISE
                                  : ShipCommand::ROTATE_COUNTERCLOCKWISE;

        log_trace("ship[%d] half_main_sea_avoid success", ship.id);
        log_trace(" pre pos:(%d,%d),pre dir:%d, pre command:%d", P_ARG(ship.pos),
                  ship.direction, ship.get_next_command());
        log_trace("next pos:(%d,%d),next dir:%d, next command:%d", P_ARG(next_pos),
                  next_dir, next_command);

        ship.next_direction_before_collison = next_dir;
        ship.next_pos_before_collison = next_pos;
        ship.next_command_before_collison = next_command;
    }


    void sell_goods(Ship &ship) {

        auto deliver_id = io_layer->in_delivery_point_area(ship.pos);
        if (deliver_id.has_value() && deliver_id.value() == ship.target_delivery_id) {
            for (auto &goods: ship.goods_list) {
                io_layer->statistic.selled_goods_list.emplace_back(goods);
            }
            log_trace("ship[%d] sell goods to deliver[%d], nums:%d,val:%d ", ship.id, deliver_id.value(),
                      ship.goods_list.size(), ship.cur_value());
            ship.goods_list.clear();
        }
    }

    /**
     * @brief 更新船下一个位置
     * @param ship
     */
    void update_next_pos(Ship &ship) {
        if (ship.path.size() > 1) {
            log_info("ship[%d],path size >1 :%d", ship.id, ship.path.size());
            ship.update_ship_next_pos(get_is_barrier_for_ship_lambda(ship.id, true, true));
            half_main_sea_avoid(ship);
        } else if (ship.path.size() == 1) {
            log_info("ship[%d],path size == 1 :%d", ship.id, ship.path.size());
            const auto target_point = ship.path.back();
            auto ship_area = ship.get_ship_area();
            if (ship_area.contain(target_point)) {
                if (target_point == ship.pos) {
                    ship.path.pop_back();
                } else {
                    go_to_point_exactly(ship, ship.path.back());
                }
            } else {
                ship.update_ship_next_pos(get_is_barrier_for_ship_lambda(ship.id, true, true));
//                half_main_sea_avoid(ship);
            }
        }
    }

    /**
     * @brief 输出船的命令
     * @param ship
     */
    void output_command(Ship &ship) {
        if (ship.recover_status()) {
            log_info("ship[%d] recover status", ship.id);
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
            case ShipCommand::BERTH: {
                io_layer->ship_berth(ship.id);
                break;
            }
            case ShipCommand::DEPT: {
                io_layer->ship_dept(ship.id);
                break;
            }
        }
    }

    void ship_control_fsm(Ship &ship) {
        if (ship.recover_status()) {
            return;
        }
        auto berth_id = io_layer->in_dock_area(ship.pos);
        const bool should_go_to_berth_and_loading = berth_id.has_value() && berth_id.value() == ship.target_berth_id;

        if (ship.path.empty() || should_go_to_berth_and_loading) {
            switch (ship.fsm) {
                case ShipFSM::FIRST_BORN: {
                    ship.fsm = ShipFSM::GO_TO_BERTH;
                    log_trace("ship[%d] fsm change to GO_TO_BERTH", ship.id);
                    break;
                }
                case ShipFSM::GO_TO_BERTH: {
                    // 当位于泊位附近时,应该停靠到泊位中
                    if (should_go_to_berth_and_loading) {
                        ship.fsm = ShipFSM::LOADING;
                        ship.next_command_before_collison = ShipCommand::BERTH;
                        ship.path.clear();
                        log_trace("ship[%d] fsm change to LOADING", ship.id);
                    } else {
                        ship.fsm = ShipFSM::GO_TO_DELIVERY;
                        log_trace("ship[%d] fsm change to GO_TO_DELIVERY", ship.id);
                    }
                    break;
                }
                case ShipFSM::GO_TO_DELIVERY: {
                    ship.fsm = ShipFSM::GO_TO_BERTH;
                    log_trace("ship[%d] fsm change to GO_TO_BERTH", ship.id);
                    break;
                }
                case ShipFSM::LOADING: {

                }
                    break;
            }
        }
    }

    void ship_loading(Ship &ship) {
        if (ship.fsm != ShipFSM::LOADING) {
            return;
        }
        if (!ship.load_status()) {
            return;
        }
        log_assert(ship.path.empty(), "ship[%d] path not empty", ship.id);

        auto &cur_berth = io_layer->berths[ship.target_berth_id];

        if (ship.full()) {
            ship.fsm = ShipFSM::GO_TO_DELIVERY;
            log_trace("ship[%d] full, change to GO_TO_DELIVERY", ship.id);
            return;
        }
        if (cur_berth.is_empty()) {
            ship.fsm = ShipFSM::GO_TO_DELIVERY;
            log_trace("cur_berth is empty, ship[%d] change to GO_TO_DELIVERY", ship.id);
            return;
        }

        log_trace("ship[%d] loading in berth[%d] ", ship.id, cur_berth.id);
        const int load_count = std::min({ship.remain_capacity(), cur_berth.loading_speed,
                                         static_cast<int>(cur_berth.goods_list.size())});
        for (int i = 0; i < load_count; i++) {
            const auto goods = cur_berth.goods_list.front();
            log_trace("ship[%d] load goods(%d,%d)", ship.id, goods.pos.x, goods.pos.y);
            ship.load(goods);
            cur_berth.goods_list.pop_front();
        }

    }

    /**
     * @brief 船的行为控制,让轮船中心点准确到达某个点，该点必须在轮船当前的区域内
     * @param ship
     * @param p 目标点
     */
    void go_to_point_exactly(Ship &ship, const Point &p) {
        auto ship_area = ship.get_ship_area();
        log_assert(ship_area.contain(p), "ship_area:%s, p(%d,%d)", ship_area.to_string().c_str(),
                   P_ARG(p));
        const auto point_id = ship.get_point_id_in_ship_area(p);

        log_assert(point_id != 3, "id is 3, ship_area:%s, p(%d,%d)", ship_area.to_string().c_str(),
                   P_ARG(p));
        const bool ship_can_move = ship.can_move(get_is_barrier_for_ship_lambda(ship.id, false, false));
        const bool ship_can_rot_clockwise = ship.can_rotate(get_is_barrier_for_ship_lambda(ship.id, false, false),
                                                            true);
        const bool ship_can_rot_counterclockwise = ship.can_rotate(
                get_is_barrier_for_ship_lambda(ship.id, false, false), false);

        auto update_ship_next_pos = [&ship](const Point &p, Direction::Direction direction, ShipCommand command
        ) {
            ship.next_pos_before_collison = p;
            ship.next_command_before_collison = command;
            ship.next_direction_before_collison = direction;
        };


        switch (point_id) {
            case 1: {
                if (ship_can_move) {
                    update_ship_next_pos(Direction::move(ship.pos, ship.direction),
                                         ship.direction, ShipCommand::GO);
                } else if (ship_can_rot_counterclockwise) {
                    const auto [next_pos, next_dir] = Ship::calc_rot_action(
                            ship.pos, ship.direction, false);
                    update_ship_next_pos(next_pos, next_dir, ShipCommand::ROTATE_COUNTERCLOCKWISE);
                } else {
                    log_fatal("ship[%d] can not move or rotate", ship.id);
                }
                break;
            }
            case 2: {
                if (ship_can_rot_clockwise) {
                    const auto [next_pos, next_dir] = Ship::calc_rot_action(
                            ship.pos, ship.direction, true);
                    update_ship_next_pos(next_pos, next_dir, ShipCommand::ROTATE_CLOCKWISE);
                } else if (ship_can_move) {
                    update_ship_next_pos(Direction::move(ship.pos, ship.direction),
                                         ship.direction, ShipCommand::GO);
                } else {
                    log_fatal("ship[%d] can not move or rotate", ship.id);
                }
                break;
            }
            case 4: {
                if (ship_can_rot_counterclockwise) {
                    const auto [next_pos, next_dir] = Ship::calc_rot_action(
                            ship.pos, ship.direction, false);
                    update_ship_next_pos(next_pos, next_dir, ShipCommand::ROTATE_COUNTERCLOCKWISE);
                } else {
                    log_fatal("ship[%d] can not move", ship.id);
                }
                break;
            }
            case 5: {
                if (ship_can_rot_clockwise) {
                    const auto [next_pos, next_dir] = Ship::calc_rot_action(
                            ship.pos, ship.direction, true);
                    update_ship_next_pos(next_pos, next_dir, ShipCommand::ROTATE_CLOCKWISE);
                } else if (ship_can_move) {
                    update_ship_next_pos(Direction::move(ship.pos, ship.direction),
                                         ship.direction, ShipCommand::GO);
                } else {
                    log_fatal("ship[%d] can not move or rotate", ship.id);
                }
                break;
            }
            default: {
                log_fatal("unknown point_id:%d", point_id);
            }
        }
    }

    /**
     * @brief 如果船到达了某个区域，就删除路径中的点
     * @param ship
     */
    void drop_path_point_if_reach(Ship &ship) {

        auto ship_cur_area = ship.get_ship_area();

        while (ship.path.size() > 1) {
            Point p = ship.path.back();
            if (!ship_cur_area.contain(p)) {
                break;
            } else {
                log_info("ship[%d] cur_area reach, pop path.back(%d,%d)", ship.id,
                         P_ARG(p));
                ship.path.pop_back();
            }
        }
    }
};
