#pragma once

#include "direction.hpp"
#include "log.h"
#include "point.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <optional>
#include <utility>

enum class ShipFSM {
    FIRST_BORN, // 第一次出生
    LOADING, // 装货
    GO_TO_BERTH, // 前往泊位
    GO_TO_NEXT_BERTH, // 前往下一个泊位
    GO_TO_DELIVERY, // 前往交货点
    DEAD // 死亡
};

enum class ShipCommand {
    IDLE, // 空闲
    GO, // 前进
    ROTATE_CLOCKWISE, // 顺时针旋转
    ROTATE_COUNTERCLOCKWISE, // 逆时针旋转
    BERTH, // 停泊
    DEPT, // 重置到主航道上


};

class Ship {
public:
    int id;
    int capacity; // 最大载货量
    int cur_capacity = 0; // 当前载货量
    // 正常行驶状态（状态0）恢复状态（状态1）装载状态（状态2)
    int status;
    int berth_id;
    Point pos; // 当前中心位置
    Direction::Direction direction;

    // 规划的路径
    std::vector<Point> path{};
    std::list<int> berth_point_id{}; // 规划好的泊位点,按照顺序依次访问
    int target_berth_id = -1; // 目标泊位id
    int target_delivery_id = -1; // 目标交货点id
    int start_delivery_id = -1; // 从哪个交货点开始运货
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
    std::vector<Goods> goods_list{}; // 货物列表
    int start_cycle = 0; // 开始运输货物的起始时间, 当到达交货点时, 会更新为当前周期
    int helf_sea_cycle = 0;

    void printf_cur_value() {
        fprintf(stderr, "ship[%d] fsm:%d, cur_num:%d,  cur_value:%d path size:%ld \n", this->id, static_cast<int>(this->fsm),this->cur_capacity, this->cur_value(),path.size());
    }
    bool in_berth_point_id_list(const int berth_id) {
        return berth_id==target_berth_id ||  std::find(berth_point_id.begin(), berth_point_id.end(), berth_id) != berth_point_id.end();
    }

    void start_new_transport(const int cur_cycle, const int start_delivery_id,
                             const std::vector<int>& new_berth_loop_id) {
        this->start_cycle = cur_cycle;
        this->start_delivery_id = start_delivery_id;
        goods_list.clear();
        berth_point_id.clear();

        for (const auto& id_tmp : new_berth_loop_id) {
            berth_point_id.emplace_back(id_tmp);
        }

        log_info("ship[%d] start new transport, start_delivery_id:%d", this->id,
                 start_delivery_id);
        for (const auto& id_tmp : berth_point_id) {
            log_info("ship[%d] berth_point_id:%d", this->id, id_tmp);
            log_assert(id_tmp<10, "id_tmp:%d", id_tmp);
        }
    }

    /**
     * @brief 轮船已经在运输中花费的周期
     * @param cur_cycle 当前周期
     * @return 
     */
    int eclipsed_cycle(const int cur_cycle) const {
        return cur_cycle - this->start_cycle;
    }


    /**
     * @brief 判断船是否可以移动到下一个位置
     *
     * @param is_barrier
     * @return true
     * @return false
     */
    bool can_move(std::function<bool(const Point& p)> is_barrier) const {
        const auto next_pos = Direction::move(this->pos, this->direction);
        auto next_ship_area = Ship::calc_ship_area_after_rotate(next_pos, this->direction);

        std::vector<Point> next_ship_points = next_ship_area.to_points();
        bool will_collision =
            std::any_of(next_ship_points.begin(), next_ship_points.end(),
                        [&](const Point& p) { return is_barrier(p); });
        return !will_collision;
    }

    /**
     * @brief 判断船是否可以旋转到下一个位置
     *
     * @param is_barrier
     * @param clockwise_direction
     * @return true
     * @return false
     */
    bool can_rotate(std::function<bool(const Point& p)> is_barrier, bool clockwise_direction) const {
        const auto [next_pos, next_dir] =
            Ship::calc_ship_pos_after_rotate(this->pos, this->direction, clockwise_direction);
        auto next_ship_area = Ship::calc_ship_area_after_rotate(next_pos, next_dir);

        std::vector<Point> next_ship_points = next_ship_area.to_points();
        bool will_collision =
            std::any_of(next_ship_points.begin(), next_ship_points.end(),
                        [&](const Point& p) { return is_barrier(p); });
        return !will_collision;
    }

    /**
     * @brief 获取船位置中点的 id 编号
     * @param p
     * @return
     */
    int get_point_id_in_ship_area(const Point& p) const {
        auto ship_area = this->get_ship_area();
        log_assert(ship_area.contain(p), "ship_area:%s not contain point(%d,%d)", ship_area.to_string().c_str(),
                   P_ARG(p));
        struct id_key {
            Direction::Direction dir;
            int x_diff;
            int y_diff;

            bool operator==(const id_key& other) const {
                return dir == other.dir && x_diff == other.x_diff && y_diff == other.y_diff;
            }
        };
        struct id_key_hash {
            std::size_t operator()(const id_key& k) const {
                return ((std::hash<int>()(k.dir)
                        ^ (std::hash<int>()(k.x_diff) << 1)) >> 1)
                    ^ (std::hash<int>()(k.y_diff) << 1);
            }
        };
        static const std::unordered_map<id_key, int, id_key_hash> id_map = {
            {{Direction::RIGHT, 0, 0}, 0},
            {{Direction::RIGHT, 0, 1}, 1},
            {{Direction::RIGHT, 0, 2}, 2},
            {{Direction::RIGHT, 1, 0}, 3},
            {{Direction::RIGHT, 1, 1}, 4},
            {{Direction::RIGHT, 1, 2}, 5},

            {{Direction::LEFT, 0, 0}, 0},
            {{Direction::LEFT, 0, -1}, 1},
            {{Direction::LEFT, 0, -2}, 2},
            {{Direction::LEFT, -1, 0}, 3},
            {{Direction::LEFT, -1, -1}, 4},
            {{Direction::LEFT, -1, -2}, 5},

            {{Direction::UP, 0, 0}, 0},
            {{Direction::UP, -1, 0}, 1},
            {{Direction::UP, -2, 0}, 2},
            {{Direction::UP, 0, 1}, 3},
            {{Direction::UP, -1, 1}, 4},
            {{Direction::UP, -2, 1}, 5},

            {{Direction::DOWN, 0, 0}, 0},
            {{Direction::DOWN, 1, 0}, 1},
            {{Direction::DOWN, 2, 0}, 2},
            {{Direction::DOWN, 0, -1}, 3},
            {{Direction::DOWN, 1, -1}, 4},
            {{Direction::DOWN, 2, -1}, 5},
        };
        const int x_diff = p.x - pos.x;
        const int y_diff = p.y - pos.y;
        const auto dir = this->direction;
        const auto iter = id_map.find({dir, x_diff, y_diff});

        log_assert(iter != id_map.end(), "id_map not contain key(%d,%d,%d)", dir, x_diff, y_diff);
        return iter->second;
    }

    void set_target_berth_id(int target_berth_id) {
        this->target_berth_id = target_berth_id;
        this->target_delivery_id = -1;
    }

    void set_target_delivery_id(int target_delivery_id) {
        this->target_delivery_id = target_delivery_id;
        this->target_berth_id = -1;
    }

    void clear_flags() {
        this->next_pos_before_collison = invalid_point;
        this->next_direction_before_collison = Direction::UP;
        this->next_command_before_collison = ShipCommand::IDLE;

        this->next_pos_after_collison = invalid_point;
        this->next_direction_after_collison = Direction::UP;
        this->next_command_after_collison = ShipCommand::IDLE;
    }

    Point get_next_pos() const {
        if (this->next_pos_after_collison == invalid_point) {
            return this->next_pos_before_collison;
        }
        return this->next_pos_after_collison;
    }

    Direction::Direction get_next_direction() const {
        if (this->next_pos_after_collison == invalid_point) {
            return this->next_direction_before_collison;
        }
        return this->next_direction_after_collison;
    }

    ShipCommand get_next_command() const {
        if (this->next_pos_after_collison == invalid_point) {
            return this->next_command_before_collison;
        }
        return this->next_command_after_collison;
    }

    void update_ship_next_pos(std::function<bool(const Point& p)> is_barrier) {
        log_assert(path.size() > 0, "path is empty");

        auto ship_head = get_ship_head();
        log_debug("ship[%d],update_ship_next_pos,ship_head:%s", this->id,
                  ship_head.to_string().c_str());
        // 下一个点相对于当前船头的方向
        const auto pos_dir_list = Direction::calc_direction(ship_head, path.back());

        auto update_func = [&](const Point& next_pos,
                               const Direction::Direction& next_dir,
                               const ShipCommand& next_command) {
            this->next_pos_before_collison = next_pos;
            this->next_direction_before_collison = next_dir;
            this->next_command_before_collison = next_command;
        };

        Direction::Direction pos_dir;
        if (pos_dir_list.size() == 1) {
            pos_dir = pos_dir_list[0];
        }
        else {
            // 有两个方向,选择一个方向
            const bool pos_dir0_is_opposite =
                this->direction == Direction::opposite_direction(pos_dir_list[0]);
            const bool pos_dir1_is_opposite =
                this->direction == Direction::opposite_direction(pos_dir_list[1]);

            if (pos_dir0_is_opposite && !pos_dir1_is_opposite) {
                pos_dir = pos_dir_list[1];
            }
            else if (!pos_dir0_is_opposite && pos_dir1_is_opposite) {
                pos_dir = pos_dir_list[0];
            }
            else {
                // 两个方向都是相反方向,或者都不是相反方向随机选一个
                pos_dir = pos_dir_list[std::rand() % pos_dir_list.size()];
            }
        }

        if (pos_dir == this->direction) {
            // 1.方向相同,前进

            update_func(Direction::move(this->pos, this->direction), this->direction,
                        ShipCommand::GO);
        }
        else if (pos_dir == Direction::opposite_direction(this->direction)) {
            // 2. 方向相反,随便找一个方向旋转
            bool clockwise = rand() % 2 == 0;

            const auto [next_pos, next_dir] =
                calc_ship_pos_after_rotate(this->pos, this->direction, clockwise);
            update_func(next_pos, next_dir,
                        clockwise
                            ? ShipCommand::ROTATE_CLOCKWISE
                            : ShipCommand::ROTATE_COUNTERCLOCKWISE);
        }
        else {
            // 3. 方向相差 90 度,旋转
            const auto rot_dir =
                Direction::calc_rotate_direction(this->direction, pos_dir);

            const auto [next_pos, next_dir] = calc_ship_pos_after_rotate(
                this->pos, this->direction, rot_dir.value() == Direction::CLOCKWISE);

            update_func(next_pos, next_dir,
                        rot_dir.value() == Direction::CLOCKWISE
                            ? ShipCommand::ROTATE_CLOCKWISE
                            : ShipCommand::ROTATE_COUNTERCLOCKWISE);
        }

        auto next_ship_area = this->get_ship_next_area();
        if (!next_ship_area.has_value()) {
            log_trace("ship %d next area is stop point", this->id);
            return;
        }
        std::vector<Point> next_ship_points = next_ship_area.value().to_points();
        bool will_collison =
            std::any_of(next_ship_points.begin(), next_ship_points.end(),
                        [&](const Point& p) { return is_barrier(p); });

        if (will_collison) {
            log_trace("ship %d will collison", this->id);
            if (pos_dir == this->direction) {
                log_trace("ship %d will collison, go to rotate", this->id);
                // 1.方向相同的情况下与地图碰撞,取消直行,改为旋转,旋转方向为目标点相对于船头另一个点(不在同一条直线上)的方向
                const auto& select_ship_head_point =
                    Point::at_same_row_or_col(ship_head.left_top, path.back())
                        ? ship_head.right_bottom
                        : ship_head.left_top;

                log_trace("select_ship_head_point:%d,%d, path_back(%d,%d)",
                          P_ARG(select_ship_head_point), P_ARG(path.back()));
                const auto refined_pos_dir_list = Direction::calc_direction_nocheck(
                    select_ship_head_point, path.back());

                log_assert(refined_pos_dir_list.size() == 2,
                           "refined_pos_dir_list size is not 2");

                // 进行旋转修正
                const auto rot_dir = Direction::calc_rotate_direction(
                    this->direction, this->direction == refined_pos_dir_list[0]
                                         ? refined_pos_dir_list[1]
                                         : refined_pos_dir_list[0]);

                const auto refined_rot_dir = choose_correct_direction(
                    this->pos, this->direction, rot_dir.value(), is_barrier);

                const auto [next_pos, next_dir] =
                    calc_ship_pos_after_rotate(this->pos, this->direction,
                                    refined_rot_dir == Direction::CLOCKWISE);

                update_func(next_pos, next_dir,
                            refined_rot_dir == Direction::CLOCKWISE
                                ? ShipCommand::ROTATE_CLOCKWISE
                                : ShipCommand::ROTATE_COUNTERCLOCKWISE);
            }
            else if (pos_dir != Direction::opposite_direction(this->direction)) {
                log_trace("ship %d will collison, go to move", this->id);

                // 1. 选择往相反的方向旋转
                const auto rot_dir =
                    Direction::calc_rotate_direction(this->direction, pos_dir);
                log_assert(rot_dir.has_value(), "rot_dir not valid");
                const auto rot_dir_opposite =
                    Direction::opposite_rotate(rot_dir.value());

                const auto [next_pos, next_dir] =
                    calc_ship_pos_after_rotate(this->pos, this->direction,
                                    rot_dir_opposite == Direction::CLOCKWISE);
                const auto next_command = rot_dir_opposite == Direction::CLOCKWISE
                                              ? ShipCommand::ROTATE_CLOCKWISE
                                              : ShipCommand::ROTATE_COUNTERCLOCKWISE;
                if (std::rand() % 2 == 0) {
                    // 1. 选择往相反的方向旋转
                    update_func(next_pos, next_dir, next_command);
                }
                else {
                    // 2. 取消旋转，改为前进
                    update_func(Direction::move(this->pos, this->direction),
                                this->direction, ShipCommand::GO);
                }
            }
        }
    }

    static Direction::Rotate
    choose_correct_direction(const Point& pos,
                             const Direction::Direction& cur_dir,
                             const Direction::Rotate& rot_dir,
                             std::function<bool(const Point& p)> is_barrier) {
        const auto [next_pos, next_dir] =
            calc_ship_pos_after_rotate(pos, cur_dir, rot_dir == Direction::CLOCKWISE);

        auto next_ship_area_points = calc_ship_area_after_rotate(next_pos, next_dir).to_points();

        bool will_collision =
            std::any_of(next_ship_area_points.begin(), next_ship_area_points.end(),
                        [&](const Point& p) { return is_barrier(p); });

        if (will_collision) {
            return Direction::opposite_rotate(rot_dir);
        }
        else {
            return rot_dir;
        }
    }

    int capacity_percent() const { return this->goods_list.size() * 100 / this->capacity; }

    int cur_value() const {
        return std::accumulate(this->goods_list.begin(), this->goods_list.end(), 0,
                               [](int sum, const Goods& goods) { return sum + goods.money; });
    }

    bool normal_status() const { return this->status == 0; }

    bool recover_status() const { return this->status == 1; }

    bool load_status() const { return this->status == 2; }

    bool can_operate() const {
        return !(this->recover_status()  || fsm == ShipFSM::DEAD);
    }

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
    std::optional<Area> get_ship_next_head() const {
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
    static Area calc_ship_head(const Point& pos, const Direction::Direction dir) {
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
        }
        else {
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
    Area get_ship_area() const { return calc_ship_area_after_rotate(this->pos, this->direction); }

    /**
     * @brief 得到下一个位置的区域,如果下一个位置是停止点,返回std::nullopt
     *
     * @return std::optional<Area>
     */
    std::optional<Area> get_ship_next_area() const {
        const auto next_pos = this->get_next_pos();
        const auto next_dir = this->get_next_direction();

        if (Point::is_stop_point(next_pos)) {
            return std::nullopt;
        }
        return calc_ship_area_after_rotate(next_pos, next_dir);
    }

    /**
     * @brief 计算旋转后船的区域位置
     *
     * @param pos
     * @param dir
     * @return Area
     */
    static Area calc_ship_area_after_rotate(const Point& pos, Direction::Direction dir) {
        switch (dir) {
        case Direction::RIGHT: {
            return {pos, Point(pos.x + 1, pos.y + 2)};
        }
        case Direction::LEFT: {
            return {{pos.x - 1, pos.y - 2}, pos};
        }
        case Direction::UP: {
            return {
                {pos.x - 2, pos.y},
                {pos.x, pos.y + 1}
            };
        }
        case Direction::DOWN:
            return {
                {pos.x, pos.y - 1},
                {pos.x + 2, pos.y}
            };
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
    calc_ship_pos_after_rotate(const Point& pos, const Direction::Direction dir,
                    bool clockwise_direction) {
        switch (dir) {
        case Direction::RIGHT: {
            if (clockwise_direction) {
                return std::make_pair(Point(pos.x, pos.y + 2), Direction::DOWN);
            }
            else {
                return std::make_pair(Point(pos.x + 1, pos.y + 1), Direction::UP);
            }
        }
        case Direction::LEFT: {
            if (clockwise_direction) {
                return std::make_pair(Point(pos.x, pos.y - 2), Direction::UP);
            }
            else {
                return std::make_pair(Point(pos.x - 1, pos.y - 1), Direction::DOWN);
            }
        }
        case Direction::UP: {
            if (clockwise_direction) {
                return std::make_pair(Point(pos.x - 2, pos.y), Direction::RIGHT);
            }
            else {
                return std::make_pair(Point(pos.x - 1, pos.y + 1), Direction::LEFT);
            }
        }
        case Direction::DOWN:
            if (clockwise_direction) {
                return std::make_pair(Point(pos.x + 2, pos.y), Direction::LEFT);
            }
            else {
                return std::make_pair(Point(pos.x + 1, pos.y - 1), Direction::RIGHT);
            }
        }

        log_assert(false, "calc_rot_action error");
    }

    void load(const Goods& goods) {
        this->goods_list.emplace_back(goods);
    }

    void unload() {
        log_assert(this->berth_id == -1, "berth_id is not -1, %d", this->berth_id);

        log_trace("Ship %d unload, cur_capacity: %d, cur_value: %d", this->id,
                  this->capacity_percent(), this->cur_value());
        this->goods_list.clear();
    }

    bool full() const { return this->goods_list.size() >= this->capacity; }

    bool empty() const { return this->goods_list.empty(); }

    int remain_capacity() const {
        const int ret = this->capacity - this->goods_list.size();
        log_assert(ret >= 0, "remain_capacity is less than 0, %d", ret);
        return ret;
    }

    explicit Ship(const int id, const int cur_capacity, const int max_capacity, const Point& pos,
                  const Direction::Direction direction, const int status, const int start_cycle)
        : id(id), cur_capacity(cur_capacity), capacity(max_capacity),
          status(status), pos(pos), direction(direction), start_cycle(start_cycle) {}

    explicit Ship() {
        this->id = 0;
        this->capacity = 0;
        this->status = 0;
        this->berth_id = -1;
    }
};
