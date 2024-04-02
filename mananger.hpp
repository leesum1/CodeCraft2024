// #pragma once

// #include "goods.hpp"
// #include "io_layer.hpp"
// #include "log.h"
// #include "point.hpp"
// #include "tools.hpp"
// #include <algorithm>
// #include <array>
// #include <cmath>
// #include <cstdlib>
// #include <unordered_map>
// #include <unordered_set>
// #include <utility>
// #include <vector>

// const int max_robots_num_in_berth = 3;

// class Manager {

// public:
//   IoLayer io_layer;

//   std::array<std::vector<Point>, ROBOT_NUM> robots_path_list;
//   std::array<Point, ROBOT_NUM> robots_cur_pos_list;
//   std::array<Point, ROBOT_NUM> robots_next_pos_list;
//   std::array<Point, ROBOT_NUM> robots_next_pos_list_copy;
//   std::vector<int> robots_need_move_but_not_move;
//   // std::array<int, ROBOT_NUM> berths_id_list;
//   std::array<bool, ROBOT_NUM> robots_get_action{};
//   std::array<bool, ROBOT_NUM> robots_pull_action{};
//   std::array<Goods, ROBOT_NUM> robots_target_goods_list;
//   std::array<bool, ROBOT_NUM> robots_has_goods{};
//   std::array<bool, ROBOT_NUM> robots_is_dead{};
//   std::array<bool, ROBOT_NUM> robots_first_get_goods{};
//   std::array<bool, 10> robots_has_pass_collision{};
//   std::array<int, ROBOT_NUM> robots_idle_cycle{};
//   std::array<bool, 10> berths_visit{};
//   std::vector<int> cycle1_berths_visit;

//   int search_count = 0;

//   Manager() = default;
//   ~Manager() = default;
//   void init_game() { io_layer.init(); }

//   auto get_is_barrier_lambda_v3(const int robot_id) {
//     auto is_barrier = [&](const Point &p) {
//       bool is_barrier1 = io_layer.game_map.is_barrier(p);
//       //
//       bool is_barrier2 = false;

//       // 与其他机器人的当前位置碰撞
//       for (int i = 0; i < ROBOT_NUM; i++) {
//         if (i == robot_id) {
//           continue;
//         }
//         if (p == robots_cur_pos_list[i]) {
//           is_barrier2 = true;
//           break;
//         }
//       }

//       bool is_barrier3 = false;
//       // is_barrier3 =
//       //     low_prio_next_pos_collision_with_others_next_pos(robot_id, p);

//       // // 与优先级高的机器人的下一个位置碰撞
//       for (int i = 0; i < ROBOT_NUM; i++) {
//         if (i == robot_id) {
//           continue;
//         }
//         if (p == robots_next_pos_list[i]) {
//           is_barrier3 = true;
//           break;
//         }
//       }
//       return is_barrier1 || is_barrier2 || is_barrier3;
//     };
//     return is_barrier;
//   }

//   auto get_is_barrier_lambda_v2(const int robot_id) {
//     auto is_barrier = [&](const Point &p) {
//       bool is_barrier1 = io_layer.game_map.is_barrier(p);
//       //
//       bool is_barrier2 = false;

//       // 与其他机器人的当前位置碰撞
//       for (int i = 0; i < ROBOT_NUM; i++) {
//         if (i == robot_id) {
//           continue;
//         }
//         if (p == robots_cur_pos_list[i]) {
//           is_barrier2 = true;
//           break;
//         }
//       }

//       bool is_barrier3 = false;
//       // is_barrier3 =
//       //     low_prio_next_pos_collision_with_others_next_pos(robot_id, p);

//       // // 与优先级高的机器人的下一个位置碰撞
//       for (int i = 0; i < ROBOT_NUM; i++) {
//         if (i == robot_id) {
//           continue;
//         }
//         if (robots_has_pass_collision[i] == false) {
//           continue;
//         }
//         if (p == robots_next_pos_list[i]) {
//           is_barrier3 = true;
//           break;
//         }
//       }
//       return is_barrier1 || is_barrier2 || is_barrier3;
//     };
//     return is_barrier;
//   }

//   auto get_is_barrier_lambda_v1(const int robot_id) {
//     auto is_barrier = [&](const Point &p) {
//       bool is_barrier1 = io_layer.game_map.is_barrier(p);
//       //
//       bool is_barrier2 = false;

//       is_barrier2 =
//           low_prio_next_pos_collision_with_others_cur_pos(robot_id, p);

//       // // 与其他机器人的当前位置碰撞
//       // for (int i = 0; i < ROBOT_NUM; i++) {
//       //   if (i == robot_id) {
//       //     continue;
//       //   }
//       //   if (p == robots_cur_pos_list[i]) {
//       //     is_barrier2 = true;
//       //     break;
//       //   }
//       // }

//       bool is_barrier3 = false;
//       is_barrier3 =
//           low_prio_next_pos_collision_with_others_next_pos(robot_id, p);

//       // // 与优先级高的机器人的下一个位置碰撞
//       // for (int i = 0; i < ROBOT_NUM; i++) {
//       //   if (i == robot_id) {
//       //     continue;
//       //   }
//       //   if (robots_has_pass_collision[i] == false) {
//       //     continue;
//       //   }
//       //   if (p == robots_next_pos_list[i]) {
//       //     is_barrier3 = true;
//       //     break;
//       //   }
//       // }
//       return is_barrier1 || is_barrier2 || is_barrier3;
//     };
//     return is_barrier;
//   }

//   // ---------------------------------------
//   // 障碍物检测
//   // 1. 地图障碍物
//   // 2. 机器人当前位置
//   // 3. 机器人下一个位置
//   auto get_is_barrier_lambda() {
//     auto is_barrier = [&](const Point &p) {
//       bool is_barrier1 = io_layer.game_map.is_barrier(p);
//       bool is_barrier2 =
//           std::any_of(robots_cur_pos_list.begin(), robots_cur_pos_list.end(),
//                       [&](Point _pos) { return p == _pos; });
//       bool is_barrier3 =
//           std::any_of(robots_next_pos_list.begin(), robots_next_pos_list.end(),
//                       [&](Point _pos) { return p == _pos; });
//       return is_barrier1 || is_barrier2 || is_barrier3;
//     };
//     return is_barrier;
//   }

//   auto get_find_neighbor_lambda() {
//     auto find_neighbor = [&](const Point &p) {
//       return io_layer.game_map.neighbors(p, false);
//     };
//     return find_neighbor;
//   }

//   // 碰撞检测(采用剪切法)
//   void check_collision(int robot_id) {

//     const Point robot_next_pos = robots_next_pos_list[robot_id];
//     const Point robot_cur_pos = robots_cur_pos_list[robot_id];
//     log_trace("robot[%d] check_collision start", robot_id);
//     log_trace("robot[%d] robot_cur_pos(%d,%d) robot_next_pos(%d,%d)", robot_id,
//               P_ARG(robot_cur_pos), P_ARG(robot_next_pos));

//     if (robot_next_pos == invalid_point || robot_next_pos == stop_point) {
//       // 不动就不需要检测
//       log_trace("robot[%d] robot_next_pos is null, no need check_collision",
//                 robot_id);
//       return;
//     }

//     // 将当前机器人与其他机器人进行碰撞检测
//     for (int i = 0; i < ROBOT_NUM; i++) {
//       if (i == robot_id) {
//         // 不需要检测自己
//         continue;
//       }

//       if (robot_next_pos != robots_cur_pos_list[i] &&
//           robot_next_pos != robots_next_pos_list[i]) {
//         // 当前机器人的下一步不是其他机器人的位置或下一步
//         continue;
//       }

//       log_info("robot[%d] and robot[%d] collision", robot_id, i);

//       bool test_success;
//       auto come_from_t = PATHHelper::cut_path(
//           robot_cur_pos, get_is_barrier_lambda(), get_find_neighbor_lambda(),
//           robots_path_list[robot_id], 20, test_success);

//       if (test_success) {

//         // 更新下一步位置
//         robots_next_pos_list.at(robot_id) = robots_path_list[robot_id].back();
//         return;
//       } else {
//         // 能不能再利用 BFS 的结果，找到一个路径，需要一个终点
//         // 需要另外的策略
//         // 1. 找不到路就停止 (在狭窄的地方会死锁)
//         // 2. 随便找块空地

//         // 获取与当前机器人碰撞机器人的路径
//         const auto &collison_path_set =
//             Tools::vector_to_set(robots_path_list[i]);
//         const auto colilison_pos = robots_cur_pos_list[i];
//         const auto colilison_next_pos = robots_next_pos_list[i];

//         Point bt_point = invalid_point;
//         bool bt_point_founded = false;
//         int bt_cost = 10000;
//         // 找到一个没有在碰撞机器人路径的空地
//         const int r_size = 1 + std::rand() % 4;
//         for (const auto &pair : come_from_t) {
//           if (collison_path_set.find(pair.first) == collison_path_set.end()) {
//             if (pair.first.x == colilison_pos.x ||
//                 pair.first.y == colilison_pos.y) {
//               continue;
//             }
//             if (pair.first.x == colilison_next_pos.x ||
//                 pair.first.y == colilison_next_pos.y) {
//               continue;
//             }

//             if (pair.second.cost < bt_cost && (pair.second.cost > r_size)) {
//               bt_cost = pair.second.cost;
//               bt_point = pair.first;
//               bt_point_founded = true;
//             }
//           }
//         }

//         if (bt_point_founded) {

//           // if (robot_id < i) {
//           //   log_info("robot[%d] stop move wait path", robot_id);
//           //   // for (int i = 0; i < 5; i++) {
//           //   //   robots_path_list[robot_id].push_back(stop_point);
//           //   // }
//           //   robots_next_pos_list.at(robot_id) = invalid_point;
//           //   return;
//           // }
//           if (bt_point == robot_cur_pos) {
//             log_info("robot[%d] bt_point == robot_cur_pos not move", robot_id);
//             // for (int i = 0; i < 5; i++) {
//             //   robots_path_list[robot_id].push_back(stop_point);
//             // }
//             robots_next_pos_list.at(robot_id) = invalid_point;
//             return;
//           }

//           log_trace("robot[%d] find random space(%d,%d)", robot_id,
//                     P_ARG(bt_point));
//           bool bt_success;
//           const auto bt_path = PATHHelper::get_path(robot_cur_pos, bt_point,
//                                                     come_from_t, bt_success);
//           log_assert(bt_success, "error, bt_success is false");

//           PATHHelper::add_backtrace_path(robot_cur_pos,
//                                          robots_path_list[robot_id], bt_path);

//           // 更新下一步位置
//           robots_next_pos_list.at(robot_id) = robots_path_list[robot_id].back();
//           // log_info("robot[%d],add bt path size:%d",robot_id, bt_path.size());
//           // for (const auto &p : robots_path_list[robot_id]) {
//           //   log_info("robot[%d] bt_path (%d,%d)", robot_id, P_ARG(p));
//           // }
//           // assert(0);
//           return;
//         }

//         const auto max_point =
//             std::max_element(come_from_t.begin(), come_from_t.end(),
//                              [&](const auto &p1, const auto &p2) {
//                                return p1.second.cost < p2.second.cost;
//                              });

//         const auto &random_point = max_point->first;

//         bool success2 = false;
//         const std::vector<Point> path_last = PATHHelper::get_path(
//             robot_cur_pos, random_point, come_from_t, success2);

//         if (path_last.empty()) {
//           // 找不到路就停止,可能会死锁? (可以优化吗,
//           // 再次寻小路去周围的空地?)
//           log_info("robot[%d] new path not found, stop move", robot_id);
//           robots_next_pos_list.at(robot_id) = Point();
//           return;
//         } else {

//           // 最对去 path_last 的五个点
//           // robots_path_list[robot_id] = path_last;

//           robots_path_list[robot_id] = Tools::last_n(path_last, 5);

//           // 放弃当前路径，去新的空地避免死锁
//           log_trace("robot[%d] give up current path, go to new space(%d,%d) "
//                     "size:%d",
//                     robot_id, P_ARG(path_last.back()), path_last.size());
//           robots_next_pos_list.at(robot_id) = path_last.back();

//           // 如果机器人有预定的货物(还没有拿到),则取消预定
//           if ((robots_target_goods_list[robot_id].status ==
//                GoodsStatus::Booked) &&
//               robots_has_goods[robot_id] == false) {

//             const auto &target_goods = robots_target_goods_list[robot_id];
//             log_debug("robot[%d] give up current target_good"
//                       "(%d,%d) status:%d, money:%d,end_cycle:%d",
//                       robot_id, P_ARG(target_goods.pos), target_goods.status,
//                       target_goods.money, target_goods.end_cycle);

//             // 取消预定状态
//             io_layer.map_goods_list.at(target_goods.pos).status =
//                 GoodsStatus::Normal;

//             robots_target_goods_list[robot_id] = invalid_goods;
//           }
//           return;
//         }
//       }
//     }
//     log_debug("robot[%d] check_collision end", robot_id);
//   }
//   // 当前机器人为高优先级
//   int high_prio_next_pos_collision_with_others_cur_pos(const int robot_id) {
//     const Point &robot_next_pos = robots_next_pos_list[robot_id];
//     if (robot_next_pos == invalid_point) {
//       return -1;
//     }
//     if (robot_next_pos == stop_point) {
//       return -1;
//     }
//     for (int i = 0; i < ROBOT_NUM; i++) {
//       if (i == robot_id) {
//         // 不需要与自己检查
//         continue;
//       }

//       if (robots_has_pass_collision[i]) {
//         // 只和没有通过碰撞检测的机器人检测
//         continue;
//       }

//       if (robot_next_pos == robots_cur_pos_list[i]) {
//         return i;
//       }
//     }

//     return -1;
//   }

//   // 当前机器人为高优先级
//   int high_prio_next_pos_collision_with_others_cur_pos_step2(
//       const int robot_id) {
//     const Point &robot_next_pos = robots_next_pos_list_copy[robot_id];
//     if (robot_next_pos == invalid_point) {
//       return -1;
//     }
//     if (robot_next_pos == stop_point) {
//       return -1;
//     }
//     for (int i = 0; i < ROBOT_NUM; i++) {
//       if (i == robot_id) {
//         // 不需要与自己检查
//         continue;
//       }

//       if (robot_next_pos == robots_cur_pos_list[i]) {
//         return i;
//       }
//     }

//     return -1;
//   }

//   // 当前机器人为高优先级
//   bool high_prio_next_pos_collision_with_others_next_pos(const int robot_id) {
//     const Point &robot_next_pos = robots_next_pos_list[robot_id];

//     if (robot_next_pos == invalid_point) {
//       return false;
//     }
//     if (robot_next_pos == stop_point) {
//       return false;
//     }

//     for (int i = 0; i < ROBOT_NUM; i++) {
//       if (i == robot_id) {
//         // 不需要与自己检查
//         continue;
//       }

//       if (robots_has_pass_collision[i]) {
//         // 只和没有通过碰撞检测的机器人检测
//         continue;
//       }

//       if (robot_next_pos == robots_next_pos_list[i]) {
//         return true;
//       }
//     }

//     return false;
//   }
//   // 当前机器人为低优先级
//   std::vector<int>
//   low_prio_next_pos_collision_with_others_cur_pos(const int robot_id) {
//     const Point &robot_next_pos = robots_next_pos_list[robot_id];

//     std::vector<int> robots_list{};
//     if (robot_next_pos == invalid_point) {
//       return robots_list;
//     }
//     if (robot_next_pos == stop_point) {
//       return robots_list;
//     }

//     for (int i = 0; i < ROBOT_NUM; i++) {
//       if (i == robot_id) {
//         // 不需要与自己检查
//         continue;
//       }

//       if (!robots_has_pass_collision[i]) {
//         // 只和通过碰撞检测的机器人检测
//         continue;
//       }

//       if (robot_next_pos == robots_cur_pos_list[i]) {
//         robots_list.emplace_back(i);
//       }
//     }

//     return robots_list;
//   }

//   // 当前机器人为低优先级
//   bool low_prio_next_pos_collision_with_others_cur_pos(const int robot_id,
//                                                        const Point &p) {

//     if (p == invalid_point) {
//       return false;
//     }
//     if (p == stop_point) {
//       return false;
//     }

//     for (int i = 0; i < ROBOT_NUM; i++) {
//       if (robot_id == i) {
//         continue;
//       }

//       if (!robots_has_pass_collision[i]) {
//         // 只和通过碰撞检测的机器人检测
//         continue;
//       }

//       if (p == robots_cur_pos_list[i]) {
//         return true;
//       }
//     }

//     return false;
//   }
//   // 当前机器人为低优先级
//   bool low_prio_next_pos_collision_with_others_next_pos(const int robot_id,
//                                                         const Point &p) {

//     if (p == invalid_point) {
//       return false;
//     }
//     if (p == stop_point) {
//       return false;
//     }

//     for (int i = 0; i < ROBOT_NUM; i++) {
//       if (robot_id == i) {
//         continue;
//       }

//       if (!robots_has_pass_collision[i]) {
//         // 只和通过碰撞检测的机器人检测
//         continue;
//       }

//       if (p == robots_next_pos_list[i]) {
//         return true;
//       }
//     }

//     return false;
//   }

//   // 当前机器人为低优先级
//   std::vector<int>
//   low_prio_next_pos_collision_with_others_next_pos(const int robot_id) {
//     const Point &robot_next_pos = robots_next_pos_list[robot_id];
//     std::vector<int> robots_list{};
//     if (robot_next_pos == invalid_point) {
//       return robots_list;
//     }
//     if (robot_next_pos == stop_point) {
//       return robots_list;
//     }

//     for (int i = 0; i < ROBOT_NUM; i++) {
//       if (i == robot_id) {
//         // 不需要与自己检查
//         continue;
//       }

//       if (!robots_has_pass_collision[i]) {
//         // 只和通过碰撞检测的机器人检测
//         continue;
//       }

//       if (robot_next_pos == robots_next_pos_list[i]) {
//         robots_list.emplace_back(i);
//       }
//     }

//     return robots_list;
//   }

//   // 当前机器人为低优先级
//   std::vector<Point>
//   low_prio_cur_pos_collision_with_others_next_pos(const int robot_id) {
//     const Point &robot_cur_pos = robots_cur_pos_list[robot_id];

//     std::vector<Point> next_pos_list{};
//     if (robot_cur_pos == invalid_point) {
//       return next_pos_list;
//     }
//     if (robot_cur_pos == stop_point) {
//       return next_pos_list;
//     }

//     for (int i = 0; i < ROBOT_NUM; i++) {
//       if (i == robot_id) {
//         // 不需要与自己检查
//         continue;
//       }

//       if (!robots_has_pass_collision[i]) {
//         // 只和通过碰撞检测的机器人检测
//         continue;
//       }

//       if (robot_cur_pos == robots_next_pos_list[i]) {
//         next_pos_list.emplace_back(robots_next_pos_list[i]);
//       }
//     }

//     return next_pos_list;
//   }

//   // 碰撞检测(采用剪切法)
//   void check_collision_v1(int robot_id) {
//     robots_has_pass_collision[robot_id] = true;

//     const Point robot_next_pos = robots_next_pos_list[robot_id];
//     const Point robot_cur_pos = robots_cur_pos_list[robot_id];
//     log_trace("robot[%d] check_collision start  robot_cur_pos(%d,%d) "
//               "robot_next_pos(%d,%d)",
//               robot_id, P_ARG(robot_cur_pos), P_ARG(robot_next_pos));

//     if (robot_next_pos == invalid_point || robot_next_pos == stop_point) {
//       // 不动就不需要检测
//       log_trace("robot[%d] robot_next_pos is null, no need check_collision",
//                 robot_id);
//       return;
//     }

//     auto other_high_collision_robot_id =
//         low_prio_next_pos_collision_with_others_cur_pos(robot_id);
//     if (!other_high_collision_robot_id.empty()) {
//       // 1. 下一次移动的位置与优先级高的机器人的当前位置冲突:
//       // 选择一个方向移走(为优先级高的开路)

//       for (const int id_tmp : other_high_collision_robot_id) {
//         log_debug(
//             "robot[%d] low_prio_next_pos_collision_with_others[%d]_cur_pos",
//             robot_id, id_tmp);
//       }

//       bool cut_success = false;
//       auto come_from_t = PATHHelper::cut_path(
//           robot_cur_pos, get_is_barrier_lambda_v1(robot_id),
//           get_find_neighbor_lambda(), robots_path_list[robot_id], 10,
//           cut_success);
//       if (cut_success) {
//         robots_next_pos_list.at(robot_id) = robots_path_list[robot_id].back();
//         log_debug("robot[%d] cut path success", robot_id);
//       } else {
//         const auto others_next_pos =
//             low_prio_cur_pos_collision_with_others_next_pos(robot_id);

//         std::vector<Point> other_cur_pos_list(
//             other_high_collision_robot_id.size());

//         std::transform(other_high_collision_robot_id.begin(),
//                        other_high_collision_robot_id.end(),
//                        other_cur_pos_list.begin(),
//                        [&](const int id) { return robots_cur_pos_list[id]; });

//         const auto flag_tmp =
//             std::any_of(other_high_collision_robot_id.begin(),
//                         other_high_collision_robot_id.end(), [&](const int id) {
//                           return robots_next_pos_list[id] == invalid_point;
//                         });

//         if (!others_next_pos.empty() || flag_tmp) {
//           Point select_space = invalid_point;
//           int select_cost = 10000;
//           auto is_barrier = get_is_barrier_lambda_v1(robot_id);

//           for (const auto &p : come_from_t) {
//             if (!is_barrier(p.first)) {
//               if (std::any_of(other_cur_pos_list.begin(),
//                               other_cur_pos_list.end(), [&](const Point &pos) {
//                                 return Point::at_same_row_or_col(pos, p.first);
//                               })) {
//                 continue;
//               }

//               if (p.second.cost < select_cost) {
//                 select_space = p.first;
//                 select_cost = p.second.cost;
//               }
//             }
//           }
//           if (select_space == invalid_point) {
//             // 没有空地可以走了, 高优先级需要让路, 需要设置一个标识位
//             robots_next_pos_list[robot_id] = invalid_point;

//             robots_need_move_but_not_move.insert(
//                 robots_need_move_but_not_move.end(),
//                 other_high_collision_robot_id.begin(),
//                 other_high_collision_robot_id.end());

//             for (const int id_tmp : other_high_collision_robot_id) {
//               log_debug("robot[%d] no space move,other robot[%d] need move",
//                         robot_id, id_tmp);
//             }

//           } else {
//             bool bt_success;

//             const auto bt_path = PATHHelper::get_path(
//                 robot_cur_pos, select_space, come_from_t, bt_success);
//             log_assert(bt_success, "error, bt_success is false");

//             PATHHelper::add_backtrace_path(robot_cur_pos,
//                                            robots_path_list[robot_id], bt_path);
//             robots_next_pos_list.at(robot_id) =
//                 robots_path_list[robot_id].back();
//             log_debug("robot[%d] add bt path size:%d", robot_id,
//                       bt_path.size());
//           }
//         } else {
//           robots_next_pos_list[robot_id] = invalid_point;
//           log_debug("robot[%d] no need move", robot_id);
//         }
//       }
//     }

//     other_high_collision_robot_id =
//         low_prio_next_pos_collision_with_others_next_pos(robot_id);
//     if (!other_high_collision_robot_id.empty()) {
//       log_debug("robot[%d],low_prio_next_pos_collision_with_others_next_pos",
//                 robot_id);
//       //   2. 下一次移动的位置与优先级高的机器人的下一次位置冲突: 取消当前移动
//       //   (给优先级高的让路) (是否可以选择一个方向移走?)

//       std::vector<Point> path_copy =
//           std::vector<Point>(robots_path_list[robot_id]);

//       bool cut_success = false;
//       auto come_from_t = PATHHelper::cut_path(
//           robot_cur_pos, get_is_barrier_lambda_v1(robot_id),
//           get_find_neighbor_lambda(), path_copy, 10, cut_success);
//       if (cut_success) {

//         robots_next_pos_list[robot_id] = invalid_point;
//       } else {
//         const auto others_next_pos =
//             low_prio_cur_pos_collision_with_others_next_pos(robot_id);

//         std::vector<Point> other_cur_pos_list(
//             other_high_collision_robot_id.size());

//         std::transform(other_high_collision_robot_id.begin(),
//                        other_high_collision_robot_id.end(),
//                        other_cur_pos_list.begin(),
//                        [&](const int id) { return robots_cur_pos_list[id]; });

//         Point select_space = invalid_point;
//         int select_cost = 10000;
//         auto is_barrier = get_is_barrier_lambda_v1(robot_id);

//         for (const auto &p : come_from_t) {
//           if (!is_barrier(p.first)) {
//             if (std::any_of(other_cur_pos_list.begin(),
//                             other_cur_pos_list.end(), [&](const Point &pos) {
//                               return Point::at_same_row_or_col(pos, p.first);
//                             })) {
//               continue;
//             }

//             if (p.second.cost < select_cost) {
//               select_space = p.first;
//               select_cost = p.second.cost;
//             }
//           }
//         }
//         if (select_space == invalid_point) {
//           robots_next_pos_list[robot_id] = invalid_point;

//           for (const int id_tmp : other_high_collision_robot_id) {
//             log_debug("robot[%d] no space move,other robot[%d] need move",
//                       robot_id, id_tmp);
//           }

//         } else {
//           bool bt_success;

//           const auto bt_path = PATHHelper::get_path(robot_cur_pos, select_space,
//                                                     come_from_t, bt_success);
//           log_assert(bt_success, "error, bt_success is false");

//           PATHHelper::add_backtrace_path(robot_cur_pos,
//                                          robots_path_list[robot_id], bt_path);
//           robots_next_pos_list.at(robot_id) = robots_path_list[robot_id].back();
//           log_debug("robot[%d] add bt path size:%d", robot_id, bt_path.size());
//         }
//       }
//     }

//     if (high_prio_next_pos_collision_with_others_next_pos(robot_id)) {
//       log_debug("robot[%d],high_prio_next_pos_collision_with_others_next_pos",
//                 robot_id);
//       // 下一次移动位置与优先级低的下一次移动的位置冲突:继续移动(因为优先级低的会取消本次移动)
//     }

//     int other_collision_robot_id =
//         high_prio_next_pos_collision_with_others_cur_pos(robot_id);

//     if (other_collision_robot_id != -1) {
//       log_debug("robot[%d],high_prio_next_pos_collision_with_other[%d]_cur_pos",
//                 robot_id, other_collision_robot_id);

//       // 下一次移动位置与优先级低的机器人当前位置冲突: 停止当前移动,
//       // 原地不动(在下一个周期,其他优先级低的会让出位置)

//       robots_next_pos_list[robot_id] = invalid_point;
//     }
//   }

//   void check_collison_step2(const int robot_id) {

//     const auto robot_cur_pos = robots_cur_pos_list[robot_id];
//     int other_collision_robot_id =
//         high_prio_next_pos_collision_with_others_cur_pos_step2(robot_id);

//     if (other_collision_robot_id == -1) {
//       return;
//     }

//     // 如果低优先级的机器人不能移动,则高优先级的机器人移动
//     if (std::find(robots_need_move_but_not_move.begin(),
//                   robots_need_move_but_not_move.end(),
//                   robot_id) != robots_need_move_but_not_move.end()) {

//       bool cut_success = false;
//       auto come_from_t = PATHHelper::cut_path(
//           robot_cur_pos, get_is_barrier_lambda_v3(robot_id),
//           get_find_neighbor_lambda(), robots_path_list[robot_id], 10,
//           cut_success);

//       Point select_space = invalid_point;
//       int select_cost = 10000;
//       auto is_barrier = get_is_barrier_lambda_v3(robot_id);
//       const auto other_cur_pos = robots_cur_pos_list[other_collision_robot_id];

//       for (const auto &p : come_from_t) {
//         if (!is_barrier(p.first)) {
//           if (p.first.x == other_cur_pos.x || p.first.y == other_cur_pos.y) {
//             continue;
//           }

//           if (p.second.cost < select_cost && p.second.cost > 1) {
//             select_space = p.first;
//             select_cost = p.second.cost;
//           }
//         }
//       }
//       if (select_space == invalid_point) {
//         // 没有空地可以走了
//         robots_next_pos_list[robot_id] = invalid_point;
//         log_debug("robot[%d] check_colliso with [%d]no space to move", robot_id,
//                   other_collision_robot_id);
//       } else {
//         bool bt_success;

//         const auto bt_path = PATHHelper::get_path(robot_cur_pos, select_space,
//                                                   come_from_t, bt_success);
//         log_assert(bt_success, "error, bt_success is false");

//         PATHHelper::add_backtrace_path(robot_cur_pos,
//                                        robots_path_list[robot_id], bt_path);
//         robots_next_pos_list.at(robot_id) = robots_path_list[robot_id].back();
//       }
//     }
//   }

//   // void err_debug(int robot_id){
//   //     // if (!io_layer.is_valid_move(robots_cur_pos_list[robot_id],
//   //     //                             robots_next_pos_list[robot_id])) {
//   //     //   log_trace("robot[%d] invalid move, path size", robot_id);

//   //     //   for (auto pos : robots_path_list[robot_id]) {
//   //     //     log_trace("robot[%d] path (%d,%d)", robot_id, pos.x, pos.y);
//   //     //   }
//   //     // }
//   // };

//   void berth_cycle() {
//     for (int i = 0; i < BERTH_NUM; i++) {
//       io_layer.berths[i].tick(io_layer.cur_cycle);
//     }
//   }

//   // 选择价值最高的泊位
//   int select_best_berth() {
//     int target_berth_id = -1;
//     int max_value = -1;

//     const int remine_time = 15000 - (io_layer.cur_cycle);
//     for (int i = 0; i < BERTH_NUM; i++) {
//       // 如果泊位已经被占用,则跳过
//       if (berths_visit[i] == true || io_layer.berths[i].is_baned) {
//         continue;
//       }
//       // 选择去价值最高的港口就是最优解
//       const int value = io_layer.berths[i].goods_value() +
//                         io_layer.berths[i].money_in_1000cycle();
//       const int cur_transport_time = io_layer.berths[i].transport_time;

//       if (remine_time - (cur_transport_time * 2 + 1) < 1) {
//         continue;
//       }

//       // float_t load_wight = 2.0 / io_layer.berths[i].loading_speed;
//       // float_t trans_wight = io_layer.berths[i].transport_time / 1000.0;
//       // float_t cur_weight = (value * load_wight * trans_wight);

//       if (value > max_value) {
//         max_value = static_cast<int>(value);
//         target_berth_id = i;
//       }
//     }

//     return target_berth_id;
//   };

//   /**
//    * @brief 获得在港口[berth_id]的机器人数量
//    *
//    * @param berth_id
//    * @return int
//    */
//   int get_berth_robot_num(const int berth_id, const int robot_id) {
//     int num = 0;
//     for (int i = 0; i < ROBOT_NUM; i++) {
//       if (robot_id == i) {
//         continue;
//       }
//       if (io_layer.robots[i].target_berth_id == berth_id) {
//         num++;
//       }
//     }
//     return num;
//   }
//   /**
//    * @brief 获得目的地为港口[berth_id]的船只数量
//    *
//    * @param berth_id
//    * @return int
//    */
//   int get_berth_ship_num(const int berth_id) {
//     int num = 0;
//     for (int i = 0; i < SHIP_NUM; i++) {
//       if (io_layer.ships[i].berth_id == berth_id) {
//         num++;
//       }
//     }
//     return num;
//   }

//   std::pair<Goods, std::vector<Point>>
//   find_best_goods_path_from_berth_v1(const int berth_id, const int robot_id,
//                                      bool &founded) {

//     // 查找到最优的高于平均值的货物
//     Goods goods_final_hi = invalid_goods;
//     float max_weight_hi = 0.0;
//     // 查找到最优的低于平均值的货物
//     Goods goods_final_lo = invalid_goods;
//     float max_weight_lo = 0.0;

//     bool success = false;
//     // 遍历所有的货物,找到最有价值的货物
//     for (auto &goods : io_layer.map_goods_list) {

//       if (goods.second.status != GoodsStatus::Normal) {
//         continue;
//       };

//       // 找不到路径
//       if (io_layer.berths_come_from_set[berth_id].find(goods.second.pos) ==
//           io_layer.berths_come_from_set[berth_id].end()) {
//         continue;
//       }

//       // 从港口到货物的路径距离
//       const auto to_goods_path_cost =
//           io_layer.get_cost_from_berth_to_point(berth_id, goods.first);

//       if (!to_goods_path_cost.has_value()) {
//         continue;
//       }

//       // 判断是否能在货物消失之前拿到货物， + 5 是为了给 cut path 预留时间容错
//       const bool can_fetch_goods = !goods.second.is_disappeared(
//           io_layer.cur_cycle + to_goods_path_cost.value() + 5);

//       const int max_path_size = 10000;

//       if (!can_fetch_goods && to_goods_path_cost > max_path_size) {
//         continue;
//       }
//       // 货物还剩多久消失
//       int goods_remin_time = (goods.second.end_cycle - io_layer.cur_cycle);

//       if (goods_remin_time < 200) {
//         goods_remin_time = 200;
//       }

//       // 从货物到港口的最短路径
//       const auto to_berth_path_cost =
//           io_layer.get_minimum_berth_cost(goods.first);

//       // TODO: 优化权重计算
//       // 都是越小越好

//       // /** 策略1**/
//       // const float to_goods_one =
//       //     to_goods_path_cost.value() / static_cast<float>(max_path_size);

//       // const float goods_remin_time_one =
//       //     goods_remin_time / static_cast<float>(max_path_size);

//       // float cur_weight =
//       //     1 / (0.6 * to_goods_one + 0.017 * goods_remin_time_one);
//       // /** 策略1**/

//       /** 策略2*/
//       const float to_goods_one = 6400.0 / (to_goods_path_cost.value() + 1);
//       const float to_berth_one =
//           to_berth_path_cost.second / static_cast<float>(max_path_size);
//       float goods_remin_time_one =
//           4200000.0 / (goods_remin_time * goods_remin_time + 1);

//       if (to_goods_path_cost.value() > 71) {
//         goods_remin_time_one = goods_remin_time_one / 2;
//       }

//       float cur_weight = to_goods_one + goods_remin_time_one;
//       /** 策略2*/

//       /** 策略3*/
//       // const float to_goods_one = to_goods_path_cost.value();

//       // float cur_weight = 1 / (to_goods_one + 1);
//       /** 策略3*/

//       //**策略4*/
//       // float cur_weight = 1.0 / (to_goods_path_cost.value());

//       if (io_layer.final_time) {
//         cur_weight =
//             1.0 / (to_goods_path_cost.value() + to_berth_path_cost.second);
//       }

//       // 分别找到高于平均值和低于平均值的货物
//       if (goods.second.money >= (io_layer.total_goods_avg_money() / 3)) {
//         // if (true) {
//         if (cur_weight > max_weight_hi) {
//           max_weight_hi = cur_weight;
//           goods_final_hi = goods.second;
//           founded = true;
//         }
//       } else {
//         if (cur_weight > max_weight_lo) {
//           max_weight_lo = cur_weight;
//           goods_final_lo = goods.second;
//           founded = true;
//         }
//       }
//     }

//     const auto &goods_final =
//         max_weight_hi != -1 ? goods_final_hi : goods_final_lo;

//     auto path_tmp = io_layer.get_path_from_berth_to_point(
//         berth_id, goods_final.pos, success);

//     log_debug("founded: %d, path_size:%d", founded, path_tmp.size());

//     return std::make_pair(goods_final, path_tmp);
//   }

//   void ship_cycle(const int ship_id) {
//     auto &cur_ship = io_layer.ships[ship_id];
//     auto &cur_berth = io_layer.berths[cur_ship.berth_id];

//     auto go_to_virtual_point = [&]() {
//       io_layer.go(ship_id);
//       const int transport_cycle = cur_berth.transport_time;
//       berths_visit[cur_ship.berth_id] = false; // 释放泊位
//       cur_ship.new_inst(transport_cycle);
//       cur_ship.has_change_berth = false;
//       cur_ship.berth_wait_cycle = 0;
//       cur_ship.goods_wait_cycle = 0;
//       cur_ship.spend_cycle = 0;
//       log_trace("ship[%d] go to virtual point, transport time %d", ship_id,
//                 transport_cycle);
//     };

//     auto select_fit_berth = [&]() {
//       int target_berth_id_hi = -1;
//       int target_berth_id_lo = -1;
//       int fit_dis_hi = 999999;
//       int fit_dis_lo = -200;

//       const int remine_capacity = cur_ship.capacity - cur_ship.cur_capacity;
//       for (int i = 0; i < BERTH_NUM; i++) {

//         log_debug("leesum berth[%d] goods_num:%d", i,
//                   io_layer.berths[i].goods_num());
//         // 如果泊位已经被占用,或者被 ban 了,则跳过
//         if (berths_visit[i] == true) {
//           continue;
//         }
//         int cur_dis = io_layer.berths[i].goods_num() - remine_capacity;

//         if (cur_dis < 0) {
//           target_berth_id_lo = cur_dis > fit_dis_lo ? i : target_berth_id_lo;
//           fit_dis_lo = std::max(fit_dis_lo, cur_dis);
//         } else {
//           target_berth_id_hi = cur_dis < fit_dis_hi ? i : target_berth_id_hi;
//           fit_dis_hi = std::min(fit_dis_hi, cur_dis);
//         }
//         log_debug("leesum target_berth_id_hi:%d, target_berth_id_lo:%d, cur "
//                   "dis %d                ",
//                   target_berth_id_hi, target_berth_id_lo, cur_dis);
//       }
//       int target_berth_id = -1;

//       if (std::abs(fit_dis_lo) <= 5) {
//         target_berth_id = target_berth_id_lo;
//       } else {
//         target_berth_id =
//             target_berth_id_hi == -1 ? target_berth_id_lo : target_berth_id_hi;
//       }

//       log_assert(target_berth_id != -1, "error, target_berth_id is -1");
//       return target_berth_id;
//     };

//     auto move_to_next_berth = [&](const int next_berth_id) {
//       cur_ship.has_change_berth = true;
//       cur_ship.goods_wait_cycle = 0;

//       berths_visit[next_berth_id] = true;
//       berths_visit[cur_ship.berth_id] = false;
//       io_layer.ship(ship_id, next_berth_id);
//       // 泊位之间的移动时间为 500
//       cur_ship.new_inst(500);
//       cur_ship.spend_cycle += 500;
//     };

//     auto from_vp_move_to_berth = [&](const int ship_id) {
//       auto &cur_ship = io_layer.ships[ship_id];
//       if (cur_ship.is_dead) {
//         return;
//       }

//       // 1. 开始卸货
//       if (io_layer.cur_cycle != 1) {
//         // 船只的出生点在虚拟点,第一周期不需要卸货
//         io_layer.selled_goods_num += cur_ship.cur_capacity;
//         io_layer.selled_goods_money += cur_ship.cur_value;
//         cur_ship.unload();
//       }

//       // 2. 选择一个价值最高的泊位,并且标记为已经占用,并且出发去该泊位
//       int target_berth_id = select_best_berth();
//       const int remine_time = 15000 - (io_layer.cur_cycle);

//       if (target_berth_id == -1) {
//         cur_ship.is_dead = true;
//         log_trace("ship[%d] don't have enough time to move to berth[%d], "
//                   "remin_time:%d, transport_cycle:%d",
//                   ship_id, target_berth_id, remine_time);
//         return;
//       }

//       const int transport_cycle =
//           io_layer.berths[target_berth_id].transport_time;

//       if ((remine_time -
//                (io_layer.minimal_transport_time() * 2 + transport_cycle * 2) <
//            10)) {
//         log_trace("ship[%d] last transport to berth[%d], remin_time:%d, "
//                   "transport_cycle:%d, at least %d cycle to start next ",
//                   ship_id, target_berth_id, remine_time,
//                   io_layer.minimal_transport_time(), transport_cycle);
//         cur_ship.is_last_transport = true;
//       }

//       berths_visit[target_berth_id] = true;
//       // 3. 出发去该泊位
//       io_layer.ship(ship_id, target_berth_id);
//       cur_ship.new_inst(transport_cycle);
//       cur_ship.spend_cycle = transport_cycle;

//       log_trace(
//           "ship[%d] will go to berth[%d] from virtual point , trans_time ",
//           ship_id, target_berth_id, transport_cycle);
//       return;
//     };

//     if (cur_ship.status == 0) {
//       // 船只移动中
//       log_trace("ship[%d] is moving to %d, remine time %d", ship_id,
//                 cur_ship.berth_id, cur_ship.inst_remine_cycle);

//       cur_ship.inst_remine_cycle--;
//       return;
//     }

//     if (cur_ship.status == 2) {
//       // 在泊位外等待
//       cur_ship.berth_wait_cycle++;
//       log_trace("ship[%d] is waiting berth[%d], wait cycle %d", ship_id,
//                 cur_ship.berth_id, cur_ship.berth_wait_cycle);
//       log_assert(cur_ship.berth_wait_cycle < 100, "error, ship wait too long");
//       return;
//     }
//     // 重置等待周期
//     cur_ship.berth_wait_cycle = 0;

//     // 船只已经到达虚拟点或者泊位
//     log_assert(cur_ship.status == 1, "error, ship status error, status:%d",
//                cur_ship.status);

//     cur_ship.inst_remine_cycle = 0;

//     if (cur_ship.berth_id == -1) {

//       from_vp_move_to_berth(ship_id);

//       return;
//     }

//     log_info("ship[%d] is in berth[%d]", ship_id, cur_ship.berth_id);
//     // 船只已经到达泊位
//     log_assert(cur_ship.berth_id != -1, "error, ship berch_id is -1");

//     if (cur_ship.full()) {
//       // 船只已经满载,应该去虚拟点卸货
//       go_to_virtual_point();
//       log_info("ship[%d] is full at berth[%d], cur capacity:%d, cur money:%d "
//                "will go to virtual point trans time:%d",
//                ship_id, cur_ship.berth_id, cur_ship.cur_capacity,
//                cur_ship.cur_value, cur_berth.transport_time);
//       return;
//     }
//     if (!cur_ship.full()) {
//       // 船只没有满
//       // 1. 剩余时间不足以虚拟点卸货,停留在当前泊位(无论当前泊位是否有货物)
//       // 2. 当前泊位有货物,继续装货
//       // 3.
//       // 当前泊位没有货物,并且剩余时间无法去下一个泊位装货,并去虚拟点卸货,停留在当前泊位
//       // 4.
//       // 当前泊位没有货物,并且剩余时间可以去下一个泊位装货,并且去虚拟点卸货,移动到下一个泊位
//       const int cur_transport_time = cur_berth.transport_time;
//       const int remine_time = 15000 - (io_layer.cur_cycle);

//       if ((remine_time - cur_transport_time) < 1) {
//         // 必须出发去虚拟点卸货了
//         log_trace(
//             "ship[%d] don't have enough time to move to next berth[%d], go "
//             "to virtual point,remin_time:%d, cur_transport_time:%d,cur "
//             "capacity:%d, cur money:%d",
//             ship_id, cur_ship.berth_id, remine_time, cur_transport_time,
//             cur_ship.cur_capacity, cur_ship.cur_value);

//         go_to_virtual_point();
//         return;
//       }

//       if (cur_berth.is_empty()) {

//         cur_ship.goods_wait_cycle++;

//         // 当前泊位已经没有货物
//         // 1. 选择一个最适合的泊位，货物最接近满载
//         int new_select_berth_id = select_fit_berth();
//         // 2. 计算去下一个泊位的装货然后去虚拟点卸货的时间
//         const int next_transport_time =
//             io_layer.berths[new_select_berth_id].transport_time + 500;
//         // 3. 如果剩余时间不足以去下一个泊位装货,停留在当前泊位
//         if (remine_time > (next_transport_time + 5)) {
//           // if (!cur_ship.has_change_berth) {

//           auto &next_berth = io_layer.berths[new_select_berth_id];
//           const int remine_capacity = cur_ship.capacity - cur_ship.cur_capacity;
//           const int next_berth_money =
//               next_berth.get_goods_value_sum_n(remine_capacity);

//           const float cur_berth_weight =
//               static_cast<float>(cur_ship.cur_value) /
//               (cur_transport_time + cur_ship.spend_cycle + 1);
//           const float next_berth_weight =
//               static_cast<float>(next_berth_money + cur_ship.cur_value) /
//               (cur_ship.spend_cycle + next_transport_time + 500);

//           // if (next_berth_weight > cur_berth_weight ||
//           //     io_layer.cur_cycle > 8000) {
//           cur_ship.has_change_berth = true;
//           cur_ship.goods_wait_cycle = 0;
//           log_trace("ship[%d] wait too long in berth[%d], go to "
//                     "berth[%d],remin_time:%d, next_transport_time:%d",
//                     ship_id, cur_ship.berth_id, new_select_berth_id,
//                     remine_time, next_transport_time);

//           move_to_next_berth(new_select_berth_id);

//           if (cur_ship.is_last_transport) {
//             log_trace("ship[%d] is last transport, will ban berth[%d]", ship_id,
//                       cur_ship.berth_id);
//             cur_berth.is_baned = true;
//           }
//           // } else {
//           // go_to_virtual_point();
//           // }

//         } else {
//           if (io_layer.final_time == false) {
//             io_layer.final_time = true;
//             // 剩余时间不足以去下一个泊位装货,停留在当前泊位
//             log_trace(
//                 "ship[%d] wait too long in berth[%d], but no time to move "
//                 "to next berth, remine_time:%d, next_transport_time:%d",
//                 ship_id, cur_ship.berth_id, remine_time, next_transport_time);
//           }
//           return;
//         }

//       } else {
//         // 当前泊位还有货物,继续装货
//         cur_ship.goods_wait_cycle = 0;

//         int load_size =
//             std::min(cur_berth.loading_speed, cur_berth.goods_num());
//         log_assert(load_size > 0, "error, load_size is 0");

//         while (load_size > 0 && !cur_ship.full() && !cur_berth.is_empty()) {
//           auto goods = cur_berth.get_goods();
//           cur_ship.load(goods.money);
//           log_trace("ship[%d] load goods  money:%d", ship_id, goods.money);
//           load_size--;
//         }
//         // 同一帧是否可以出发去卸货, 不可以
//       }
//       return;
//     }
//   }

//   void goods_list_cycle() {
//     // 将新货物添加到货物列表中
//     for (int i = 0; i < io_layer.new_goods_num; i++) {
//       io_layer.map_goods_list[io_layer.new_goods_list[i].pos] =
//           io_layer.new_goods_list[i];
//       log_assert(io_layer.new_goods_list[i].pos != invalid_point,
//                  "invalid goods");
//     }

//     const bool update_goods_info = io_layer.cur_cycle % 10 == 0;
//     // const bool update_goods_info = false;

//     if (update_goods_info) {
//       for (int i = 0; i < BERTH_NUM; i++) {
//         io_layer.berths[i].clear_goods_info();
//         io_layer.berths[i].tmp_baned = false;
//       }
//     }

//     // 删除 goods_list 中已经消失的货物
//     for (auto it = io_layer.map_goods_list.begin();
//          it != io_layer.map_goods_list.end();) {
//       if (it->second.end_cycle < io_layer.cur_cycle &&
//           it->second.status != GoodsStatus::Got) {
//         it = io_layer.map_goods_list.erase(it);
//       } else {

//         if (update_goods_info) {
//           for (int i = 0; i < 10; i++) {
//             auto &cur_berth = io_layer.berths[i];
//             auto cur_cost = io_layer.get_cost_from_berth_to_point(i, it->first);

//             if (!cur_cost.has_value()) {
//               continue;
//             }
//             auto minimum_cost = io_layer.get_minimum_berth_cost_2(it->first);

//             if (minimum_cost.first != i) {
//               continue;
//             }

//             if (cur_cost.value() <= 120 &&
//                 it->second.money >= io_layer.total_goods_avg_money()) {
//               cur_berth.near_goods_num++;
//               cur_berth.near_goods_value += it->second.money;
//               cur_berth.near_goods_distance += cur_cost.value();
//             }
//           }
//         }

//         ++it;
//       }
//     }
//     log_info("map_goods_list size:%d", io_layer.map_goods_list.size());

//     // if (update_goods_info) {

//     //   std::vector<std::pair<int, int>> berths_sort;
//     //   for (int i = 0; i < BERTH_NUM; i++) {
//     //     berths_sort.emplace_back(i, io_layer.berths[i].goods_num());
//     //   }
//     //   std::sort(berths_sort.begin(), berths_sort.end(),
//     //             [&](const auto &p1, const auto &p2) {
//     //               return p1.second < p2.second;
//     //             });

//     //   for (int i = 0; i < 10; i++) {
//     //     io_layer.berths[i].tmp_baned = false;
//     //   }

//     //   std::for_each_n(berths_sort.begin(), 2, [&](const auto &p) {
//     //     log_info("berth[%d] goods_num:%d", p.first, p.second);
//     //     io_layer.berths[p.first].tmp_baned = true;
//     //   });
//     // }

//     // for (int i = 0; i < BERTH_NUM; i++) {
//     //   io_layer.berths[i].printf_near_goods_info();

//     //   // // 动态禁用港口可能还是副作用
//     //   if (update_goods_info) {
//     //     if (io_layer.cur_cycle > 1000) {
//     //       if (io_layer.berths[i].near_goods_num < 6) {

//     //         if (!io_layer.berths[i].tmp_baned) {
//     //           bool maybe_baned = true;
//     //           io_layer.berths[i].tmp_baned = maybe_baned;
//     //         }
//     //       } else {
//     //         io_layer.berths[i].tmp_baned = false;
//     //       }
//     //     }
//     //   }

//     //   log_info("berth[%d] is_baned:%d", i, io_layer.berth_is_baned(i));
//     // }
//   }

//   void go_near_berth(const int robot_id) {
//     if (!robots_has_goods[robot_id] && !robots_first_get_goods[robot_id]) {
//       log_trace("robot[%d] has no goods, no need go_to_berth", robot_id);
//       // 机器人没有货物,不需要去靠泊点
//       return;
//     }

//     auto &robot_path = robots_path_list[robot_id];
//     const Point robot_pos = robots_cur_pos_list[robot_id];

//     if (!robot_path.empty()) {
//       robots_next_pos_list.at(robot_id) = robot_path.back();
//     }

//     if (robot_path.empty()) {
//       bool founded = false;
//       int berth_id;

//       std::vector<int> exclude_berths{};
//       // for (int i = 0; i < BERTH_NUM; i++) {
//       //   if (get_berth_robot_num(i, robot_id) > 3 && !io_layer.final_time) {
//       //     exclude_berths.emplace_back(i);
//       //   }
//       // }

//       auto path = io_layer.get_near_berth_path_exclude(robot_pos, berth_id,
//                                                        founded, exclude_berths);

//       if (founded) {
//         robot_path = path;
//         // berths_id_list[robot_id] = berth_id;
//         io_layer.robots[robot_id].target_berth_id = berth_id;
//         robots_next_pos_list.at(robot_id) = robot_path.back();
//       } else {
//         // 一定可以找到路径, 如果找不到路径,则机器人被困住了
//         log_trace("robot[%d] (%d,%d) not found path to berth, is dead!",
//                   robot_id, P_ARG(robot_pos));
//         // robots_is_dead[robot_id] = true;
//       }
//     }
//   };

//   void find_new_goods(int robot_id) {

//     // 机器人位置位置信息
//     auto &cur_robot_target_goods = robots_target_goods_list[robot_id];
//     auto &cur_robot = io_layer.robots[robot_id];
//     const Point &robot_pos = robots_cur_pos_list[robot_id];
//     const Point &robot_next_pos = robots_next_pos_list[robot_id];

//     if (robots_target_goods_list[robot_id].status == GoodsStatus::Got) {
//       // 机器人已经拿到货物,应该去卸货
//       return;
//     }

//     if (io_layer.cur_cycle == 1) {
//       bool cycle1_founded = false;
//       int cycle1_near_berth_id;
//       auto near_path = io_layer.get_near_berth_path_exclude(
//           robot_pos, cycle1_near_berth_id, cycle1_founded, cycle1_berths_visit);

//       if (cycle1_founded) {
//         robots_path_list[robot_id] = Tools::last_n(near_path, 15);
//         robots_next_pos_list[robot_id] = near_path.back();
//         io_layer.robots[robot_id].target_berth_id = cycle1_near_berth_id;
//         cycle1_berths_visit.emplace_back(cycle1_near_berth_id);
//         robots_idle_cycle[robot_id] = near_path.size();

//       } else {
//         // 一定可以找到路径, 如果找不到路径,则机器人被困住了
//         log_trace("robot[%d] (%d,%d) not found path to berth, is dead!",
//                   robot_id, P_ARG(robot_pos));
//         // robots_is_dead[robot_id] = true;
//       }
//     }

//     if (robots_has_goods[robot_id]) {
//       // 机器人已经拿到货物,应该去卸货
//       log_trace("robot[%d] got goods, no need find_new_goods", robot_id);
//       log_trace("cur_robot_target_goods (%d,%d) status:%d, cur_cycle:%d, "
//                 "end_cycle:%d",
//                 P_ARG(cur_robot_target_goods.pos),
//                 cur_robot_target_goods.status, io_layer.cur_cycle,
//                 cur_robot_target_goods.end_cycle);

//       // log_assert(cur_robot_target_goods.status == GoodsStatus::Got,
//       //            "error, robot should had goods (%d,%d),but status is
//       //            %d,", P_ARG(robot_pos),
//       //            cur_robot_target_goods.status);
//       return;
//     }
//     if (cur_robot_target_goods.status == GoodsStatus::Booked &&
//         !cur_robot_target_goods.is_disappeared(io_layer.cur_cycle)) {
//       // 货物没有消失
//       log_assert(!robots_path_list[robot_id].empty(),
//                  "error, robot[%d] should had path to goods (%d,%d),but path "
//                  "is empty",
//                  robot_id, P_ARG(cur_robot_target_goods.pos));
//       robots_next_pos_list[robot_id] = robots_path_list[robot_id].back();
//       return;
//     }

//     if (cur_robot_target_goods.status == GoodsStatus::Booked &&
//         !cur_robot_target_goods.is_disappeared(io_layer.cur_cycle)) {
//       // 货物没有消失
//       log_assert(!robots_path_list[robot_id].empty(),
//                  "error, robot should had path to goods (%d,%d),but path "
//                  "is empty",
//                  P_ARG(cur_robot_target_goods.pos));
//       robots_next_pos_list[robot_id] = robots_path_list[robot_id].back();
//       // 机器人已经有预定的货物,并且货物没有消失，不需要再次寻找
//       return;
//     }

//     if (cur_robot_target_goods.status == GoodsStatus::Dead) {
//       if (robots_idle_cycle[robot_id] > 0) {
//         log_trace("robot[%d] idle_cycle:%d, no need find_new_goods", robot_id,
//                   robots_idle_cycle[robot_id]);
//         robots_idle_cycle[robot_id]--;
//         if (!robots_path_list[robot_id].empty()) {
//           robots_next_pos_list[robot_id] = robots_path_list[robot_id].back();
//           // 还有随机路径时,不需要再次寻找
//           return;
//         }
//       }
//     }

//     // 打表法
//     // 已经运送货物到港口区域,在港口区域寻找下一个货物
//     auto can_search = io_layer.in_berth_search_area(robot_pos);
//     if (can_search.has_value()) {

//       // 将机器人的目标港口设置为当前选择的港口
//       cur_robot.target_berth_id = can_search.value();

//       log_debug("robot[%d] in_berth_search_area", robot_id);
//       const int search_berth_id = can_search.value();
//       log_assert(search_berth_id >= 0 && search_berth_id < BERTH_NUM,
//                  "error, search_berth_id is invalid");
//       bool search_founded = false;

//       auto search_result = find_best_goods_path_from_berth_v1(
//           search_berth_id, robot_id, search_founded);

//       if (search_founded) {
//         const auto searched_goods = search_result.first;
//         auto searched_path = search_result.second;
//         log_assert(!searched_path.empty(), "error, searched_path is empty");

//         bool cut_success = false;
//         auto search_come_from = PATHHelper::cut_path(
//             robot_pos, get_is_barrier_lambda(), get_find_neighbor_lambda(),
//             searched_path, 20, cut_success);

//         if (cut_success) {
//           log_assert(!searched_path.empty(),
//                      "error, search_come_from is "
//                      "empty, robot_pos(%d,%d), "
//                      "searched_path size:%d",
//                      P_ARG(robot_pos), searched_path.size());

//           // 更新机器人的路径,下一步位置,货物信息
//           robots_path_list[robot_id] = searched_path;
//           robots_next_pos_list[robot_id] = searched_path.back();
//           robots_target_goods_list[robot_id] = searched_goods;
//           robots_target_goods_list[robot_id].status = GoodsStatus::Booked;
//           io_layer.map_goods_list.at(searched_goods.pos).status =
//               GoodsStatus::Booked;

//           log_trace("robot[%d] in_berth_search_area success, find "
//                     "goods(%d,%d), "
//                     "path size %d ",
//                     robot_id, P_ARG(searched_goods.pos), searched_path.size());

//           return;
//         }
//       }
//     }

//     // 不在港口区域或者找不到货物,选择去港口区域
//     search_count++;

//     log_trace("robot[%d] find_new_goods start from any postion", robot_id);

//     const auto goods_set = Tools::map_to_set(io_layer.map_goods_list);

//     auto goods_goal_func = [&](Point p) {
//       if (goods_set.find(p) != goods_set.end()) {
//         auto goods_tmp = io_layer.map_goods_list.at(p);
//         if (goods_tmp.status != GoodsStatus::Normal) {
//           return false;
//         }
//         if (goods_tmp.money < io_layer.total_goods_avg_money()) {
//           return false;
//         }
//         if (goods_tmp.is_disappeared(io_layer.cur_cycle + 10)) {
//           return false;
//         }

//         return true;
//       }
//       return false;
//     };
//     bool goods_founded = false;

//     auto goods_test_path = PATHHelper::bfs_path_v1(
//         cur_robot.pos, goods_goal_func, get_is_barrier_lambda(),
//         get_find_neighbor_lambda(), 30, goods_founded);

//     if (goods_founded) {
//       const auto &searched_goods =
//           io_layer.map_goods_list.at(goods_test_path.front()); // 获取货物信息

//       // 更新机器人的路径,下一步位置,货物信息
//       robots_path_list[robot_id] = goods_test_path;
//       robots_next_pos_list[robot_id] = goods_test_path.back();
//       robots_target_goods_list[robot_id] = searched_goods;
//       robots_target_goods_list[robot_id].status = GoodsStatus::Booked;
//       io_layer.map_goods_list.at(searched_goods.pos).status =
//           GoodsStatus::Booked;

//       return;
//     }

//     bool near_founded = false;
//     int near_berth_id;

//     std::vector<int> exclude_berths{};
//     for (int i = 0; i < BERTH_NUM; i++) {
//       if (get_berth_robot_num(i, robot_id) > 1 && !io_layer.final_time) {
//         exclude_berths.emplace_back(i);
//       }
//     }

//     std::vector<Point> near_path = io_layer.get_near_berth_path_exclude(
//         robot_pos, near_berth_id, near_founded, exclude_berths);

//     if (near_founded) {
//       cur_robot.target_berth_id = near_berth_id; // 设置机器人的目标港口
//       robots_path_list[robot_id] = Tools::last_n(near_path, 10);
//       robots_idle_cycle[robot_id] =
//           robots_path_list[robot_id]
//               .size(); // 更新机器人的路径时间,时间内不再寻路

//       robots_next_pos_list[robot_id] =
//           near_path.back(); // 更新机器人的下一步位置
//     } else {
//       // 一定可以找到路径, 如果找不到路径,则机器人被困住了
//       log_trace("robot[%d] (%d,%d) not found path to berth, is dead!", robot_id,
//                 P_ARG(robot_pos));
//       // robots_is_dead[robot_id] = true;
//     }
//   };

//   void run_game() {

//     robots_is_dead.fill(false);
//     robots_get_action.fill(false);
//     robots_has_pass_collision.fill(false);

//     for (int zhen = 1; zhen <= 15000; zhen++) {
//       io_layer.input_cycle();

//       // 更新货物信息
//       goods_list_cycle();

//       search_count = 0;
//       // 获取机器人当前位置
//       for (int i = 0; i < 10; i++) {
//         robots_cur_pos_list[i] = io_layer.robots[i].pos;
//         robots_next_pos_list[i] = invalid_point;
//         robots_has_goods[i] = io_layer.robots[i].had_goods;
//       }

//       for (int i = 0; i < 10; i++) {
//         log_trace("robot[%d] is dead:%d", i, robots_is_dead[i]);
//       }

//       // berth_cycle();

//       for (int i = 0; i < SHIP_NUM; i++) {
//         ship_cycle(i);
//       }

//       robots_first_get_goods.fill(false);
//       robots_has_pass_collision.fill(false);
//       robots_need_move_but_not_move.clear();

//       for (int i = 0; i < 10; i++) {
//         if (robots_is_dead[i]) {
//           continue;
//         }
//         robots_get_cycle(i);
//         find_new_goods(i);
//         go_near_berth(i);
//       }

//       std::copy(robots_next_pos_list.begin(), robots_next_pos_list.end(),
//                 robots_next_pos_list_copy.begin());

//       for (int i = 0; i < 10; i++) {
//         if (robots_is_dead[i]) {
//           continue;
//         }
//         check_collision_v1(i);
//       }

//       for (int i = 0; i < 10; i++) {
//         if (robots_is_dead[i]) {
//           continue;
//         }
//         check_collison_step2(i);
//       }

//       for (int i = 0; i < 10; i++) {
//         robots_move(i);
//         robots_pull_cycle(i);
//       }
//       io_layer.output_cycle();
//       log_info("map_goods_list size:%d", io_layer.map_goods_list.size());
//       io_layer.print_goods_info();

//       if (abs(io_layer.cur_cycle) == 15000) {
//         break;
//       }
//     }
//     io_layer.print_final_info();
//   }

//   void robots_move(const int rebot_id) {
//     const auto &next_pos = robots_next_pos_list[rebot_id];
//     if (next_pos != invalid_point && next_pos != stop_point) {
//       io_layer.robot_move(rebot_id, next_pos);
//       robots_path_list[rebot_id].pop_back();
//     }
//     if (next_pos == stop_point) {
//       robots_path_list[rebot_id].pop_back();
//     }
//   }

//   // 机器人卸货
//   void robots_pull_cycle(const int robot_id) {
//     const auto &next_pos = robots_next_pos_list[robot_id];
//     auto &cur_berth =
//         io_layer.berths[io_layer.robots[robot_id].target_berth_id];
//     const auto &target_goods = robots_target_goods_list[robot_id];
//     const auto &had_goods = robots_has_goods[robot_id];

//     if (target_goods.status == GoodsStatus::Dead) {
//       log_assert(!had_goods, "error, robot should not had goods");
//       // 没有货物
//       return;
//     }
//     if (!had_goods) {
//       return;
//     }

//     log_assert(target_goods.status == GoodsStatus::Got,
//                "error, robot should had goods ,but status is %d,",
//                target_goods.status);

//     auto berth_id_opt = io_layer.in_berth_area(next_pos);
//     if (berth_id_opt.has_value() && had_goods) {

//       log_trace("robot[%d] pull goods at (%d,%d)", robot_id,
//                 P_ARG(target_goods.pos));
//       log_trace("robot[%d] goods money:%d,cur_cycle:%d end_cycle:%d, status:%d",
//                 robot_id, target_goods.money, io_layer.cur_cycle,
//                 target_goods.end_cycle, target_goods.status);

//       // 卸货要求
//       // 1. 机器人已经到达靠泊点
//       // 2. 机器人已经拿到货物
//       // 3. 机器人有预定的货物
//       io_layer.robot_pull(robot_id);
//       io_layer.goted_goods_num++;
//       io_layer.goted_goods_money += target_goods.money;

//       // 清空状态位
//       // 1. 机器人的目标货物
//       // 2. 地图上的货物
//       // 3. 机器人的路径
//       // map_goods_list.at(target_goods.pos) = invalid_goods;

//       io_layer.berths[berth_id_opt.value()].add_goods(target_goods,
//                                                       io_layer.cur_cycle);
//       robots_target_goods_list[robot_id] = invalid_goods;
//       robots_path_list[robot_id].clear();
//     }
//   };

//   // 机器人获取货物
//   void robots_get_cycle(const int robot_id) {
//     const auto &cur_pos = robots_cur_pos_list[robot_id];
//     auto &cur_berth =
//         io_layer.berths[io_layer.robots[robot_id].target_berth_id];
//     auto &target_goods = robots_target_goods_list[robot_id];
//     const auto &had_goods = robots_has_goods[robot_id];

//     if (target_goods.status == GoodsStatus::Dead) {
//       // 没有货物
//       return;
//     }

//     if (had_goods) {
//       log_assert(target_goods.status == GoodsStatus::Got,
//                  "error, robot should had goods (%d,%d),but status is %d,",
//                  P_ARG(cur_pos), target_goods.status);
//     }

//     if (target_goods.status == GoodsStatus::Booked &&
//         target_goods.is_disappeared(io_layer.cur_cycle)) {
//       // 正在去货物的路上,货物消失了
//       log_trace("robot[%d] target_goods is disappeared", robot_id);
//       robots_target_goods_list[robot_id].status = GoodsStatus::Dead;
//       robots_path_list[robot_id].clear();
//       return;
//     }
//     log_assert(target_goods.status != GoodsStatus::Normal,
//                "goods states error");

//     if (target_goods.status == GoodsStatus::Booked && !had_goods) {
//       if (cur_pos == target_goods.pos) {
//         // 装货要求
//         // 1. 机器人已经到达货物位置
//         // 2. 机器人没有拿到货物
//         // 3. 机器人有预定的货物
//         io_layer.robot_get(robot_id);
//         log_trace("robot[%d] get goods at (%d,%d)", robot_id,
//                   P_ARG(target_goods.pos));

//         // 更改货物状态
//         robots_target_goods_list[robot_id].status = GoodsStatus::Got;
//         // map_goods_list.at(target_goods.pos).status = GoodsStatus::Got;
//         robots_first_get_goods[robot_id] = true;
//       }
//     }
//   };
// };
