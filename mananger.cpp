
// #include "mananger.h"
// #include "berth.h"
// #include "io_layer.h"
// #include "log.h"
// #include "point.h"
// #include <algorithm>
// #include <array>
// #include <cassert>
// #include <chrono>
// #include <deque>
// #include <stdexcept>
// #include <sys/select.h>
// #include <sys/time.h> //引入头文件
// #include <vector>
// Manager::Manager() { io_layer = IoLayer(); }

// void Manager::init_game() {
//   io_layer.init();
//   for (int i = 0; i < ROBOT_NUM; i++) {
//     auto robot = RobotC(i, Point(0, 0));
//     robots.push_back(robot);
//   }
//   for (int i = 0; i < SHIP_NUM; i++) {
//     auto ship = Ship(i, 0, 0, 0);
//     ships.push_back(ship);
//   }

//   for (int i = 0; i < BERTH_NUM; i++) {
//     auto berth = io_layer.berths[i];
//     berth_list.push_back(berth);
//   }
// }

// Manager::~Manager() {}

// void Manager::print_goods_list() {
//   for (auto &goods : goods_list) {
//     // log_trace("goods pos: %d, %d, money: %d", goods.second.pos.x,
//     //           goods.second.pos.y, goods.second.money,
//     //           goods.second.be_selected, goods.second.end_cycle);
//     log_trace("goods pos: (%d,%d), money: %d, end time:%d", goods.second.pos.x,
//               goods.second.pos.y, goods.second.money, goods.second.end_cycle);
//   }
// }

// void Manager::go_to_goods(RobotC &robot) {
//   if (!robot.path_valid || !robot.target_goods.has_value()) {
//     log_fatal("robot path not valid or target_goods empty");
//     assert(false);
//   }

//   if (robot.path.empty() && robot.pos != robot.target_goods->pos) {
//     log_fatal("when path empty, robot pos not equal to target_goods pos");
//     assert(robot.pos == robot.target_goods->pos);
//   }

//   auto get_goods_func = [&]() {
//     io_layer.robot_get(robot.id);

//     robot.target_goods = std::nullopt;
//     robot.path_valid = false;
//     goods_list[robot.target_goods->pos].end_cycle = io_layer.cur_cycle - 10;
//     io_layer.goted_goods_num++;
//     io_layer.goted_goods_money += robot.target_goods->money;
//   };

//   if (robot.path.empty() && robot.pos == robot.target_goods->pos) {
//     // 机器人和货物位置重合
//     log_trace("no need move, robot[%d] get goods (%d,%d)", robot.id,
//               robot.target_goods->pos.x, robot.target_goods->pos.y);
//     robot.next_pos = std::nullopt;
//     robot.next_type = RobotC::RobotNextPointType::EMPTY;
//     get_goods_func();
//   }

//   if (!robot.collision_path.empty()) {
//     auto recover_point = robot.collision_path.top();
//     robot.next_pos = robot.collision_path.top();
//     robot.next_type = RobotC::RobotNextPointType::RECOVER;
//     log_trace("recover robot[%d] next pos to (%d,%d)", robot.id,
//               recover_point.x, recover_point.y);
//   } else if (!robot.path.empty()) {
//     auto next_point = robot.path.back();
//     robot.next_pos = next_point;
//     robot.next_type = RobotC::RobotNextPointType::GOODS;
//     log_trace("robot[%d] move form (%d,%d) to (%d,%d)", robot.id, robot.pos.x,
//               robot.pos.y, next_point.x, next_point.y);
//   } else {
//     log_trace("robot stop move");
//   }
// }

// void Manager::find_new_berth(RobotC &robot) {
//   if (robot.fsm_status != RobotFSM::SELECT_BERTH) {
//     log_fatal("robot [%d] fsm status not select_berth", robot.id);
//     assert(false);
//   }
//   if (robot.target_berth.has_value()) {
//     log_fatal("robot [%d] target_berth not empty", robot.id);
//     assert(false);
//   }
//   if (!robot.target_goods.has_value() || robot.had_goods) {
//     log_fatal("robot [%d] target_goods empty, target_goods %d, had_goods %d",
//               robot.id, robot.target_goods.has_value(), robot.had_goods);
//     assert(false);
//   }
//   if (robot.path_valid || !robot.path.empty() ||
//       !robot.collision_path.empty()) {
//     log_fatal("robot [%d] path valid or not empty", robot.id);
//     log_fatal("robot [%d] path_valid %d, path size %d, collision_path size %d",
//               robot.id, robot.path_valid, robot.path.size(),
//               robot.collision_path.size());
//     assert(false);
//   }
// }

// void Manager::find_new_goods(RobotC &robot) {
//   // if (robot.fsm_status != RobotFSM::SELECT_GOODS) {
//   //   log_fatal("robot fsm status not select_goods");
//   //   assert(false);
//   // }

//   if (robot.target_goods.has_value()) {
//     log_fatal("robot target_goods not empty");
//     assert(false);
//     return;
//   }
//   if (robot.status != 1) {
//     log_fatal("robot status not ok");
//     assert(false);
//     return;
//   }

//   if (goods_list.size() <= robot.id) {
//     log_info("remaining goods less than robot num, no goods to find");
//     return;
//   }

//   auto cur_map = io_layer.game_map_original;
//   Point robot_pos = robot.pos;
//   Point target_pos = Point(-1, -1);

//   int cur_zhen = io_layer.cur_cycle;
//   int search_level = 0;
//   std::deque<Point> q;
//   static std::unordered_map<Point, Point> came_from;
//   came_from.clear();

//   came_from[robot_pos] = Point(-1, -1);
//   q.push_back(robot_pos);

//   while (!q.empty() && search_level < 40) {
//     int level_size = q.size();
//     for (int i = 0; i < level_size; i++) {
//       auto current = q.front();
//       q.pop_front();

//       // 判断是否找到目标, 并且货物还没有消失, 也没有被其他机器人预定
//       if (goods_list.find(current) != goods_list.end() &&
//           goods_list[current].end_cycle > cur_zhen &&
//           !goods_list.at(current).be_selected) {

//         log_info("find goods, pos: %d, %d", current.x, current.y);

//         // 第一次选择货物
//         goods_list[current].be_selected = true;
//         robot.target_goods = goods_list[current];
//         target_pos = current;
//         // goods_list.erase(current);
//         goto end_search;
//       }

//       // 将顶点的下一层没有访问过的节点加入队列
//       for (auto &next : cur_map->neighbors(current)) {
//         if (came_from.find(next) == came_from.end()) {
//           q.push_back(next);
//           came_from[next] = current;
//         }
//       }
//     }
//     search_level++;
//   }

// end_search:

//   // find the path
//   Point current_point = target_pos;
//   std::vector<Point> cur_path;
//   bool found_path = true;
//   while (current_point != robot_pos) {
//     cur_path.push_back(current_point);
//     try {
//       current_point = came_from.at(current_point);
//     } catch (const std::out_of_range &e) {
//       log_warn("robot[%d] no goods found from (%d, %d),search level %d ",
//                robot.id, robot_pos.x, robot_pos.y, search_level);
//       found_path = false;
//       return;
//     }
//   }
//   if (found_path) {
//     log_trace("robot[%d] find new goods at (%d,%d), distance: %d", robot.id,
//               target_pos.x, target_pos.y, cur_path.size());
//     goods_list[target_pos].be_selected = true;
//     robot.target_goods = goods_list[target_pos];
//     robot.path = cur_path;
//     robot.path_valid = true;
//   }
// }

// void Manager::run_game() {

//   io_layer.input_cycle();

//   struct timeval t1, t2;
//   double timeuse;
//   gettimeofday(&t1, NULL);

//   log_info("cycle[%d], goods size:%d", io_layer.cur_cycle, goods_list.size());

//   // 新增货物
//   for (int i = 0; i < io_layer.new_goods_num; i++) {
//     // goods_list.push_back(GoodsC(io_layer.new_goods_list[i]));
//     goods_list[io_layer.new_goods_list[i].pos] = io_layer.new_goods_list[i];
//   }
//   log_trace("new goods num:%d", io_layer.new_goods_num);

//   // print_goods_list();
//   // 更新机器人状态
//   for (int i = 0; i < ROBOT_NUM; i++) {
//     robots[i].status = io_layer.robots[i].status;
//     robots[i].had_goods = io_layer.robots[i].goods == 1;
//     robots[i].pos.x = io_layer.robots[i].x;
//     robots[i].pos.y = io_layer.robots[i].y;
//   }

//   // 更新船只状态
//   for (int i = 0; i < SHIP_NUM; i++) {
//     ships[i].status = io_layer.ships[i].status;
//     ships[i].berch_id = io_layer.ships[i].berch_id;
//   }

//   // print_goods_list();
//   static int last_select = 0;

//   auto last_select_inc = []() {
//     last_select++;
//     if (last_select >= ROBOT_NUM) {
//       last_select = 0;
//     }
//   };

//   // 寻路
//   for (int i = 0; i < ROBOT_NUM; i++) {
//     // 轮询
//     if (robots[last_select].status == 1 &&
//         !robots[last_select].target_goods.has_value()) {
//       gettimeofday(&t2, NULL);
//       timeuse =
//           t2.tv_sec - t1.tv_sec + (double)(t2.tv_usec - t1.tv_usec) / 1000000.0;

//       find_new_goods(robots[last_select]);
//     }
//     last_select_inc();
//     if (timeuse > 0.008) {
//       log_warn("time overflow : %f", timeuse);
//       break;
//     }
//   }

//   // 机器人指令
//   for (auto &robot : robots) {
//     if (robot.status == 1) {
//       if (robot.target_goods.has_value()) {
//         go_to_goods(robot);
//       }
//     }
//   }
//   // 碰撞检测
//   collision_detect();

//   robot_move();

//   // 删除 goods_list 中已经消失的货物
//   for (auto it = goods_list.begin(); it != goods_list.end();) {
//     if (it->second.end_cycle < io_layer.cur_cycle) {
//       it = goods_list.erase(it);
//     } else {
//       it++;
//     }
//   }

//   // auto start = std::chrono::high_resolution_clock::now();

//   // std::for_each(robots.begin(), robots.end(), [&](auto &robot) {
//   //   for (int i = 0; i < BERTH_NUM; i++) {
//   //     auto berth_i_path = io_layer.get_berth_path(i, robot.pos);

//   //     if (berth_i_path.has_value()) {

//   //       int cost = io_layer.berths_come_from[i][robot.pos].cost;
//   //       log_trace("robot[%d] to berth[%d] path size:%d, cost%d", robot.id, i,
//   //                 berth_i_path.value().size(), cost);

//   //     } else {
//   //       log_trace("robot[%d] to berth[%d] no path", robot.id, i);
//   //     }
//   //   }
//   // });
//   // auto end = std::chrono::high_resolution_clock::now();
//   // log_info("berth search time:%d ms",
//   //          std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
//   //              .count());

//   io_layer.output_cycle();
// }

// void Manager::robot_move() {
//   // 机器人移动
//   std::for_each(robots.begin(), robots.end(), [&](auto &robot) {
//     if (robot.next_pos.has_value()) {
//       if (robot.next_type == RobotC::RobotNextPointType::EMPTY) {
//         log_fatal("robot next type empty when move");
//         assert(false);
//       }
//       io_layer.robot_move(robot.id,
//                           calc_direction(robot.pos, robot.next_pos.value()));
//     };
//     switch (robot.next_type) {
//     case RobotC::RobotNextPointType::GOODS: {
//       auto next_pointa = robot.path.back();
//       if (robot.next_pos.has_value() && robot.next_pos.value() != next_pointa) {
//         log_fatal("robot next pos not equal to path back");
//         assert(false);
//       } else if (!robot.next_pos.has_value()) {
//         log_fatal("robot next pos empty");
//       }

//       log_trace("robot[%d] move form (%d,%d) to (%d,%d)", robot.id, robot.pos.x,
//                 robot.pos.y, next_pointa.x, next_pointa.y);

//       robot.path.pop_back();
//       if (robot.path.empty()) {
//         log_info("robot get goods, robot id:%d", robot.id);
//         io_layer.robot_get(robot.id);
//       };
//       break;
//     }
//     case RobotC::RobotNextPointType::COLLISION: {
//       auto cur_pointa = robot.collision_path.top();
//       if (robot.pos != cur_pointa) {
//         log_fatal("robot next pos not equal to collision path top");
//         assert(false);
//       } else if (!robot.next_pos.has_value()) {
//         log_fatal("robot next pos empty");
//       }
//       log_trace("robot[%d] move form (%d,%d) to (%d,%d)", robot.id, robot.pos.x,
//                 robot.pos.y, robot.next_pos.value().x,
//                 robot.next_pos.value().y);
//       break;
//     }
//     case RobotC::RobotNextPointType::RECOVER: {
//       auto next_pointa = robot.collision_path.top();
//       if (robot.next_pos.has_value() && robot.next_pos.value() != next_pointa) {
//         log_fatal("robot next pos not equal to recover point");
//         assert(false);
//       } else if (!robot.next_pos.has_value()) {
//         log_fatal("robot next pos empty");
//       }

//       log_trace("robot[%d] move form (%d,%d) to (%d,%d)", robot.id, robot.pos.x,
//                 robot.pos.y, next_pointa.x, next_pointa.y);
//       robot.collision_path.pop();
//       break;
//     }
//     case RobotC::RobotNextPointType::EMPTY: {
//       log_trace("robot[%d] stop move", robot.id);
//       break;
//     }

//     default: {
//       log_fatal("robot next type invalid");
//       assert(false);
//       break;
//     }
//     };

//     robot.next_pos = std::nullopt;
//     robot.next_type = RobotC::RobotNextPointType::EMPTY;
//   });
// }

// RobotDrirection Manager::calc_direction(Point from, Point to) {

//   if (!(abs(from.x - to.x) <= 1 && abs(from.y - to.y) <= 1)) {
//     log_fatal("calc_direction err, from: (%d, %d), to: (%d, %d)", from.x,
//               from.y, to.x, to.y);
//     assert(false);
//   }

//   if (from.x == to.x) {
//     if (from.y > to.y) {
//       return RobotDrirection::LEFT;
//     } else {
//       return RobotDrirection::RIGHT;
//     }
//   } else {
//     if (from.x > to.x) {
//       return RobotDrirection::UP;
//     } else {
//       return RobotDrirection::DOWN;
//     }
//   }
// }

// void Manager::collision_detect() {
//   // 需要收集所有机器人的当前位置和下一步的位置
//   std::array<Point, 10> all_cur_pos;
//   std::array<Point, 10> all_next_pos;

//   const Point invalid_point = Point(-1, -1);

//   for (int i = 0; i < ROBOT_NUM; i++) {
//     all_cur_pos[i] = robots[i].pos;
//     all_next_pos[i] = robots[i].next_pos.value_or(invalid_point);
//   }

//   auto move_to_another_direction = [&](int robot_id) {
//     // 1. 找到当前位置的邻居
//     auto nerbors = io_layer.game_map_original->neighbors(all_cur_pos[robot_id]);

//     // 2. 从邻居中找到新下一步位置
//     auto new_pos = std::find_if(nerbors.begin(), nerbors.end(), [&](auto &pos) {
//       return pos != all_next_pos[robot_id];
//     });

//     // 3. 更新机器人的下一步位置
//     //   a. 如果找到新的位置, 更新机器人的下一步位置
//     //   b. 如果没有找到新的位置, 机器人的下一步位置改为 std::nullopt, 停止移动
//     if (new_pos != nerbors.end()) {
//       log_trace("robot[%d] next pos(%d,%d) collision, change to (%d,%d)",
//                 robot_id, all_next_pos[robot_id].x, all_next_pos[robot_id].y,
//                 new_pos->x, new_pos->y);
//       robots[robot_id].next_pos = *new_pos;
//       robots[robot_id].next_type = RobotC::RobotNextPointType::COLLISION;
//       robots[robot_id].collision_path.push(
//           all_cur_pos[robot_id]); // 保存当前位置, 用于恢复
//       all_next_pos[robot_id] = *new_pos;
//     } else {
//       log_trace("robot[%d] cur_pos(%d,%d) can not find a avoid path, stop go "
//                 "to (%d,%d)",
//                 robot_id, all_cur_pos[robot_id].x, all_cur_pos[robot_id].y,
//                 all_next_pos[robot_id].x, all_next_pos[robot_id].y);
//       robots[robot_id].next_pos = std::nullopt;
//       robots[robot_id].next_type = RobotC::RobotNextPointType::EMPTY;
//       all_next_pos[robot_id] = Point(-1, -1);
//     }
//   };

//   // 检测是否有碰撞

//   for (int i = 0; i < ROBOT_NUM; i++) {

//     //   1. 下一次移动的位置与优先级高的机器人的当前位置冲突:
//     //   选择一个方向移走(为优先级高的开路)
//     if (std::any_of(all_cur_pos.begin(), all_cur_pos.begin() + i,
//                     [&](auto &pos) { return pos == all_next_pos[i]; })) {

//       if (all_next_pos[i] != invalid_point) {
//         move_to_another_direction(i);
//       } else {
//         log_fatal("error, other robot pos invalid, cur robot id:%d", i);
//         assert(false);
//       }
//     };

//     //   2. 下一次移动的位置与优先级高的机器人的下一次位置冲突: 取消当前移动
//     //   (给优先级高的让路) (是否可以选择一个方向移走?)
//     if (std::any_of(all_next_pos.begin(), all_next_pos.begin() + i,
//                     [&](auto &pos) {
//                       return pos == all_next_pos[i] && pos != invalid_point;
//                     })) {
//       if (all_next_pos[i] != invalid_point) {
//         move_to_another_direction(i);
//       } else {
//         log_fatal("should not happen, cur robot id:%d", i);
//         assert(false);
//       }
//     };

//     //   1. 下一次移动位置与优先级低的机器人当前位置冲突: 停止当前移动,
//     //   原地不动(在下一个周期,其他优先级低的会让出位置)
//     if (std::any_of(all_cur_pos.begin() + i + 1, all_cur_pos.end(),
//                     [&](auto &pos) { return pos == all_next_pos[i]; })) {
//       if (all_next_pos[i] != invalid_point) {
//         log_trace("robot[%d] cur pos(%d,%d) collision, stop go to (%d,%d)", i,
//                   all_cur_pos[i].x, all_cur_pos[i].y, all_next_pos[i].x,
//                   all_next_pos[i].y);
//         robots[i].next_pos = std::nullopt;
//         robots[i].next_type = RobotC::RobotNextPointType::EMPTY;
//         all_next_pos[i] = invalid_point;
//       } else {
//         log_fatal("should not happen, cur robot id:%d", i);
//         assert(false);
//       }
//     };

//     //   2.
//     //   下一次移动位置与优先级低的下一次移动的位置冲突:继续移动(因为优先级低的会取消本次移动)
//     if (std::any_of(all_next_pos.begin() + i + 1, all_next_pos.end(),
//                     [&](auto &pos) {
//                       return pos == all_next_pos[i] && pos != invalid_point;
//                     })) {
//       // 继续移动(因为优先级低的会取消本次移动)
//       log_trace("robot[%d] next pos(%d,%d) collision, continue go to (%d,%d)",
//                 i, all_next_pos[i].x, all_next_pos[i].y, all_next_pos[i].x,
//                 all_next_pos[i].y);
//     };
//   }
// }
