#include "robot.h"
#include "a_star_path.h"
#include "goods.h"
#include "point.h"
#include <algorithm>
#include <vector>

RobotC::RobotC(int id, Point pos) {
  this->id = id;
  this->pos = pos;
  // this->orignal_map = map;
  // this->current_map = std::make_unique<GameMap>(map->width, map->height);
}

RobotC::~RobotC() {}

// void RobotC::init_robot_map() {
//   const auto original_map_ptr = this->orignal_map->get_map_ptr();
//   const auto current_map_ptr = this->current_map->get_map_ptr();
//   const auto map_size =
//       this->orignal_map->width * this->orignal_map->height * sizeof(char);

//   std::copy_n(original_map_ptr, map_size, current_map_ptr);
// }

// void RobotC::select_goods() {

//   if (this->had_goods) {
//     return;
//   }

//   // 机器人当前位置
//   auto robot_pos = this->pos;
//   // 机器人当前地图
//   auto map = this->current_map.get();

//   // 所有货物

//   // 计算所有货物的价值
//   std::vector<float> goods_values;
//   for (auto &goods : *goods_list) {
//     goods_values.push_back(goods.calc_goods_value(robot_pos));
//   }

//   // 选择价值最大的货物, 并删除货物
//   auto max_goods_iter =
//       std::max_element(goods_list->begin(), goods_list->end(),
//                        [&robot_pos](GoodsC &a, GoodsC &b) {
//                          auto a_value = a.calc_goods_value(robot_pos);
//                          auto b_value = b.calc_goods_value(robot_pos);
//                          return a_value < b_value;
//                        });

//   auto max_goods = GoodsC(*max_goods_iter);
//   goods_list->erase(max_goods_iter);

//   // 计算机器人到货物的路径
//   auto goods_pos = max_goods.pos;

//   auto a_path = AStarPath(robot_pos, goods_pos, *current_map.get());

//   a_path.search_path();
//   auto serched_path = a_path.get_path();

//   if (serched_path.has_value()) {
//     this->path = serched_path.value();
//     this->path_valid = true;
//   } else {
//     this->path_valid = false;
//   }
// }

// void RobotC::go_to_berth() {
//   if (!this->had_goods) {
//     return;
//   }

//   // 机器人当前位置
//   auto robot_pos = this->pos;
//   // 机器人当前地图
//   auto map = this->current_map.get();

//   // 所有泊位

//   std::vector<Point> selected_path;
//   int selected_berth_id = 0;
//   int selected_berth_dis = 20000;
//   bool selected_path_valid = false;

//   for (auto &berth : *berth_list) {
//     // 计算机器人到泊位的路径
//     auto berth_pos = berth.pos;

//     auto a_path = AStarPath(robot_pos, berth_pos, *current_map.get());

//     a_path.search_path();
//     auto serched_path = a_path.get_path();

//     if (serched_path.has_value()) {
//       if (serched_path.value().size() < selected_berth_dis) {
//         selected_berth_dis = serched_path.value().size();
//         selected_path = serched_path.value();
//         selected_berth_id = berth.id;
//         selected_path_valid = true;
//       }
//     }
//   }

//   if (selected_path_valid) {
//     this->path = selected_path;
//     this->path_valid = true;
//   } else {
//     this->path_valid = false;
//   }
// }
