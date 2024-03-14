#include "berth.h"
#include "game_map.h"
#include "goods.h"
#include "point.h"
#include <list>
#include <memory>
#include <optional>
#include <stack>
#include <vector>
enum RobotDrirection { RIGHT, LEFT, UP, DOWN };

enum RobotFSM {
  IDLE,
  SELECT_GOODS,
  GO_TO_GOODS,
  SELECT_BERTH,
  GO_TO_BERTH,
  DEAD
};

struct Robot {
  int goods, x, y, status;
  Robot() {}
};

class RobotC {

public:
  // std::vector<Point> path;       // 机器人路径
  // bool path_valid = false;       // 机器人路径是否有效
  // bool berth_path_valid = false; // 机器人路径是否有效
  // bool goods_path_valid = false; // 机器人路径是否有效

  // std::shared_ptr<GameMap> orignal_map;           // 初始化的地图
  // std::shared_ptr<std::list<GoodsC>> goods_list;  // 地图上的所有货物
  // std::shared_ptr<std::vector<Berth>> berth_list; // 地图上的所有泊位

  // std::unique_ptr<GameMap> current_map; // 机器人视角的当前地图

  enum RobotNextPointType { GOODS, BERTH, EMPTY, COLLISION, RECOVER };

  int id;                               // 机器人编号
  Point pos;                            // 机器人位置
  std::optional<Point> next_pos;        // 机器人下一步位置
  RobotNextPointType next_type = EMPTY; // 机器人下一步位置类型
  std::stack<Point>
      collision_path; // 机器人执行的规避碰撞路径(使用 stack 可以回溯)
  bool had_goods; // 机器人是否有货物
  int status;     // 机器人状态

  RobotFSM fsm_status = IDLE; // 机器人状态机状态
  std::optional<GoodsC> target_goods = std::nullopt;
  std::optional<Berth> target_berth = std::nullopt;
  std::vector<Point> path; // 机器人路径
  bool path_valid = false; // 机器人路径是否有效

  explicit RobotC(int id, Point pos);
  // void init_robot_map();
  // void select_goods();
  // void go_to_berth();

  ~RobotC();
};