#include "point.hpp"
#include <list>
#include <memory>
#include <optional>
#include <stack>

enum RobotDrirection { RIGHT, LEFT, UP, DOWN };

class Robot {

public:
  enum RobotNextPointType { GOODS, BERTH, EMPTY, COLLISION, RECOVER };

  int id;         // 机器人编号
  Point pos;      // 机器人位置
  bool had_goods; // 机器人是否有货物
  int status;     // 机器人状态

  explicit Robot() {
    this->id = 0;
    this->pos = Point(0, 0);
    this->had_goods = false;
    this->status = 0;
  }

  ~Robot() {}
};