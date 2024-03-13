#pragma once

#include "point.h"
struct Berth {
  Point pos;
  int transport_time;
  int loading_speed;
  int id;
  Berth() {}
};