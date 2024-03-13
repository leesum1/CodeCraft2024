

#include "a_star_path.h"
#include "game_map.h"
#include "log.h"
#include "point.h"
#include <cassert>
#include <cstdio>
#include <iostream>
#include <list>
#include <vector>

AStarPath::AStarPath(Point start, Point goal, std::shared_ptr<GameMap> graph) {
  this->start = start;
  this->goal = goal;
  this->graph = graph;
  this->frontier.emplace(start, 0);
  this->came_from[start] = Point(-1, -1); // came_from[start] = None
  this->cost_so_far[start] = 0;
}

AStarPath::~AStarPath() {}

int AStarPath::heuristic(Point a, Point b) { return a.manhattan_distance(b); }
constexpr int dx[4] = {1, 0, -1, 0};
constexpr int dy[4] = {0, 1, 0, -1};

void AStarPath::bfs_search_path() {
  std::queue<Point> q;
  q.push(start);
  came_from[start] = Point(-1, -1);
  cost_so_far[start] = 0;

  while (!q.empty()) {
    auto current = q.front();
    q.pop();

    if (current == goal) {
      break;
    }

    for (auto &next : graph->neighbors(current)) {
      if (cost_so_far.find(next) == cost_so_far.end()) {
        cost_so_far[next] = cost_so_far[current] + 1;
        q.push(next);
        came_from[next] = current;
      }
    }
  }
}

void AStarPath::search_path() {

  while (!frontier.empty()) {
    auto current = frontier.top();
    frontier.pop();

    if (current.pos == goal) {
      break;
    }

    for (auto &next : graph->neighbors(current.pos)) {
      // int new_cost = cost_so_far[current.pos] + graph.cost(current.pos,
      // next);

      int new_cost = cost_so_far[current.pos] + 1;
      if (cost_so_far.find(next) == cost_so_far.end() ||
          new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        int priority = new_cost + goal.manhattan_distance(next);
        frontier.emplace(next, priority);
        came_from[next] = current.pos; // came_from[next] = current
      }
    }
  }

  // find the path
  Point current_point = goal;
  found_path = true;
  while (current_point != start) {
    path.push_back(current_point);
    try {
      current_point = came_from.at(current_point);
    } catch (const std::out_of_range &e) {
      this->graph = nullptr;
      log_warn("no path found from (%d, %d) to (%d, %d)", start.x, start.y,
               goal.x, goal.y);
      found_path = false;
      // print_path();
      return;
    }
  }
}

void AStarPath::print_path() {
  for (auto it = path.rbegin(); it != path.rend(); it++) {
    log_raw("(%d, %d)\n", it->x, it->y);
  }
}

std::optional<std::vector<Point>> AStarPath::get_path() {
  if (found_path) {
    return path;
  }
  return std::nullopt;
}

template <typename T> class MyPriorityQueue : public std::priority_queue<T> {
public:
  void clear() { this->c = std::vector<T>(); }
};

static MyPriorityQueue<PrioItem> frontier;
static std::unordered_map<Point, Point> came_from;
static std::unordered_map<Point, int> cost_so_far;
static std::vector<Point> final_path;

void c_astar_init() {
  // came_from.reserve(1024 * 8);
  // cost_so_far.reserve(1024 * 8);
  // final_path.reserve(200);
}

std::vector<Point> c_astar(Point start, Point goal,
                           std::shared_ptr<GameMap> graph) {

  frontier.clear();
  came_from.clear();
  cost_so_far.clear();
  final_path.clear();

  frontier.emplace(start, 0);
  came_from[start] = Point(-1, -1); // came_from[start] = None
  cost_so_far[start] = 0;

  while (!frontier.empty()) {
    auto current = frontier.top();
    frontier.pop();

    if (current.pos == goal) {
      break;
    }

    for (auto &next : graph->neighbors(current.pos)) {
      // int new_cost = cost_so_far[current.pos] + graph.cost(current.pos,
      // next);

      int new_cost = cost_so_far[current.pos] + 1;
      if (cost_so_far.find(next) == cost_so_far.end() ||
          new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        int priority = new_cost + goal.manhattan_distance(next);
        frontier.emplace(next, priority);
        came_from[next] = current.pos; // came_from[next] = current
      }
    }
  }

  // find the path
  Point current_point = goal;
  bool found_path = true;
  while (current_point != start) {
    final_path.push_back(current_point);
    try {
      current_point = came_from.at(current_point);
    } catch (const std::out_of_range &e) {
      printf("no path found from (%d, %d) to (%d, %d)", start.x, start.y,
             goal.x, goal.y);
      found_path = false;
      return final_path;
    }
  }

  return final_path;
}