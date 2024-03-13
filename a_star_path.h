#pragma once

#include "game_map.h"
#include "point.h"
#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

// frontier = PriorityQueue()
// frontier.put(start, 0)
// came_from = dict()
// cost_so_far = dict()
// came_from[start] = None
// cost_so_far[start] = 0

// while not frontier.empty():
//    current = frontier.get()

//    if current == goal:
//       break

//    for next in graph.neighbors(current):
//       new_cost = cost_so_far[current] + graph.cost(current, next)
//       if next not in cost_so_far or new_cost < cost_so_far[next]:
//          cost_so_far[next] = new_cost
//          priority = new_cost + heuristic(goal, next)
//          frontier.put(next, priority)
//          came_from[next] = current

class AStarPath {

private:
  Point start;
  Point goal;
  std::shared_ptr<GameMap> graph;

  std::priority_queue<PrioItem> frontier;
  std::unordered_map<Point, Point> came_from;
  std::unordered_map<Point, int> cost_so_far;

public:
  std::vector<Point> path;
  bool found_path = true;
  explicit AStarPath(Point start, Point goal, std::shared_ptr<GameMap> graph);
  ~AStarPath();
  void search_path();
  void bfs_search_path();
  std::optional<std::vector<Point>> get_path();
  int heuristic(Point a, Point b);
  void print_path();
};