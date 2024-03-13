#include "a_star_path.h"
#include "point.h"
#include <cstdio>
#include <sys/time.h> //引入头文件

void a_star_test() {
  auto map = std::make_shared<GameMap>(10, 10);
  char map_data[10][10] = {
      {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
      {'#', '.', '.', '.', '.', '.', '.', '.', '.', '#'},
      {'#', '.', '#', '#', '#', '#', '#', '#', '.', '#'},
      {'#', '.', '.', '.', '.', '.', '.', '.', '.', '#'},
      {'#', '.', '#', '#', '#', '#', '#', '#', '.', '#'},
      {'#', '.', '.', '.', '.', '.', '.', '.', '.', '#'},
      {'#', '.', '#', '#', '#', '#', '#', '#', '.', '#'},
      {'#', '.', '.', '.', '.', '.', '.', '.', '.', '#'},
      {'#', '.', '#', '#', '#', '#', '#', '#', '.', '#'},
      {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
  };

  char map_data2[10][10] = {
      {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
      {'#', '.', '.', '.', '#', '.', '.', '.', '.', '#'},
      {'#', '.', '#', '.', '#', '.', '#', '#', '.', '#'},
      {'#', '.', '#', '.', '.', '.', '#', '#', '.', '#'},
      {'#', '.', '#', '#', '#', '.', '#', '#', '.', '#'},
      {'#', '.', '.', '.', '.', '.', '.', '.', '.', '#'},
      {'#', '#', '#', '#', '.', '#', '#', '#', '.', '#'},
      {'#', '.', '.', '.', '.', '#', '.', '.', '.', '#'},
      {'#', '.', '#', '#', '#', '#', '#', '.', '.', '#'},
      {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
  };

  map->set_map((char *)map_data2);

  auto a_path = AStarPath(Point(4, 5), Point(7, 8), map);
  a_path.search_path();
  map->print_map();
  a_path.print_path();
}

void a_star_test2() {
  auto map = std::make_shared<GameMap>(200, 200);

  char *map_data = new char[200 * 200];

  // 打开 maps/map1.txt
  FILE *fp = fopen(
      "/home/leesum/Documents/huawei_soft2024/C++/C++/maps/map1.txt", "r");
  if (fp == NULL) {
    perror("Error opening file");
    return;
  }
  // 读取一行
  char *line = new char[2048];
  for (int i = 0; i < 200; i++) {
    fgets(line, 2048, fp);
    for (int j = 0; j < 200; j++) {
      map_data[i * 200 + j] = line[j];
    }
  }

  fclose(fp);
  delete[] line;
  map->set_map(map_data);
  // (64, 75) to (130, 83)(36, 173) to (100, 150)
  struct timeval t1, t2;
  double timeuse;
  gettimeofday(&t1, NULL);

  for (int i = 0; i < 1000; i++) {
    auto a_path = AStarPath(Point(189, 14), Point(136, 77), map);
    a_path.bfs_search_path();
  }

  extern void c_astar_init();
  void c_astar_init();

  // for (int i = 0; i < 1000; i++) {
  //   extern std::vector<Point> c_astar(Point start, Point goal,
  //                                     std::shared_ptr<GameMap> graph);
  //   // (189,14) (136,77)
  //   auto f_path = c_astar(Point(189, 14), Point(136, 77), map);

  //   // for (auto it = f_path.rbegin(); it != f_path.rend(); it++) {
  //   //   printf("(%d, %d)\n", it->x, it->y);
  //   // }
  // }

  gettimeofday(&t2, NULL);
  timeuse =
      (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec) / 1000000.0;
  printf("time: %f", timeuse);

  // map->print_map();
  // a_path.print_path();
  delete[] map_data;
}