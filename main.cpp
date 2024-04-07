#include <vector>

#include "config.h"
#include "log.h"
#include "mananger_new.hpp"

struct Node {
    int val;
    // 其他成员...
    std::string name{};

    Node(int v, std::string n) :
        val(v), name(n) {}
};

int ntest() {
    std::vector<Node> nodes = {Node(5, "i"), Node(2, "come"), Node(8, "from"), Node(3, "china")};

    // 创建节点索引数组
    std::vector<size_t> indices(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        indices[i] = i;
    }

    // 按照节点值的大小顺序排序索引数组，而不改变原始节点的顺序
    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
        return nodes[a].val < nodes[b].val;
    });

    // 遍历节点并修改其他值
    for (size_t i : indices) {
        // 对节点进行操作，例如修改其他成员的值
        printf("node[%s] val[%d]\n", nodes[i].name.c_str(), nodes[i].val);
        nodes[i].name += "_new";
    }

    // 打印节点
    for (const auto &node : nodes) {
        std::cout << node.val << " " << node.name << std::endl;
    }
    std::cout << std::endl;

    return 0;
}


int main() {
#ifdef LOG_ENABLE
    log_init("log.txt", 6);
#endif
    auto m = new ManagerNew();
    try {
        m->init_game();
        m->run_game();
    }
    catch (const std::exception &e) {

        log_fatal("exception:%s", e.what());
    }
    delete m;
    return 0;
}
