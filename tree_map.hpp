#pragma once
#include <cstdio>
#include <list>
#include <unordered_map>
#include <vector>


class TreeMap {


public:
    struct TreeNode {
        int id;
        std::vector<TreeNode *> children{};

        TreeNode() :
            id(-1) {}

        explicit TreeNode(const int x) :
            id(x) {}
    };

    std::unordered_map<int, TreeNode *> node_table{};
    std::vector<TreeNode *> roots{};


    explicit TreeMap(const std::unordered_map<int, int> &node_map) {
        std::unordered_set<int> unique_nodes{};
        for (const auto &pair : node_map) {
            if (unique_nodes.find(pair.first) != unique_nodes.end()) {
                log_assert(false, "node id[%d] is not unique", pair.first);
            }
            unique_nodes.emplace(pair.first);
        }
        parse_tree(node_map);
    }

    ~TreeMap() {
        for (const auto &pair : node_table) {
            delete pair.second;
        }
    }

    std::vector<TreeNode *> &get_roots() {
        return roots;
    }

    void print_tree() {
        for (const auto &root : roots) {
            printf("root %d\n", root->id);
            tree_bfs_for_each(root, [](TreeNode *node) {
                printf(" %d ", node->id);
            });
            printf("\n");
        }
    }


    /**
     * @brief 广度优先遍历所有树
     * @param func 遍历树的回调函数
     */
    void trees_bfs_for_each(const std::function<void(TreeNode *)> &func) {
        for (const auto &root : roots) {
            tree_bfs_for_each(root, func);
        }
    }

    static void tree_bfs_for_each(TreeNode *tree, const std::function<void(TreeNode *)> &func) {
        std::list<TreeNode *> q;
        q.push_back(tree);
        while (!q.empty()) {
            TreeNode *node = q.front();
            func(node);
            q.pop_front();
            for (const auto &child : node->children) {
                q.push_back(child);
            }
        }
    }

    /**
     * @brief 根据节点关系网解析树
     * @param node_map 节点之间关系网， first 为当前节点 id, second 为父节点 id
     * @return 返回树的根节点，可以根据每个树的根节点遍历这个树，可能有多科树
     */
    void parse_tree(const std::unordered_map<int, int> &node_map) {
        // Step 1: Create nodes
        for (const auto &pair : node_map) {
            int child_id = pair.first;
            int parent_id = pair.second;

            if (node_table.find(child_id) == node_table.end()) {
                node_table[child_id] = new TreeNode(child_id);
            }
            if (node_table.find(parent_id) == node_table.end()) {
                node_table[parent_id] = new TreeNode(parent_id);
            }
            // Link child to parent
            node_table[parent_id]->children.push_back(node_table[child_id]);
        }

        // Step 2: Find root nodes
        for (const auto &pair : node_table) {
            int node_id = pair.first;
            int parent_id = pair.second->id;

            if (node_map.find(parent_id) == node_map.end()) {
                roots.push_back(pair.second);
            }
        }
    }
};





